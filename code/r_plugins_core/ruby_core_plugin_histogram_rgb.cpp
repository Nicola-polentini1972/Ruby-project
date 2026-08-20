// Licence: you can copy, edit, change or do whatever you wish with the code in this file
//
// RGB Histogram core plugin - live video companion to the "RGB Histogram" OSD plugin
// (code/r_plugins_osd/ruby_plugin_histogram_rgb.cpp).
//
// Ruby's own video pipeline (H264/etc from the camera, decoded for display) is not
// exposed to plugins anywhere in the SDK - core plugins only get frames they capture
// themselves. So, on the VEHICLE side, this plugin opens its own V4L2 capture device
// in a background thread, grabs a low res frame every HIST_CAPTURE_INTERVAL_MS,
// computes an RGB channel histogram from it, and sends the (small, ~800 byte) bin
// counts to the controller using the CORE_PLUGIN_CAPABILITY_DATA_STREAM channel.
// On the CONTROLLER side, received bins are written to a small binary file that the
// OSD histogram plugin polls and draws.
//
// IMPORTANT / hardware caveat: this only works if the vehicle exposes a raw YUYV
// V4L2 capture device (HIST_V4L2_DEVICE) separate from the main H264 encode path.
// That's true on many Raspberry Pi / USB camera setups (e.g. bcm2835-v4l2), but is
// NOT guaranteed on OpenIPC SoCs (Goke/SigmaStar), whose ISP is normally only
// reachable through the vendor's own encoder pipeline. Validate HIST_V4L2_DEVICE
// against `v4l2-ctl --list-devices` / `--list-formats` on your actual vehicle
// before relying on this - if the device can't be opened the plugin logs it and
// simply produces no live data (the OSD plugin falls back to its static/demo mode).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include "../public/ruby_core_plugin.h"
#include "../public/utils/core_plugins_utils.h"

// ---- configuration -------------------------------------------------------

#define HIST_V4L2_DEVICE "/dev/video0"
#define HIST_CAPTURE_WIDTH 160
#define HIST_CAPTURE_HEIGHT 120
#define HIST_CAPTURE_INTERVAL_MS 500
#define HIST_NUM_V4L2_BUFFERS 4
#define HIST_NUM_BINS 64
#define HIST_MAGIC 0x52474248u // 'RGBH'
#define HIST_VERSION 1

// Controller-side output file, read by the OSD histogram plugin.
// Defaults to the Radxa ground station layout; for a Raspberry Pi controller
// change this to "/home/pi/ruby/tmp/histogram_live.bin".
#define HIST_LIVE_FILE_PATH "/home/radxa/ruby/tmp/histogram_live.bin"

// ---- wire / file format ---------------------------------------------------

typedef struct
{
   u32 uMagic;
   u32 uVersion;
   u32 uNumBins;
   u32 uTimestampSec;
   u32 uHistR[HIST_NUM_BINS];
   u32 uHistG[HIST_NUM_BINS];
   u32 uHistB[HIST_NUM_BINS];
} __attribute__((packed)) HistogramWireData;

// ---- plugin state -----------------------------------------------------

static const char* g_szPluginNameHist = "RGB Histogram (live video)";
static const char* g_szUIDHist = "77QF3A-RGBHISTV-02Q1-RUBYX";

static u32 g_uRuntimeLocation = 0;
static u32 g_uAllocatedCapabilities = 0;

static pthread_t s_captureThread;
static bool s_bCaptureThreadStarted = false;
static volatile bool s_bStopCaptureThread = false;

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static HistogramWireData s_latestHistogram;
static bool s_bHaveNewHistogram = false;

static u32 s_uTxSegmentIndex = 0;
static u8 s_uTxBuffer[sizeof(HistogramWireData)];

#ifdef __cplusplus
extern "C" {
#endif

// ---- V4L2 capture (vehicle side only) -------------------------------------

struct V4L2Buffer
{
   void* pStart;
   size_t nLength;
};

static int s_iV4L2Fd = -1;
static V4L2Buffer s_v4l2Buffers[HIST_NUM_V4L2_BUFFERS];
static int s_nV4L2BuffersCount = 0;

static void _v4l2_close()
{
   if ( s_iV4L2Fd >= 0 )
   {
      enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(s_iV4L2Fd, VIDIOC_STREAMOFF, &type);
      for( int i=0; i<s_nV4L2BuffersCount; i++ )
         if ( s_v4l2Buffers[i].pStart != MAP_FAILED && s_v4l2Buffers[i].pStart != NULL )
            munmap(s_v4l2Buffers[i].pStart, s_v4l2Buffers[i].nLength);
      close(s_iV4L2Fd);
   }
   s_iV4L2Fd = -1;
   s_nV4L2BuffersCount = 0;
}

static bool _v4l2_open()
{
   s_iV4L2Fd = open(HIST_V4L2_DEVICE, O_RDWR);
   if ( s_iV4L2Fd < 0 )
   {
      char szBuff[256];
      snprintf(szBuff, sizeof(szBuff), "RGB Histogram: failed to open %s (%s)", HIST_V4L2_DEVICE, strerror(errno));
      core_plugin_util_log_line(szBuff);
      return false;
   }

   struct v4l2_format fmt;
   memset(&fmt, 0, sizeof(fmt));
   fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   fmt.fmt.pix.width = HIST_CAPTURE_WIDTH;
   fmt.fmt.pix.height = HIST_CAPTURE_HEIGHT;
   fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
   fmt.fmt.pix.field = V4L2_FIELD_NONE;

   if ( ioctl(s_iV4L2Fd, VIDIOC_S_FMT, &fmt) < 0 )
   {
      core_plugin_util_log_line("RGB Histogram: VIDIOC_S_FMT (YUYV) failed, device not usable");
      _v4l2_close();
      return false;
   }

   struct v4l2_requestbuffers req;
   memset(&req, 0, sizeof(req));
   req.count = HIST_NUM_V4L2_BUFFERS;
   req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   req.memory = V4L2_MEMORY_MMAP;

   if ( ioctl(s_iV4L2Fd, VIDIOC_REQBUFS, &req) < 0 )
   {
      core_plugin_util_log_line("RGB Histogram: VIDIOC_REQBUFS failed");
      _v4l2_close();
      return false;
   }

   s_nV4L2BuffersCount = (int)req.count;
   for( int i=0; i<s_nV4L2BuffersCount; i++ )
   {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if ( ioctl(s_iV4L2Fd, VIDIOC_QUERYBUF, &buf) < 0 )
      {
         core_plugin_util_log_line("RGB Histogram: VIDIOC_QUERYBUF failed");
         _v4l2_close();
         return false;
      }

      s_v4l2Buffers[i].nLength = buf.length;
      s_v4l2Buffers[i].pStart = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_iV4L2Fd, buf.m.offset);
      if ( s_v4l2Buffers[i].pStart == MAP_FAILED )
      {
         core_plugin_util_log_line("RGB Histogram: mmap failed");
         _v4l2_close();
         return false;
      }

      if ( ioctl(s_iV4L2Fd, VIDIOC_QBUF, &buf) < 0 )
      {
         core_plugin_util_log_line("RGB Histogram: VIDIOC_QBUF failed");
         _v4l2_close();
         return false;
      }
   }

   enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   if ( ioctl(s_iV4L2Fd, VIDIOC_STREAMON, &type) < 0 )
   {
      core_plugin_util_log_line("RGB Histogram: VIDIOC_STREAMON failed");
      _v4l2_close();
      return false;
   }

   core_plugin_util_log_line("RGB Histogram: V4L2 capture device opened");
   return true;
}

static inline void _yuv_to_rgb(int y, int u, int v, int* pR, int* pG, int* pB)
{
   int c = y - 16;
   int d = u - 128;
   int e = v - 128;

   int r = (298*c + 409*e + 128) >> 8;
   int g = (298*c - 100*d - 208*e + 128) >> 8;
   int b = (298*c + 516*d + 128) >> 8;

   *pR = r < 0 ? 0 : (r > 255 ? 255 : r);
   *pG = g < 0 ? 0 : (g > 255 ? 255 : g);
   *pB = b < 0 ? 0 : (b > 255 ? 255 : b);
}

static void _compute_histogram_from_yuyv(const unsigned char* pData, int nWidth, int nHeight)
{
   int nHist256R[256];
   int nHist256G[256];
   int nHist256B[256];
   memset(nHist256R, 0, sizeof(nHist256R));
   memset(nHist256G, 0, sizeof(nHist256G));
   memset(nHist256B, 0, sizeof(nHist256B));

   long nPixelPairs = ((long)nWidth * (long)nHeight) / 2;
   for( long i=0; i<nPixelPairs; i++ )
   {
      int y0 = pData[i*4+0];
      int u  = pData[i*4+1];
      int y1 = pData[i*4+2];
      int v  = pData[i*4+3];

      int r,g,b;
      _yuv_to_rgb(y0,u,v,&r,&g,&b);
      nHist256R[r]++; nHist256G[g]++; nHist256B[b]++;
      _yuv_to_rgb(y1,u,v,&r,&g,&b);
      nHist256R[r]++; nHist256G[g]++; nHist256B[b]++;
   }

   int nSamplesPerBin = 256/HIST_NUM_BINS;

   pthread_mutex_lock(&s_mutex);
   s_latestHistogram.uMagic = HIST_MAGIC;
   s_latestHistogram.uVersion = HIST_VERSION;
   s_latestHistogram.uNumBins = HIST_NUM_BINS;
   s_latestHistogram.uTimestampSec = (u32)time(NULL);
   for( int b=0; b<HIST_NUM_BINS; b++ )
   {
      u32 sumR=0, sumG=0, sumB=0;
      for( int i=0; i<nSamplesPerBin; i++ )
      {
         sumR += nHist256R[b*nSamplesPerBin+i];
         sumG += nHist256G[b*nSamplesPerBin+i];
         sumB += nHist256B[b*nSamplesPerBin+i];
      }
      s_latestHistogram.uHistR[b] = sumR;
      s_latestHistogram.uHistG[b] = sumG;
      s_latestHistogram.uHistB[b] = sumB;
   }
   s_bHaveNewHistogram = true;
   pthread_mutex_unlock(&s_mutex);
}

static void* _capture_thread_proc(void* pParam)
{
   while ( ! s_bStopCaptureThread )
   {
      if ( s_iV4L2Fd < 0 )
      {
         if ( ! _v4l2_open() )
         {
            for( int i=0; (i<50) && (!s_bStopCaptureThread); i++ )
               usleep(100*1000); // retry opening the device every ~5s
            continue;
         }
      }

      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if ( ioctl(s_iV4L2Fd, VIDIOC_DQBUF, &buf) < 0 )
      {
         core_plugin_util_log_line("RGB Histogram: VIDIOC_DQBUF failed, reopening device");
         _v4l2_close();
         continue;
      }

      _compute_histogram_from_yuyv((const unsigned char*)s_v4l2Buffers[buf.index].pStart, HIST_CAPTURE_WIDTH, HIST_CAPTURE_HEIGHT);

      ioctl(s_iV4L2Fd, VIDIOC_QBUF, &buf);

      for( int i=0; (i<(HIST_CAPTURE_INTERVAL_MS/10)) && (!s_bStopCaptureThread); i++ )
         usleep(10*1000);
   }

   _v4l2_close();
   return NULL;
}

// ---- controller side: write received histogram to file --------------------

static void _write_histogram_to_file(HistogramWireData* pHist)
{
   char szTmp[300];
   snprintf(szTmp, sizeof(szTmp), "%s.tmp", HIST_LIVE_FILE_PATH);

   FILE* fd = fopen(szTmp, "wb");
   if ( NULL == fd )
      return;
   fwrite(pHist, sizeof(HistogramWireData), 1, fd);
   fclose(fd);
   rename(szTmp, HIST_LIVE_FILE_PATH);
}

// ---- core plugin ABI --------------------------------------------------

u32 core_plugin_on_requested_capabilities()
{
   return CORE_PLUGIN_CAPABILITY_DATA_STREAM;
}

const char* core_plugin_get_name()
{
   return g_szPluginNameHist;
}

const char* core_plugin_get_guid()
{
   return g_szUIDHist;
}

int core_plugin_get_version()
{
   return 1;
}

int core_plugin_init(u32 uRuntimeLocation, u32 uAllocatedCapabilities)
{
   g_uRuntimeLocation = uRuntimeLocation;
   g_uAllocatedCapabilities = uAllocatedCapabilities;

   memset(&s_latestHistogram, 0, sizeof(s_latestHistogram));
   s_bHaveNewHistogram = false;
   s_bStopCaptureThread = false;

   if ( uRuntimeLocation & CORE_PLUGIN_RUNTIME_LOCATION_VEHICLE )
   {
      if ( 0 == pthread_create(&s_captureThread, NULL, _capture_thread_proc, NULL) )
         s_bCaptureThreadStarted = true;
      else
         core_plugin_util_log_line("RGB Histogram: failed to start capture thread");
   }

   core_plugin_util_log_line("RGB Histogram core plugin initialized");
   return 0;
}

void core_plugin_uninit()
{
   s_bStopCaptureThread = true;
   if ( s_bCaptureThreadStarted )
   {
      pthread_join(s_captureThread, NULL);
      s_bCaptureThreadStarted = false;
   }
   core_plugin_util_log_line("RGB Histogram core plugin uninitialized");
}

void core_plugin_on_rx_data(u8* pData, int iDataLength, int iDataType, u32 uSegmentIndex)
{
   if ( 0 == (g_uRuntimeLocation & CORE_PLUGIN_RUNTIME_LOCATION_CONTROLLER) )
      return;
   if ( iDataType != CORE_PLUGIN_TYPE_DATA_SEGMENT )
      return;
   if ( iDataLength != (int)sizeof(HistogramWireData) )
      return;

   HistogramWireData* pHist = (HistogramWireData*)pData;
   if ( pHist->uMagic != HIST_MAGIC )
      return;

   _write_histogram_to_file(pHist);
}

u32 core_plugin_has_pending_tx_data()
{
   if ( 0 == (g_uRuntimeLocation & CORE_PLUGIN_RUNTIME_LOCATION_VEHICLE) )
      return 0;

   u32 uReturn = 0;
   pthread_mutex_lock(&s_mutex);
   if ( s_bHaveNewHistogram )
   {
      s_uTxSegmentIndex++;
      memcpy(s_uTxBuffer, &s_latestHistogram, sizeof(HistogramWireData));
      s_bHaveNewHistogram = false;
      uReturn = s_uTxSegmentIndex;
   }
   pthread_mutex_unlock(&s_mutex);
   return uReturn;
}

u8* core_plugin_on_get_segment_data(u32 uSegmentIndex)
{
   return s_uTxBuffer;
}

int core_plugin_on_get_segment_length(u32 uSegmentIndex)
{
   return sizeof(HistogramWireData);
}

int core_plugin_on_get_segment_type(u32 uSegmentIndex)
{
   return CORE_PLUGIN_TYPE_DATA_SEGMENT;
}

#ifdef __cplusplus
}
#endif
