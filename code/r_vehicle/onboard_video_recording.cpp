/*
    Ruby Licence
    Copyright (c) 2020-2025 Petru Soroaga petrusoroaga@yahoo.com
    All rights reserved.

    Redistribution and/or use in source and/or binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions and/or use of the source code (partially or complete) must retain
        the above copyright notice, this list of conditions and the following disclaimer
        in the documentation and/or other materials provided with the distribution.
        * Redistributions in binary form (partially or complete) must reproduce
        the above copyright notice, this list of conditions and the following disclaimer
        in the documentation and/or other materials provided with the distribution.
        * Copyright info and developer info must be preserved as is in the user
        interface, additions could be made to that info.
        * Neither the name of the organization nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.
        * Military use is not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE AUTHOR (PETRU SOROAGA) BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "onboard_video_recording.h"

#ifdef HW_PLATFORM_RADXA

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include "../base/base.h"
#include "../base/config.h"
#include "../base/models.h"
#include "../base/hardware.h"
#include "../base/hardware_procs.h"
#include "../base/hardware_files.h"
#include "../base/msp.h"
#include "../base/shared_mem.h"
#include "../radio/radiopackets2.h"

#include "shared_vars.h"

namespace
{
   bool s_bOnboardRecEnabled = false; // snapshot of the config flag, taken at start of a recording session
   volatile bool s_bOnboardRecShouldRun = false;   // set by notify_armed(true), cleared by notify_armed(false)
   volatile bool s_bOnboardRecThreadRunning = false;
   pthread_t s_OnboardRecThread;

   int s_iOnboardRecPipeWrite = -1;

   u8 s_uOnboardRecTempBuffer[64000];
   int s_iOnboardRecTempBufferFilled = 0;

   u32 s_uOnboardRecCurrentToken = 0x11111111;
   bool s_bOnboardRecFoundFirstNAL = false;

   u32 s_uOnboardRecDroppedBytes = 0;
   u32 s_uOnboardRecLastDropLogTime = 0;
}

void _onboard_video_recording_flush_temp_buffer()
{
   if ( (s_iOnboardRecPipeWrite <= 0) || (0 == s_iOnboardRecTempBufferFilled) )
      return;

   int iRes = write(s_iOnboardRecPipeWrite, s_uOnboardRecTempBuffer, s_iOnboardRecTempBufferFilled);
   if ( iRes != s_iOnboardRecTempBufferFilled )
      s_uOnboardRecDroppedBytes += (u32)(s_iOnboardRecTempBufferFilled - (iRes > 0 ? iRes : 0));
   s_iOnboardRecTempBufferFilled = 0;
}

// Reads the MSP-OSD screen buffer published by ruby_tx_telemetry (telemetry_msp.cpp) and,
// if it has a new frame, appends it to the .osd file, using the exact same binary format
// already produced station-side (code/r_station/rx_video_recording_data.cpp), so it can be
// read back by the existing video_playback.cpp without any changes there.
void _onboard_video_recording_try_write_osd_frame(int iFileOSD, shared_mem_msp_osd_screen* pSMOSD,
   bool* pbHeaderWritten, int* piLastConsumedFrameNumber, u32 uTimeStartMs, u32* puBytesWritten)
{
   if ( (iFileOSD < 0) || (NULL == pSMOSD) )
      return;

   if ( ! (*pbHeaderWritten) )
   {
      u8 uHeader[40];
      memset(uHeader, 0, sizeof(uHeader));
      switch ( pSMOSD->uMSPFlags & MSP_FLAGS_FC_TYPE_MASK )
      {
         case MSP_FLAGS_FC_TYPE_BETAFLIGHT: memcpy(uHeader, "BTFL", 4); break;
         case MSP_FLAGS_FC_TYPE_INAV: memcpy(uHeader, "INAV", 4); break;
         case MSP_FLAGS_FC_TYPE_PITLAB: memcpy(uHeader, "PITL", 4); break;
         case MSP_FLAGS_FC_TYPE_ARDUPILOT: memcpy(uHeader, "ARDU", 4); break;
         default: break;
      }
      int iRes = write(iFileOSD, uHeader, sizeof(uHeader));
      if ( iRes > 0 )
         (*puBytesWritten) += (u32)iRes;
      *pbHeaderWritten = true;
      // Same as station-side: the first call only writes the 40 byte header, no frame yet.
      return;
   }

   u32 uGen1 = pSMOSD->uGenerationCounter;
   int iFrameNo = pSMOSD->iLastDrawFrameNumber;
   if ( iFrameNo == (*piLastConsumedFrameNumber) )
      return;

   int iCols = (int) pSMOSD->uMSPOSDCols;
   if ( iCols <= 0 )
      iCols = DEFAULT_MSPOSD_RECORDING_COLS;

   u16 uBuffer[DEFAULT_MSPOSD_RECORDING_ROWS * DEFAULT_MSPOSD_RECORDING_COLS];
   int iBuffPos = 0;
   for( int y=0; y<DEFAULT_MSPOSD_RECORDING_ROWS; y++ )
   for( int x=0; x<DEFAULT_MSPOSD_RECORDING_COLS; x++ )
   {
      int iSrc = x + y*iCols;
      uBuffer[iBuffPos++] = ((iSrc >= 0) && (iSrc < MAX_MSP_CHARS_BUFFER)) ? pSMOSD->uScreenChars[iSrc] : 0;
   }

   if ( uGen1 != pSMOSD->uGenerationCounter )
      return; // Possible torn read: retry next pass, this frame number is not marked consumed.

   u32 uTimeMs = get_current_timestamp_ms() - uTimeStartMs;
   int iRes = write(iFileOSD, (u8*)&uTimeMs, sizeof(u32));
   if ( iRes > 0 )
      (*puBytesWritten) += (u32)iRes;
   iRes = write(iFileOSD, uBuffer, iBuffPos * sizeof(u16));
   if ( iRes > 0 )
      (*puBytesWritten) += (u32)iRes;

   *piLastConsumedFrameNumber = iFrameNo;
}

void* _thread_onboard_video_recording(void* argument)
{
   log_line("[OnboardVideoRecording-Th] Thread started.");
   hw_log_current_thread_attributes("onboard video recording");

   char szComm[256];
   sprintf(szComm, "mkdir -p %s", FOLDER_MEDIA);
   hw_execute_bash_command(szComm, NULL);
   sprintf(szComm, "chmod 777 %s", FOLDER_MEDIA);
   hw_execute_bash_command(szComm, NULL);

   const char* szExt = "h264";
   if ( (NULL != g_pCurrentModel) && (g_pCurrentModel->video_params.uVideoExtraFlags & VIDEO_FLAG_GENERATE_H265) )
      szExt = "h265";

   u32 uRecTimestampMs = get_current_timestamp_ms();

   char szFile[MAX_FILE_PATH_SIZE];
   snprintf(szFile, sizeof(szFile)/sizeof(szFile[0]), "%sonboard_rec_%llu.%s",
      FOLDER_MEDIA, (unsigned long long)uRecTimestampMs, szExt);

   bool bRecordOSD = (NULL != g_pCurrentModel) && (0 != g_pCurrentModel->onboard_recording_params.uRecordOSD);
   char szFileOSD[MAX_FILE_PATH_SIZE];
   snprintf(szFileOSD, sizeof(szFileOSD)/sizeof(szFileOSD[0]), "%sonboard_rec_%llu.osd",
      FOLDER_MEDIA, (unsigned long long)uRecTimestampMs);

   int iPipeRead = -1;
   int iRetries = 100;
   while ( iRetries > 0 )
   {
      iRetries--;
      iPipeRead = open(FIFO_RUBY_VEHICLE_VIDEO_STREAM_RECORDING, O_CREAT | O_RDONLY | O_NONBLOCK);
      if ( iPipeRead > 0 )
         break;
      hardware_sleep_ms(5);
   }
   if ( iPipeRead <= 0 )
   {
      log_softerror_and_alarm("[OnboardVideoRecording-Th] Failed to open recording pipe read endpoint: %s", FIFO_RUBY_VEHICLE_VIDEO_STREAM_RECORDING);
      s_bOnboardRecThreadRunning = false;
      return NULL;
   }

   iRetries = 100;
   while ( iRetries > 0 )
   {
      iRetries--;
      s_iOnboardRecPipeWrite = open(FIFO_RUBY_VEHICLE_VIDEO_STREAM_RECORDING, O_CREAT | O_WRONLY | O_NONBLOCK);
      if ( s_iOnboardRecPipeWrite > 0 )
         break;
      hardware_sleep_ms(5);
   }
   if ( s_iOnboardRecPipeWrite <= 0 )
   {
      log_softerror_and_alarm("[OnboardVideoRecording-Th] Failed to open recording pipe write endpoint: %s", FIFO_RUBY_VEHICLE_VIDEO_STREAM_RECORDING);
      close(iPipeRead);
      s_bOnboardRecThreadRunning = false;
      return NULL;
   }
   fcntl(s_iOnboardRecPipeWrite, F_SETPIPE_SZ, 2*1024*1024);

   int iOutputFile = open(szFile, O_CREAT | O_WRONLY | O_TRUNC, 0777);
   if ( -1 == iOutputFile )
   {
      log_softerror_and_alarm("[OnboardVideoRecording-Th] Failed to create recording file: %s", szFile);
      close(iPipeRead);
      close(s_iOnboardRecPipeWrite);
      s_iOnboardRecPipeWrite = -1;
      s_bOnboardRecThreadRunning = false;
      return NULL;
   }

   log_line("[OnboardVideoRecording-Th] Recording onboard video to: %s", szFile);

   int iOutputFileOSD = -1;
   shared_mem_msp_osd_screen* pSMOSD = NULL;
   bool bOSDHeaderWritten = false;
   int iOSDLastConsumedFrameNumber = -1;
   u32 uOSDBytesWritten = 0;
   if ( bRecordOSD )
   {
      iOutputFileOSD = open(szFileOSD, O_CREAT | O_WRONLY | O_TRUNC, 0777);
      if ( -1 == iOutputFileOSD )
         log_softerror_and_alarm("[OnboardVideoRecording-Th] Failed to create .osd recording file: %s", szFileOSD);
      else
      {
         log_line("[OnboardVideoRecording-Th] Recording onboard OSD data to: %s", szFileOSD);
         pSMOSD = shared_mem_msp_osd_screen_open_for_read();
         if ( NULL == pSMOSD )
            log_softerror_and_alarm("[OnboardVideoRecording-Th] Failed to open MSP OSD shared mem for reading, .osd file will have no data.");
      }
   }

   s_iOnboardRecTempBufferFilled = 0;
   s_uOnboardRecCurrentToken = 0x11111111;
   s_bOnboardRecFoundFirstNAL = false;
   s_uOnboardRecDroppedBytes = 0;
   s_uOnboardRecLastDropLogTime = get_current_timestamp_ms();

   u32 uTimeLastFreeSpaceCheck = get_current_timestamp_ms();
   u32 uTotalBytesWritten = 0;
   u8 uReadBuffer[64000];

   while ( (! g_bQuit) && s_bOnboardRecShouldRun )
   {
      u32 uTimeNow = get_current_timestamp_ms();

      if ( -1 != iOutputFileOSD )
         _onboard_video_recording_try_write_osd_frame(iOutputFileOSD, pSMOSD,
            &bOSDHeaderWritten, &iOSDLastConsumedFrameNumber, (u32)uRecTimestampMs, &uOSDBytesWritten);

      if ( uTimeNow > uTimeLastFreeSpaceCheck + 4000 )
      {
         uTimeLastFreeSpaceCheck = uTimeNow;
         int iFreeSpaceKb = hardware_get_free_space_kb();
         if ( (iFreeSpaceKb >= 0) && (iFreeSpaceKb < 50000) )
         {
            log_softerror_and_alarm("[OnboardVideoRecording-Th] Low free space on SD (%d Kb). Stopping onboard recording.", iFreeSpaceKb);
            break;
         }
      }

      if ( uTimeNow > s_uOnboardRecLastDropLogTime + 5000 )
      {
         s_uOnboardRecLastDropLogTime = uTimeNow;
         if ( s_uOnboardRecDroppedBytes > 0 )
         {
            log_softerror_and_alarm("[OnboardVideoRecording-Th] Dropped %u bytes in the last 5 seconds (write backpressure).", s_uOnboardRecDroppedBytes);
            s_uOnboardRecDroppedBytes = 0;
         }
      }

      fd_set fdSet;
      FD_ZERO(&fdSet);
      FD_SET(iPipeRead, &fdSet);
      struct timeval timeInput;
      timeInput.tv_sec = 0;
      timeInput.tv_usec = 40*1000;

      int iSelectResult = select(iPipeRead+1, &fdSet, NULL, NULL, &timeInput);
      if ( (! s_bOnboardRecShouldRun) || g_bQuit )
         break;
      if ( iSelectResult <= 0 )
         continue;

      int iReadSize = read(iPipeRead, uReadBuffer, sizeof(uReadBuffer)/sizeof(uReadBuffer[0]));
      if ( iReadSize <= 0 )
      {
         if ( iReadSize < 0 )
            hardware_sleep_ms(5);
         continue;
      }

      int iRes = write(iOutputFile, uReadBuffer, iReadSize);
      if ( iRes > 0 )
         uTotalBytesWritten += (u32)iRes;
   }

   log_line("[OnboardVideoRecording-Th] Stopping. Total bytes written: %u", uTotalBytesWritten);

   close(iOutputFile);
   close(iPipeRead);
   close(s_iOnboardRecPipeWrite);
   s_iOnboardRecPipeWrite = -1;

   if ( uTotalBytesWritten < 10000 )
   {
      log_line("[OnboardVideoRecording-Th] Recording too short, deleting file: %s", szFile);
      char szDel[600];
      snprintf(szDel, sizeof(szDel)/sizeof(szDel[0]), "rm -f %s", szFile);
      hw_execute_bash_command(szDel, NULL);
   }

   if ( -1 != iOutputFileOSD )
   {
      close(iOutputFileOSD);
      if ( NULL != pSMOSD )
         shared_mem_msp_osd_screen_close(pSMOSD);

      // Delete the .osd file together with a too-short video, and also when it never got
      // any actual OSD frame data (only the 40 byte header), e.g. no FC DisplayPort MSP.
      if ( (uTotalBytesWritten < 10000) || (uOSDBytesWritten <= 40) )
      {
         log_line("[OnboardVideoRecording-Th] Deleting incomplete/empty .osd file: %s", szFileOSD);
         char szDelOSD[600];
         snprintf(szDelOSD, sizeof(szDelOSD)/sizeof(szDelOSD[0]), "rm -f %s", szFileOSD);
         hw_execute_bash_command(szDelOSD, NULL);
      }
   }

   s_bOnboardRecThreadRunning = false;
   log_line("[OnboardVideoRecording-Th] Thread exit.");
   return NULL;
}

void _onboard_video_recording_start()
{
   if ( s_bOnboardRecThreadRunning )
      return;
   if ( (NULL == g_pCurrentModel) || (! g_pCurrentModel->onboard_recording_params.uEnabled) )
      return;
   if ( ! g_pCurrentModel->hasCamera() )
      return;
   if ( !(g_pCurrentModel->isActiveCameraCSICompatible() || g_pCurrentModel->isActiveCameraOpenIPC()) )
      return;

   s_bOnboardRecShouldRun = true;
   s_bOnboardRecThreadRunning = true;

   pthread_attr_t attr;
   hw_init_worker_thread_attrs(&attr, CORE_AFFINITY_OTHERS, 64000, SCHED_OTHER, 0, "onboard video recording");
   if ( 0 != pthread_create(&s_OnboardRecThread, &attr, &_thread_onboard_video_recording, NULL) )
   {
      log_softerror_and_alarm("[OnboardVideoRecording] Failed to create recording thread.");
      s_bOnboardRecShouldRun = false;
      s_bOnboardRecThreadRunning = false;
   }
   else
      log_line("[OnboardVideoRecording] Started onboard video recording thread.");
   pthread_attr_destroy(&attr);
}

void _onboard_video_recording_stop()
{
   s_bOnboardRecShouldRun = false;
}

void onboard_video_recording_init()
{
   s_bOnboardRecEnabled = false;
   s_bOnboardRecShouldRun = false;
   s_bOnboardRecThreadRunning = false;
   s_iOnboardRecPipeWrite = -1;
   s_iOnboardRecTempBufferFilled = 0;
   log_line("[OnboardVideoRecording] Init complete.");
}

void onboard_video_recording_uninit()
{
   if ( ! s_bOnboardRecThreadRunning )
      return;
   s_bOnboardRecShouldRun = false;
   u32 uTimeStart = get_current_timestamp_ms();
   while ( s_bOnboardRecThreadRunning )
   {
      if ( get_current_timestamp_ms() > uTimeStart + 2000 )
         break;
      hardware_sleep_ms(20);
   }
   log_line("[OnboardVideoRecording] Uninit complete.");
}

void onboard_video_recording_notify_armed(bool bArmed)
{
   if ( bArmed )
      _onboard_video_recording_start();
   else
      _onboard_video_recording_stop();
}

void onboard_video_recording_on_new_data(u8* pData, int iLength)
{
   if ( (! s_bOnboardRecThreadRunning) || (! s_bOnboardRecShouldRun) || (s_iOnboardRecPipeWrite <= 0) || (NULL == pData) || (iLength <= 0) )
      return;

   while ( (! s_bOnboardRecFoundFirstNAL) && (iLength > 0) )
   {
      s_uOnboardRecCurrentToken = (s_uOnboardRecCurrentToken << 8) | (*pData);
      pData++;
      iLength--;
      if ( s_uOnboardRecCurrentToken == 0x00000001 )
      {
         s_bOnboardRecFoundFirstNAL = true;
         u8 uHeader[4] = {0,0,0,1};
         int iRes = write(s_iOnboardRecPipeWrite, uHeader, 4);
         if ( iRes != 4 )
            s_uOnboardRecDroppedBytes += 4;
      }
   }
   if ( iLength <= 0 )
      return;

   if ( s_iOnboardRecTempBufferFilled + iLength > (int)(sizeof(s_uOnboardRecTempBuffer)/sizeof(s_uOnboardRecTempBuffer[0])) )
      _onboard_video_recording_flush_temp_buffer();

   if ( iLength > (int)(sizeof(s_uOnboardRecTempBuffer)/sizeof(s_uOnboardRecTempBuffer[0])) )
   {
      // Larger than the whole temp buffer: write directly, best effort.
      int iRes = write(s_iOnboardRecPipeWrite, pData, iLength);
      if ( iRes != iLength )
         s_uOnboardRecDroppedBytes += (u32)(iLength - (iRes > 0 ? iRes : 0));
      return;
   }

   memcpy(&(s_uOnboardRecTempBuffer[s_iOnboardRecTempBufferFilled]), pData, iLength);
   s_iOnboardRecTempBufferFilled += iLength;
}

bool onboard_video_recording_is_recording()
{
   return s_bOnboardRecThreadRunning;
}

void onboard_video_recording_periodic_loop()
{
   if ( s_bOnboardRecThreadRunning && (s_iOnboardRecTempBufferFilled > 0) )
      _onboard_video_recording_flush_temp_buffer();
}

#else // HW_PLATFORM_RADXA

void onboard_video_recording_init() {}
void onboard_video_recording_uninit() {}
void onboard_video_recording_notify_armed(bool bArmed) {}
void onboard_video_recording_on_new_data(u8* pData, int iLength) {}
bool onboard_video_recording_is_recording() { return false; }
void onboard_video_recording_periodic_loop() {}

#endif
