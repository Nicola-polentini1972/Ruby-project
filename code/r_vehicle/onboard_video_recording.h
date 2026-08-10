#pragma once

#include "../base/base.h"

// Onboard video recording: writes a copy of the already H264/H265 encoded video stream
// (same data as sent over radio) to a local file on the vehicle's SD card.
// Only implemented on HW_PLATFORM_RADXA; a no-op stub is used on other platforms.

void onboard_video_recording_init();
void onboard_video_recording_uninit();

// Called (locally, from the router's local control packet handling) when the vehicle
// arms/disarms. Starts/stops the recording if enabled in the current Model settings.
// Must be non-blocking: only signals a request, the actual (possibly slow) file/pipe
// setup happens on a dedicated thread.
void onboard_video_recording_notify_armed(bool bArmed);

// Called from the video capture hot path (video_sources.cpp) with the same NAL data
// that is being sent over radio. Must be O(1) in the common case (non-blocking).
void onboard_video_recording_on_new_data(u8* pData, int iLength);

bool onboard_video_recording_is_recording();

void onboard_video_recording_periodic_loop();
