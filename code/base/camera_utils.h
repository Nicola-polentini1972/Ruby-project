#pragma once
#include "../base/base.h"
#include "../base/models.h"

int camera_get_active_camera_h264_slices(Model* pModel);
void camera_get_awb_gains_for_color_temperature(u32 uColorTempK, float* pfGainR, float* pfGainB);
