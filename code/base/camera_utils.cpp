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

#include "../base/base.h"
#include "../base/config.h"
#include "../base/models.h"
#include <math.h>


int camera_get_active_camera_h264_slices(Model* pModel)
{
   if ( NULL == pModel )
      return 1;

   if ( hardware_board_is_openipc(pModel->hwCapabilities.uBoardType) )
      return pModel->video_params.iH264Slices;

   if ( pModel->isActiveCameraVeye() )
      return pModel->video_params.iH264Slices;

   if ( pModel->isActiveCameraCSICompatible() )
   if ( (pModel->video_params.iVideoWidth > 1280) || (pModel->video_params.iVideoHeight > 720) )
      return 1;
   return pModel->video_params.iH264Slices;
}

// Approximates the red/blue AWB correction gains needed to neutralize a light
// source of a given color temperature, using the Tanner Helland blackbody
// approximation to get that light's RGB tint, then inverting it around green.
void camera_get_awb_gains_for_color_temperature(u32 uColorTempK, float* pfGainR, float* pfGainB)
{
   if ( (NULL == pfGainR) || (NULL == pfGainB) )
      return;

   if ( uColorTempK < 1000 )
      uColorTempK = 1000;
   if ( uColorTempK > 12000 )
      uColorTempK = 12000;

   float fTemp = ((float)uColorTempK) / 100.0f;
   float fRed, fGreen, fBlue;

   if ( fTemp <= 66.0f )
      fRed = 255.0f;
   else
   {
      fRed = 329.698727446f * powf(fTemp - 60.0f, -0.1332047592f);
      if ( fRed < 0.0f ) fRed = 0.0f;
      if ( fRed > 255.0f ) fRed = 255.0f;
   }

   if ( fTemp <= 66.0f )
      fGreen = 99.4708025861f * logf(fTemp) - 161.1195681661f;
   else
      fGreen = 288.1221695283f * powf(fTemp - 60.0f, -0.0755148492f);
   if ( fGreen < 0.0f ) fGreen = 0.0f;
   if ( fGreen > 255.0f ) fGreen = 255.0f;

   if ( fTemp >= 66.0f )
      fBlue = 255.0f;
   else if ( fTemp <= 19.0f )
      fBlue = 0.0f;
   else
   {
      fBlue = 138.5177312231f * logf(fTemp - 10.0f) - 305.0447927307f;
      if ( fBlue < 0.0f ) fBlue = 0.0f;
      if ( fBlue > 255.0f ) fBlue = 255.0f;
   }

   if ( fRed < 1.0f ) fRed = 1.0f;
   if ( fBlue < 1.0f ) fBlue = 1.0f;

   *pfGainR = fGreen / fRed;
   *pfGainB = fGreen / fBlue;

   if ( *pfGainR < 0.1f ) *pfGainR = 0.1f;
   if ( *pfGainR > 5.0f ) *pfGainR = 5.0f;
   if ( *pfGainB < 0.1f ) *pfGainB = 0.1f;
   if ( *pfGainB > 5.0f ) *pfGainB = 5.0f;
}