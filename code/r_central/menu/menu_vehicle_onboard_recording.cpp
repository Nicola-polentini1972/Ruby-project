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

#include "menu.h"
#include "menu_vehicle_onboard_recording.h"
#include "menu_item_select.h"
#include "menu_item_text.h"

MenuVehicleOnboardRecording::MenuVehicleOnboardRecording(void)
:Menu(MENU_ID_VEHICLE_ONBOARD_RECORDING, L("Onboard Video Recording"), NULL)
{
   m_Width = 0.32;
   m_xPos = menu_get_XStartPos(m_Width); m_yPos = 0.3;
}

MenuVehicleOnboardRecording::~MenuVehicleOnboardRecording()
{
}

void MenuVehicleOnboardRecording::onShow()
{
   int iTmp = getSelectedMenuItemIndex();
   addItems();
   Menu::onShow();
   if ( iTmp >= 0 )
      m_SelectedIndex = iTmp;
   if ( m_SelectedIndex >= m_ItemsCount )
      m_SelectedIndex = m_ItemsCount-1;
}

void MenuVehicleOnboardRecording::addItems()
{
   int iTmp = getSelectedMenuItemIndex();
   removeAllItems();

   m_IndexEnableOnArm = -1;

   if ( (NULL == g_pCurrentModel) || (! g_pCurrentModel->hasCamera()) )
   {
      addMenuItem(new MenuItemText(L("This vehicle doesn't have a camera. Onboard recording can't be used.")));
      return;
   }

   if ( ! g_pCurrentModel->isRunningOnRadxaHardware() )
   {
      addMenuItem(new MenuItemText(L("Onboard recording is only available on Radxa based vehicles.")));
      return;
   }

   m_pItemsSelect[0] = new MenuItemSelect(L("Start Recording on Arm"), L("Automatically starts saving the video stream to the vehicle's SD card when the vehicle arms, and stops it when it disarms."));
   m_pItemsSelect[0]->addSelection(L("Disabled"));
   m_pItemsSelect[0]->addSelection(L("Enabled"));
   m_pItemsSelect[0]->setIsEditable();
   m_IndexEnableOnArm = addMenuItem(m_pItemsSelect[0]);
   m_pItemsSelect[0]->setSelectedIndex(g_pCurrentModel->onboard_recording_params.uEnabled ? 1 : 0);

   if ( iTmp >= 0 )
      m_SelectedIndex = iTmp;
   if ( m_SelectedIndex >= m_ItemsCount )
      m_SelectedIndex = m_ItemsCount-1;
}

void MenuVehicleOnboardRecording::valuesToUI()
{
   addItems();
}

void MenuVehicleOnboardRecording::Render()
{
   RenderPrepare();
   float yTop = RenderFrameAndTitle();
   float y = yTop;
   for( int i=0; i<m_ItemsCount; i++ )
      y += RenderItem(i,y);
   RenderEnd(yTop);
}

void MenuVehicleOnboardRecording::sendParams()
{
   if ( -1 == m_IndexEnableOnArm )
      return;

   onboard_recording_params_t params;
   memcpy(&params, &(g_pCurrentModel->onboard_recording_params), sizeof(onboard_recording_params_t));
   params.uEnabled = (u8) m_pItemsSelect[0]->getSelectedIndex();

   if ( params.uEnabled == g_pCurrentModel->onboard_recording_params.uEnabled )
      return;

   if ( g_pCurrentModel->is_spectator )
   {
      memcpy(&(g_pCurrentModel->onboard_recording_params), &params, sizeof(onboard_recording_params_t));
      saveControllerModel(g_pCurrentModel);
      valuesToUI();
   }
   else if ( ! handle_commands_send_to_vehicle(COMMAND_ID_SET_ONBOARD_RECORDING_PARAMS, 0, (u8*)&params, sizeof(onboard_recording_params_t)) )
      valuesToUI();
}

void MenuVehicleOnboardRecording::onSelectItem()
{
   Menu::onSelectItem();
   if ( (-1 == m_SelectedIndex) || (m_pMenuItems[m_SelectedIndex]->isEditing()) )
      return;

   if ( handle_commands_is_command_in_progress() )
   {
      handle_commands_show_popup_progress();
      return;
   }

   if ( m_IndexEnableOnArm == m_SelectedIndex )
   {
      sendParams();
      return;
   }
}
