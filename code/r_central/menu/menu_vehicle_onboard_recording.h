#pragma once
#include "menu_objects.h"
#include "menu_item_select.h"

class MenuVehicleOnboardRecording: public Menu
{
   public:
      MenuVehicleOnboardRecording();
      virtual ~MenuVehicleOnboardRecording();
      virtual void Render();
      virtual void onShow();
      virtual void onSelectItem();
      virtual void valuesToUI();

   private:
      void addItems();
      void sendParams();

      MenuItemSelect* m_pItemsSelect[5];
      int m_IndexEnableOnArm;
      int m_IndexRecordOSD;
};
