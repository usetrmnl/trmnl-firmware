#include <Arduino.h>
#include "bl.h"
#include "esp_ota_ops.h"
#include "qa.h"
#include "display.h"
#include "esp_sleep.h"
#include "filesystem.h"
#include "modem.h"
#include "logo_medium.h"

void setup()
{

#ifndef NO_QA

  // bool testPassed = checkIfAlreadyPassed();

  // if (!testPassed) {
  //   startQA();
  // }

#ifdef BOARD_TRMNL_X

  bool isShipped = checkIfAlreadyShipped();
  if (!isShipped) {
   bool shipmentStarted = checkIfShipmentStarted();

   if (shipmentStarted && check_usb_power()) {
      // Battery died during shipping, now powered by USB charger
      // Skip shipment mode and mark as complete
      saveShipmentDone();
      ESP.restart();
   }
   else {
      Serial.begin(115200);
      display_init();
      filesystem_init();

      Serial.println("[MODEM] Resetting modem...");

      modem_reset_target();
      delay(500);  // let modem reach bootloader

      display_show_msg(const_cast<uint8_t *>(logo_medium),MODEM_FLASHING);

      Modem modem(115200);
      if (modem.flashFromFile("/system/factory_ESP32C5-4MB.bin")) {
        Serial.println("[MODEM] Factory flash complete.");
      } 
      else {
        Serial.println("[MODEM] Factory flash FAILED.");
      }

      display_show_msg(const_cast<uint8_t *>(logo_medium),READY_TO_SHIP);
      // enableShipmentMode() will block until charger is connected
      enableShipmentMode();

      // Charger detected, save shipment status to preferences
      saveShipmentDone();
      ESP.restart();
   }
  }
#endif // BOARD_TRMNL_X
#endif // NO_QA

//   esp_ota_mark_app_valid_cancel_rollback();

  bl_init();
}

void loop()
{
  bl_process();
}
