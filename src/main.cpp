#include <Arduino.h>
#include "bl.h"
#include "esp_ota_ops.h" 
#include "qa.h"

#ifdef BOARD_TRMNL_X
#include "display.h"
#include "esp_sleep.h"
#include "filesystem.h"
#include "modem.h"
#include "logo_medium.h"

// TRMNL X setup (quite different from OG)
void setup()
{
#ifndef NO_QA

  bool isShipped = checkIfAlreadyShipped();
  if (!isShipped) {
   bool shipmentStarted = checkIfShipmentStarted();
   
   display_init();

   if (shipmentStarted && check_usb_power()) {
      // USB connected (e.g. battery died in transit and USB revived the device)
      enter_shipment_sleep();
      saveShipmentDone();
      ESP.restart();
   }
   else {
      filesystem_init();

      Serial.begin(115200);
      bool modemFlashed = checkIfModemFlashed();

      if (!modemFlashed) {
        Serial.println("[MODEM] Resetting modem...");
        modem_reset_target();
        delay(500);  // let modem reach bootloader

        display_show_msg(const_cast<uint8_t *>(logo_medium), MODEM_FLASHING);

        Modem modem(115200);
        String flashError;
        if (modem.flashFromFile("/system/factory_ESP32C5-4MB.bin", flashError)) {
          Serial.println("[MODEM] Factory flash complete.");
          saveModemFlashed();
        }
        else {
          Serial.println("[MODEM] Factory flash FAILED.");
          display_show_msg(const_cast<uint8_t *>(logo_medium), MODEM_FLASH_FAILED, flashError.c_str());
          delay(5000);
        }
      }
      else {
        Serial.println("[MODEM] Already flashed, skipping...");
      }
      // enableShipmentMode() will block until charger is connected
      enableShipmentMode();

      // Charger detected, save shipment status to preferences
      saveShipmentDone();
      ESP.restart();
   }
  }
#endif // NO_QA

  bl_init();
}
#else // TRMNL OG setup()
void setup()
{
  
  bool testPassed = checkIfAlreadyPassed();
  if (!testPassed) {
    startQA();
  }
  esp_ota_mark_app_valid_cancel_rollback();
  bl_init();
}
#endif // !BOARD_TRMNL_X

void loop()
{
  bl_process();
}
