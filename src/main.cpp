#include <Arduino.h>
#include "bl.h"
#include "esp_ota_ops.h"
#include "qa.h"
#include "display.h"
#include "esp_sleep.h"

void setup()
{

  // bool testPassed = checkIfAlreadyPassed();

  // if (!testPassed) {
  //   startQA();
  // }

#ifdef BOARD_TRMNL_X

  bool isShipped = checkIfAlreadyShipped();
  if (!isShipped) {
//    bool shipmentStarted = checkIfShipmentStarted();

//    if (shipmentStarted && check_usb_power()) {
      // Battery died during shipping, now powered by USB charger
      // Skip shipment mode and mark as complete
      saveShipmentDone();
//      ESP.restart();
//    }
//    else {
      // enableShipmentMode() will block until charger is connected
//      enableShipmentMode();

      // Charger detected, save shipment status to preferences
//      saveShipmentDone();
//      ESP.restart();
//    }
  }
#endif
//   esp_ota_mark_app_valid_cancel_rollback();


  bl_init();
}

void loop()
{
  bl_process();
}
