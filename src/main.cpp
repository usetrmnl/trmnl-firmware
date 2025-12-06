#include <Arduino.h>
#include "bl.h"
#include "esp_ota_ops.h" 
#include "qa.h"


void setup()
{
  
  bool testPassed = checkIfAlreadyPassed();
  
  if (!testPassed) {
    startQA();
  }
  esp_ota_mark_app_valid_cancel_rollback();
  bl_init();
}

void loop()
{
  bl_process();
}
