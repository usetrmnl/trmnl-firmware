#include <Arduino.h>
#include "bl.h"
#include "esp_ota_ops.h" 

void setup()
{
  esp_ota_mark_app_valid_cancel_rollback();
  bl_init();
}

void loop()
{
  bl_process();
}
