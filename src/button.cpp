#include <Arduino.h>
#include "trmnl_log.h"
#include <config.h>
#include "button.h"

ButtonPressResult read_button_presses()
{
  auto time_start = millis();
  Log_info("Button time=%d: start", time_start);
  ButtonPressResult bpr = NoAction;

  while (digitalRead(PIN_INTERRUPT) == LOW && millis() - time_start < BUTTON_SOFT_RESET_TIME) // while button held
  {
    delay(10); // can save power if configured correctly
  }
  auto elapsed = millis() - time_start;
  if (elapsed >= BUTTON_SOFT_RESET_TIME) {
      Log_info("Button time=%d detected extra-long press", elapsed);
      bpr = SoftReset;
  } else if (elapsed > BUTTON_HOLD_TIME) {
      Log_info("Button time=%d detected long press", elapsed);
      bpr = LongPress;
  } else if (elapsed > BUTTON_MEDIUM_HOLD_TIME) {
      Log_info("Button time=%d detected double-click", elapsed);
      bpr = DoubleClick;
  } else {
      Log_info("Button time=%d detected no-action", elapsed);
  }
  return bpr;
}

const char *ButtonPressResultNames[] = {
    "LongPress",
    "DoubleClick",
    "NoAction",
    "SoftReset"};