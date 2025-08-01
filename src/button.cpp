#include <Arduino.h>
#include "trmnl_log.h"
#include <config.h>
#include "button.h"

ButtonPressResult read_button_presses()
{
  bool sampled_high = false;
  auto time_start = millis();
  Log_info("Button time=%d: start", time_start);
  while (1)
  {
    auto elapsed = millis() - time_start;
    auto pin = digitalRead(PIN_INTERRUPT);
    if(pin == LOW && elapsed > BUTTON_SOFT_RESET_TIME){
      return SoftReset;
    }
    else if (pin == HIGH && elapsed > BUTTON_HOLD_TIME)
    {
      Log_info("Button time=%d pin=%d: detected long press", elapsed, pin);
      return LongPress;
    }
    else if (pin == HIGH && !sampled_high)
    {
      Log_info("Button time=%d pin=%d: NOT reset; waiting for double-click", elapsed, pin);
      sampled_high = true;
    }
    else if (pin == LOW && sampled_high)
    {
      Log_info("Button time=%d pin=%d: detected double-click", elapsed, pin);
      return DoubleClick;
    }
    else if (pin == HIGH && (elapsed > 2000
       && elapsed < BUTTON_HOLD_TIME))
    {
      Log_info("Button time=%d pin=%d: detected no-action", elapsed, pin);
      return NoAction;
    }
  }
  return NoAction;
}

const char *ButtonPressResultNames[] = {
    "LongPress",
    "DoubleClick",
    "NoAction",
    "SoftReset"};