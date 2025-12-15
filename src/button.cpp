#include <Arduino.h>
#include "trmnl_log.h"
#include <config.h>
#include "button.h"

static unsigned long wait_for_button_release(unsigned long start_time) {
  while (digitalRead(PIN_INTERRUPT) == LOW && millis() - start_time < BUTTON_SOFT_RESET_TIME) {
    delay(10);
  }
  return millis() - start_time;
}

static ButtonPressResult classify_press_duration(unsigned long duration) {
  if (duration >= BUTTON_SOFT_RESET_TIME) {
    Log_info("Button time=%lu detected extra-long press", duration);
    return SoftReset;
  } else if (duration > BUTTON_HOLD_TIME) {
    Log_info("Button time=%lu detected long press", duration);
    return LongPress;
  } else if(duration > BUTTON_MEDIUM_HOLD_TIME){
    Log_info("Button time=%lu detected long press", duration);
    return DoubleClick;
  }
  return NoAction; 
}

static ButtonPressResult wait_for_second_press(unsigned long start_time) {
  auto release_time = millis();

  while (millis() - release_time < BUTTON_DOUBLE_CLICK_WINDOW) {
    if (digitalRead(PIN_INTERRUPT) == LOW) {
      auto second_press_start = millis();
      auto second_duration = wait_for_button_release(second_press_start);

      ButtonPressResult long_press_result = classify_press_duration(second_duration);
      if (long_press_result != NoAction) {
        return long_press_result;
      }

      Log_info("Button time=%lu detected double-click", millis() - start_time);
      return DoubleClick;
    }
    delay(10);
  }

  return ShortPress;
}

ButtonPressResult read_button_presses()
{
  auto time_start = millis();
  Log_info("Button time=%lu: start", time_start);

  if (digitalRead(PIN_INTERRUPT) == HIGH) {
    if (time_start < 2000) {
      Log_info("Button: already released at start (GPIO wakeup), waiting for second press");
      return wait_for_second_press(time_start);
    } else {
      Log_info("Button: waiting for button press");
      while (digitalRead(PIN_INTERRUPT) == HIGH) {
        delay(10);
      }
      time_start = millis();
    }
  }

  auto press_duration = wait_for_button_release(time_start);

  ButtonPressResult long_press_result = classify_press_duration(press_duration);
  if (long_press_result != NoAction) {
    return long_press_result;
  }

  if (press_duration > 50) {
    Log_info("Button: first press detected, waiting for second press");
    return wait_for_second_press(time_start);
  }

  return NoAction;
}

const char *ButtonPressResultNames[] = {
    "LongPress",
    "DoubleClick",
    "ShortPress",
    "SoftReset"};