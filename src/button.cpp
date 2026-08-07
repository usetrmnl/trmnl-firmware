#include "button.h"

#include <Arduino.h>
#include <config.h>
#include <misc/buzzer.h>

#include "trmnl_log.h"

static unsigned long wait_for_button_release(unsigned long start_time) {
  pinMode(PIN_INTERRUPT, INPUT);
#ifdef SHIP_MODE_SUPPORTED
  // Boards that support user-triggered shipping mode allow the hold to run all
  // the way to BUTTON_SHIP_MODE_TIME. Feedback is staged during the hold:
  //   5 s  -> 2 beeps (long press / WiFi reset)
  //   15 s -> 3 beeps (soft reset / credentials reset)
  //   30 s -> 4 beeps (shipping mode)
  bool hold_buzzer_fired = false;
  bool soft_reset_buzzer_fired = false;
  while (digitalRead(PIN_INTERRUPT) == LOW && millis() - start_time < BUTTON_SHIP_MODE_TIME) {
    unsigned long held = millis() - start_time;
    if (!hold_buzzer_fired && held >= BUTTON_HOLD_TIME) {
      buzzer().beepPattern(2, 100, 100);
      hold_buzzer_fired = true;
    }
    if (!soft_reset_buzzer_fired && held >= BUTTON_SOFT_RESET_TIME) {
      buzzer().beepPattern(3, 100, 100);
      soft_reset_buzzer_fired = true;
    }
    delay(10);
  }
  if (millis() - start_time >= BUTTON_SHIP_MODE_TIME) {
    // Shipping-mode threshold reached, four beeps to signal the user can release.
    buzzer().beepPattern(4, 120, 120);
  } else if (!hold_buzzer_fired) {
    // Single beep on release for a short press (e.g. the refresh trigger), unless
    // the longer hold pattern already fired.
    buzzer().beep(100);
  }
  return millis() - start_time;
#else
  bool hold_buzzer_fired = false;
  while (digitalRead(PIN_INTERRUPT) == LOW && millis() - start_time < BUTTON_SOFT_RESET_TIME) {
    if (!hold_buzzer_fired && millis() - start_time >= BUTTON_HOLD_TIME) {
      buzzer().beepPattern(2, 100, 100);
      hold_buzzer_fired = true;
    }
    delay(10);
  }
  if (millis() - start_time >= BUTTON_SOFT_RESET_TIME) {
    // Soft reset threshold reached, triple beep to signal the user can release.
    buzzer().beepPattern(3, 100, 100);
  } else if (!hold_buzzer_fired) {
    // Single beep on release for a short press (e.g. the refresh trigger), unless
    // the longer hold pattern already fired.
    buzzer().beep(100);
  }
  return millis() - start_time;
#endif // SHIP_MODE_SUPPORTED
}

static ButtonPressResult classify_press_duration(unsigned long duration) {
#ifdef SHIP_MODE_SUPPORTED
  if (duration >= BUTTON_SHIP_MODE_TIME) {
    Log_info("Button time=%lu detected ship-mode press", duration);
    return ShipMode;
  }
#endif
  if (duration >= BUTTON_SOFT_RESET_TIME) {
    Log_info("Button time=%lu detected extra-long press", duration);
    return SoftReset;
  } else if (duration > BUTTON_HOLD_TIME) {
    Log_info("Button time=%lu detected long press", duration);
    return LongPress;
  } else if (duration > BUTTON_MEDIUM_HOLD_TIME) {
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

static ButtonPressResult classify_button_presses() {
  auto time_start = millis();
  Log_info("Button time=%lu: start", time_start);
  pinMode(PIN_INTERRUPT, INPUT);
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

ButtonPressResult read_button_presses() {
  // Short-press feedback is emitted immediately on release inside
  // wait_for_button_release(), so no additional beep is needed here.
  return classify_button_presses();
}

const char *ButtonPressResultNames[] = {
    "LongPress",
    "DoubleClick",
    "ShortPress",
    "SoftReset",
    "ShipMode"};
