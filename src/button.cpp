#include "button.h"

#include <Arduino.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <bl.h>
#include <board_config.h>
#include <config.h>
#include <globals.h>
#include <misc/buzzer.h>

#include "ship_mode.h"
#include "trmnl_log.h"

static unsigned long wait_for_button_release(unsigned long start_time) {
  pinMode(pDevice->interrupt_pin, INPUT);
#ifdef SHIP_MODE_SUPPORTED
  // Boards that support user-triggered shipping mode allow the hold to run all
  // the way to BUTTON_SHIP_MODE_TIME. Feedback is staged during the hold:
  //   5 s  -> 2 beeps (long press / WiFi reset)
  //   15 s -> 3 beeps (soft reset / credentials reset)
  //   30 s -> 4 beeps (shipping mode)
  bool hold_buzzer_fired = false;
  bool soft_reset_buzzer_fired = false;
  while (digitalRead(pDevice->interrupt_pin) == LOW && millis() - start_time < BUTTON_SHIP_MODE_TIME) {
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
  while (digitalRead(pDevice->interrupt_pin) == LOW && millis() - start_time < BUTTON_SOFT_RESET_TIME) {
    delay(10);
  }
  if (millis() - start_time >= BUTTON_SOFT_RESET_TIME) {
    // Soft reset threshold reached, triple beep to signal the user can release.
    buzzer().beepPattern(3, 100, 100);
  } else if (millis() - start_time < BUTTON_HOLD_TIME) {
    // Single beep on release for a short press (e.g. the refresh trigger).
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
    if (digitalRead(pDevice->interrupt_pin) == LOW) {
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
  pinMode(pDevice->interrupt_pin, INPUT);
  if (digitalRead(pDevice->interrupt_pin) == HIGH) {
    if (time_start < 2000) {
      Log_info("Button: already released at start (GPIO wakeup), waiting for second press");
      return wait_for_second_press(time_start);
    } else {
      Log_info("Button: waiting for button press");
      while (digitalRead(pDevice->interrupt_pin) == HIGH) {
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

// Acts on the press read after a button wake: a long press forgets WiFi, a soft-reset press forgets the
// device (does not return). Returns true for a double click, which runs the special function.
bool button_wake_act(ButtonPressResult button) {
  bool double_click = false;
  Log_info("GPIO wakeup (%d) -> button was read (%s)", wakeup_reason, ButtonPressResultNames[button]);
  switch (button) {
  case LongPress:
    Log_info("WiFi reset");
    WifiCaptivePortal.resetSettings();
    break;
  case DoubleClick:
    double_click = true;
    break;
  case ShortPress:
  case NoAction:
    break;
  case SoftReset:
    resetDeviceCredentials();
    break;
#ifdef SHIP_MODE_SUPPORTED
  case ShipMode:
    enter_ship_mode(); // does not return
    break;
#endif
  default:
    break;
  }
  Log_info("button handling end");
  return double_click;
}

// Loads the special function the server assigned into special_function and opens the WiFi portal for
// SF_ADD_WIFI. The others run with the next /api/display request.
void button_wake_run_special_function(void) {
  if (preferences.isKey(PREFERENCES_SF_KEY)) {
    Log.info("%s [%d]: SF saved. Reading...\r\n", __FILE__, __LINE__);
    special_function = (SPECIAL_FUNCTION)preferences.getUInt(PREFERENCES_SF_KEY, 0);
    Log.info("%s [%d]: Read special function - %d\r\n", __FILE__, __LINE__, special_function);
    switch (special_function) {
    case SF_IDENTIFY: {
      Log.info("%s [%d]: Identify special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
    } break;
    case SF_SLEEP: {
      Log.info("%s [%d]: Sleep special function...\r\n", __FILE__, __LINE__);
        // still in progress
    } break;
    case SF_ADD_WIFI: {
      Log.info("%s [%d]: Add WiFi function...\r\n", __FILE__, __LINE__);
      WifiCaptivePortal.startPortal();
    } break;
    case SF_RESTART_PLAYLIST: {
      Log.info("%s [%d]: Restart Playlist special function...It will be handled with API ping...\r\n", __FILE__,
               __LINE__);
    } break;
    case SF_REWIND: {
      Log.info("%s [%d]: Rewind special function...\r\n", __FILE__, __LINE__);
    } break;
    case SF_SEND_TO_ME: {
      Log.info("%s [%d]: Send to me special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
    } break;
    case SF_GUEST_MODE: {
      Log.info("%s [%d]: Guest Mode special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
    } break;
    default:
      break;
    }
  } else {
    Log_error("SF not saved");
  }
}
