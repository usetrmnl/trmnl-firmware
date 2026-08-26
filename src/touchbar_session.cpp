#include <touchbar_session.h>

#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <display.h>
#include <globals.h>
#include <qa.h>
#include <trmnl_log.h>

#include "IQS323.h"
#include "iqs323_task.h"

void showMessageWithLogo(MSG message_type);
void resetDeviceCredentials(void);
void showLastImageAndSleep(void);

#define WIFI_RESET_CONFIRMATION_TIMEOUT_MS 15000
#define WIFI_RESET_POLL_INTERVAL_MS        100

bool in_wifi_reset_confirmation = false;
bool in_power_off_confirmation = false;
uint32_t s_power_off_cooldown_until = 0;

// Read gesture data directly without triggering other handlers
static void read_gesture_data_only() {
  // Read slider coordinates
  uint16_t buffer = iqs323.sliderCoordinate();
  if (buffer != slider_position) {
    slider_position = buffer;
  }

  // Read gesture event
  bool gesture_event = iqs323.getSliderEvent();
  if (gesture_event) {
    iqs323_gesture_events gesture_buffer = iqs323.getGestureType();
    if (gesture_buffer != IQS323_GESTURE_NONE) {
      slider_event = gesture_buffer;
    }
  }
}
// Check if user wants to confirm WiFi reset (middle button hold)
static bool check_wifi_reset_confirm() {
  if (slider_event == IQS323_GESTURE_HOLD && iqs323.channel_touchState(IQS323_CH1)) {
    Log_info("WiFi reset confirmed by user - holding middle button");
    return true;
  }
  return false;
}

// Check if user wants to cancel WiFi reset (any tap)
static bool check_wifi_reset_cancel() {
  if (slider_event == IQS323_GESTURE_TAP) {
    Log_info("WiFi reset cancelled by user - tap detected");
    return true;
  }
  return false;
}

static void confirm_wifi_reset() { resetDeviceCredentials(); }

static void confirm_power_off() {
  clearShipmentStatus();
  ESP.restart();
}

static bool handle_confirmation_flow(bool &in_flag, MSG message, void (*on_confirm)(void)) {
  in_flag = true;
  showMessageWithLogo(message);

  // Wait for the triggering hold to be fully released before accepting new input.
  // Without this, lifting fingers from the initial hold could register as a cancel tap.
  {
    const uint32_t RELEASE_TIMEOUT_MS = 2000;
    unsigned long release_start = millis();
    do {
      delay(200);
      iqs323.updateInfoFlags(STOP);
    } while ((iqs323.channel_touchState(IQS323_CH0) || iqs323.channel_touchState(IQS323_CH1) ||
              iqs323.channel_touchState(IQS323_CH2)) &&
             millis() - release_start < RELEASE_TIMEOUT_MS);
    slider_event = IQS323_GESTURE_NONE;
  }

  if (touchbar_tap_mode) {
    const uint32_t HOLD_MS = 600;
    const uint32_t POLL_MS = 20;
    unsigned long start_time = millis();

    while (millis() - start_time < WIFI_RESET_CONFIRMATION_TIMEOUT_MS) {
      delay(POLL_MS);
      if (iqs323.getRDYStatus()) {
        iqs323.updateInfoFlags(STOP);
      }

      if (iqs323.channel_touchState(IQS323_CH0) || iqs323.channel_touchState(IQS323_CH2)) {
        bool left_cancel = iqs323.channel_touchState(IQS323_CH0);
        Log_info("Confirmation cancelled - outer button in tap mode, status: left=%d right=%d",
                 iqs323.channel_touchState(IQS323_CH0), iqs323.channel_touchState(IQS323_CH2));
        display_draw_touchbar_indicator(left_cancel ? TOUCHBAR_LEFT : TOUCHBAR_RIGHT, false);
        in_flag = false;
        return false;
      }

      if (iqs323.channel_touchState(IQS323_CH1)) {
        unsigned long touch_start = millis();
        while (millis() - touch_start < HOLD_MS) {
          delay(POLL_MS);
          if (iqs323.getRDYStatus()) {
            iqs323.updateInfoFlags(STOP);
          }
          if (!iqs323.channel_touchState(IQS323_CH1)) {
            Log_info("Confirmation cancelled - tap on middle button in tap mode");
            display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, false);
            in_flag = false;
            return false;
          }
        }
        display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, true);
        Log_info("Confirmed - holding middle button in tap mode");
        in_flag = false;
        on_confirm();
        return true;
      }
    }

    Log_info("Confirmation timeout - cancelling");
    in_flag = false;
    return false;
  }

  unsigned long start_time = millis();

  while (millis() - start_time < WIFI_RESET_CONFIRMATION_TIMEOUT_MS) {
    delay(WIFI_RESET_POLL_INTERVAL_MS);
    read_gesture_data_only();

    if (check_wifi_reset_confirm()) {
      in_flag = false;
      on_confirm();
      return true;
    }

    if (check_wifi_reset_cancel()) {
      in_flag = false;
      return false;
    }

    if (slider_position == 65535) {
      slider_event = IQS323_GESTURE_NONE;
    }
  }

  Log_info("Confirmation timeout - cancelling");
  in_flag = false;
  return false;
}

void handle_wifi_reset_confirmation() {
  Log_info("Entering WiFi reset confirmation mode");
  bool confirmed = handle_confirmation_flow(in_wifi_reset_confirmation, WIFI_RESET_CONFIRM, confirm_wifi_reset);

  if (!confirmed) {
    Log_info("WiFi reset cancelled - redrawing last image and sleeping");
    showLastImageAndSleep();
  }
}

void handle_power_off_confirmation() {
  Log_info("Entering power-off confirmation mode");
  handle_confirmation_flow(in_power_off_confirmation, POWER_OFF_CONFIRM, confirm_power_off);
}

#endif // BOARD_TRMNL_X
