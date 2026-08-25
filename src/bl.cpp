#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <bl.h>
#include <wifi_network.h>
#include <power.h>
#include <battery.h>
#include <device_id.h>
#include <trmnl_log.h>
#include <types.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <pins.h>
#include <config.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <display.h>
#include <stdlib.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <cstdint>
#include "png.h"
#include <bmp.h>
#include <Update.h>
#include <math.h>
#include <filesystem.h>
#include <stored_logs.h>
#include <button.h>
#include "api-client/submit_log.h"
#include <api-client/setup.h>
#include <special_function.h>
#include <refresh_interval.h>
#include <services/firmware_update.h>
#include <api_response_parsing.h>
#include "logging_parcers.h"
#include <SPIFFS.h>
#include "http_client.h"
#include <StreamString.h>
#include <api-client/display.h>
#include <api-client/request_headers.h>
#include "driver/gpio.h"
#include "esp_flash.h"
#include <nvs.h>
#include <serialize_log.h>
#include <preferences_persistence.h>
#include "logo_small.h"
#include "logo_medium.h"
#include "loading.h"
#include <wifi-helpers.h>
#include <sys/time.h>
#include <misc/buzzer.h>
#include <misc/clock.h>
#include <misc/sensor.h>
#include <services/device_setup.h>
#include "messages.h"
#include "displayed_image.h"
#include <globals.h>
#include <display_session.h>
const char *szHTTPErrors[] = {
    "HTTPS_NO_ERR",
    "HTTPS_RESET",
    "HTTPS_NO_REGISTER",
    "HTTPS_SUCCESS",
    "HTTPS_CLIENT_FAILED",
    "HTTPS_REQUEST_FAILED",
    "HTTPS_UNABLE_TO_CONNECT",
    "HTTPS_CONNECTION_FAILED",
    "HTTPS_RESPONSE_CODE_INVALID",
    "HTTPS_JSON_PARSING_ERR",
    "HTTPS_WRONG_IMAGE_SIZE",
    "HTTPS_WRONG_IMAGE_FORMAT",
    "HTTPS_IMAGE_FILE_TOO_BIG",
    "HTTPS_PLUGIN_NOT_ATTACHED",
    "HTTPS_BAD_CLIENT",
    "HTTPS_OUT_OF_MEMORY"
};

// downloadAndShow → display_session
// handleApiDisplayResponse → display_session
static void resetDeviceCredentials(void);            // reset device credentials API key, Friendly ID, Wi-Fi SSID and password
void goToSleep(void);                         // sleep preparing
static void goToSleepButtonOnly(void);               // sleep until button press, no timer
// Non-static: used by display_session (downloadAndShow)
void submitStoredLogs(void);
// Non-static: used by display_session (handleApiDisplayResponse)
void writeSpecialFunction(SPECIAL_FUNCTION function);
void showMessageWithLogo(MSG message_type);
static void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message);
static void showMessageWithLogo(MSG message_type, const ApiSetupResponse &apiResponse);
static void wifiErrorDeepSleep();
static uint8_t *storedLogoOrDefault(int iType);
static DeviceStatusStamp getDeviceStatusStamp();
void config_gpio_for_lp();
int png_to_epd(const uint8_t *pPNG, int iDataSize, bool bPrevious);

static unsigned long startup_time = 0;

#ifndef BOARD_TRMNL_X
// Create stub functions for the touchbar workaround
void iqs323_task_i2c_lock(void) {}
void iqs323_task_i2c_unlock(void) {}
#endif // !BOARD_TRMNL_X

void wait_for_serial() {
#ifdef WAIT_FOR_SERIAL
  int idx = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
      if (Serial)
        break;
      delay(100);
      idx++;
    }
  Log_info("## Waited for serial.. %d ms", idx * 100);
#endif
}
#ifdef BOARD_TRMNL_X
#include <qa.h> // For device turn off feature
// ############################ WAKEUP STUB #############################
#include "rtc_wake_stub_trmnl_x.h"
// ############################ WAKEUP STUB #############################

// ############################ IQS323 TASK #############################
#include "iqs323_task.h"
// ############################ IQS323 TASK #############################

// ############################ SLIDER ##################################
#include "IQS323.h"

void process_iqs323_data(void);

#define IQS323_I2C_ADDRESS 0x44
// Touchbar indicator to redraw after a full-refresh (e.g. logo screen)
static touchbar_side_t pending_indicator_side = TOUCHBAR_LEFT;
static bool pending_indicator_filled = false;
static bool has_pending_indicator = false;

// WiFi reset confirmation constants
#define WIFI_RESET_CONFIRMATION_TIMEOUT_MS 15000
#define WIFI_RESET_POLL_INTERVAL_MS 100

// Static flag to prevent re-entry during WiFi reset confirmation
static bool in_wifi_reset_confirmation = false;
// Static flag to prevent re-entry during power-off confirmation
static bool in_power_off_confirmation = false;
// Cooldown timestamp after cancel to prevent immediate re-trigger
static uint32_t s_power_off_cooldown_until = 0;
// Tracks first detection of both corners held so wakeup_time can be reset once
static bool s_corners_detected = false;

// Read gesture data directly without triggering other handlers
void read_gesture_data_only()
{
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
// Returns true if channel i has been held for HOLD_THRESHOLD_MS since wakeup stub.
// Releases the I2C lock during the wait so the iqs323 task can update the memory map.
static bool tap_mode_is_hold(uint8_t channel_index, time_t hold_threshold_ms = 600)
{
  const uint32_t POLL_INTERVAL_MS = 20;

  // update the memory map to get the latest touch states, but save wakeup stub values for TAPs
  uint8_t saved_status[2] = { iqs323.IQSMemoryMap.SYSTEM_STATUS[0], iqs323.IQSMemoryMap.SYSTEM_STATUS[1] };
  iqs323_task_i2c_lock();
  iqs323.updateInfoFlags(STOP);
  iqs323_task_i2c_unlock();

  while (true) {
    if (millis() - startup_time >= hold_threshold_ms) break;

    iqs323_task_i2c_unlock();
    delay(POLL_INTERVAL_MS);
    iqs323_task_i2c_lock();

    // Only read from chip when it has naturally opened a window (RDY LOW).
    // Forcing I2C on every tick causes the chip to stop responding after ~30+ iterations
    if (iqs323.getRDYStatus()) {
      iqs323.updateInfoFlags(STOP);
    }
    if (!iqs323.channel_touchState((iqs323_channel_e)channel_index)) {
      iqs323.IQSMemoryMap.SYSTEM_STATUS[0] = saved_status[0];
      iqs323.IQSMemoryMap.SYSTEM_STATUS[1] = saved_status[1];
      return false;
    }
  }

  iqs323.updateInfoFlags(STOP);

  if (!iqs323.channel_touchState((iqs323_channel_e)channel_index)) {
    iqs323.IQSMemoryMap.SYSTEM_STATUS[0] = saved_status[0];
    iqs323.IQSMemoryMap.SYSTEM_STATUS[1] = saved_status[1];
  }
  return iqs323.channel_touchState((iqs323_channel_e)channel_index);
}

// Check if user wants to confirm WiFi reset (middle button hold)
bool check_wifi_reset_confirm()
{
  if (slider_event == IQS323_GESTURE_HOLD && iqs323.channel_touchState(IQS323_CH1)) {
    Log_info("WiFi reset confirmed by user - holding middle button");
    return true;
  }
  return false;
}

// Check if user wants to cancel WiFi reset (any tap)
bool check_wifi_reset_cancel()
{
  if (slider_event == IQS323_GESTURE_TAP) {
    Log_info("WiFi reset cancelled by user - tap detected");
    return true;
  }
  return false;
}

static void confirm_wifi_reset()
{
  resetDeviceCredentials();
}

static void confirm_power_off()
{
  clearShipmentStatus();
  ESP.restart();
}

static bool handle_confirmation_flow(bool &in_flag, MSG message, void (*on_confirm)(void))
{
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
    } while ((iqs323.channel_touchState(IQS323_CH0) ||
              iqs323.channel_touchState(IQS323_CH1) ||
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
        Log_info("Confirmation cancelled - outer button in tap mode, status: left=%d right=%d", iqs323.channel_touchState(IQS323_CH0), iqs323.channel_touchState(IQS323_CH2));
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

static void showLastImageAndSleep()
{
  int file_size = 0;
  String curPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
  if (!curPath.isEmpty()) {
    uint8_t *buf = display_read_file(curPath.c_str(), &file_size);
    if (buf && file_size > 0) {
      display_show_image(buf, file_size, true);
      free(buf);
      DisplayedImage::remember(curPath.c_str());
    }
  }
  goToSleep();
}

void handle_wifi_reset_confirmation()
{
  Log_info("Entering WiFi reset confirmation mode");
  bool confirmed = handle_confirmation_flow(in_wifi_reset_confirmation, WIFI_RESET_CONFIRM, confirm_wifi_reset);
  
  if (!confirmed) {
    Log_info("WiFi reset cancelled - redrawing last image and sleeping");
    showLastImageAndSleep();
  }
}

void handle_power_off_confirmation()
{
  Log_info("Entering power-off confirmation mode");
  handle_confirmation_flow(in_power_off_confirmation, POWER_OFF_CONFIRM, confirm_power_off);
}

// Check if both left and right corners are being held
bool check_corners_gesture()
{
  // check updated values
  bool left  = iqs323.channel_touchState(IQS323_CH0);
  bool middle = iqs323.channel_touchState(IQS323_CH1);
  bool right = iqs323.channel_touchState(IQS323_CH2);
  Log_info("Hold edges: left=%d middle=%d right=%d tap_mode=%d", left, middle, right, touchbar_tap_mode);

  if (touchbar_tap_mode) {
    bool hold_left  = tap_mode_is_hold(0);
    bool hold_right = tap_mode_is_hold(2);
    Log_info("Hold edges tap mode: hold_left=%d hold_right=%d", hold_left, hold_right);
    return hold_left && hold_right;
  }
  Log_info("Hold edges slider mode: event=%d (HOLD=%d) left=%d right=%d", slider_event, IQS323_GESTURE_HOLD, left, right);
  return slider_event == IQS323_GESTURE_HOLD && left && right;
}

static void show_cached_image_by_offset(int offset) {
  String order = preferences.getString(PREFERENCES_PLAYLIST_ORDER_KEY, "");

  if (order.isEmpty()) {
    String path = (offset > 0)
      ? preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "")
      : preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    if (path.isEmpty()) { Log_info("No cached image for gesture"); return; }
    int file_size = 0;
    buffer = display_read_file(path.c_str(), &file_size);
    if (buffer && file_size > 0) {
      display_show_image(buffer, file_size, true);
      DisplayedImage::remember(path.c_str());
      goToSleep();
    }
    return;
  }

  char images[MAX_CACHED_IMAGES][36];
  int count = 0;
  int start = 0;
  while (start <= (int)order.length() && count < MAX_CACHED_IMAGES) {
    int sep = order.indexOf('|', start);
    String entry = (sep < 0) ? order.substring(start) : order.substring(start, sep);
    if (!entry.isEmpty() && filesystem_file_exists(entry.c_str())) {
      strncpy(images[count], entry.c_str(), 35);
      images[count][35] = '\0';
      count++;
    }
    if (sep < 0) break;
    start = sep + 1;
  }

  if (count == 0) { Log_info("No cached images available"); return; }

  String browsePath = preferences.getString(PREFERENCES_BROWSE_PATH_KEY, "");
  if (browsePath.isEmpty()) {
    // Seed from last_path so first RIGHT shows curr_path (forward) and first LEFT shows older (backward).
    // Falls back to curr_path if last_path is absent (e.g. only one image cached).
    String lp = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    browsePath = lp.isEmpty() ? preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "") : lp;
  }

  int cur_idx = count - 1;
  for (int i = 0; i < count; i++) {
    if (browsePath == String(images[i])) { cur_idx = i; break; }
  }

  int new_idx = (cur_idx + offset + count) % count;
  Log_info("Playlist browse: %d/%d -> %d (%s)", cur_idx, count, new_idx, images[new_idx]);

  int file_size = 0;
  buffer = display_read_file(images[new_idx], &file_size);
  if (!buffer || file_size == 0) { Log_info("Failed to read %s", images[new_idx]); return; }

  preferences.putString(PREFERENCES_BROWSE_PATH_KEY, String(images[new_idx]));
  display_show_image(buffer, file_size, true);
  DisplayedImage::remember(images[new_idx]);
  goToSleep();
}

void check_channel_states(void)
{
  /* Loop through all the active channels */
  for (uint8_t i = 0; i < 3; i++) {
    if (iqs323.channel_touchState((iqs323_channel_e)(i))) {
      if (touchbar_tap_mode) {
        // Tap mode
        bool hold = tap_mode_is_hold(i, 2000);  // 2 second hold for tap mode actions
        switch (i) {
        case 0:
          if (!hold) {
            display_draw_touchbar_indicator(TOUCHBAR_LEFT, false);
            Log_info("Back button tapped");
            pending_indicator_side = TOUCHBAR_LEFT;
            pending_indicator_filled = false;
            has_pending_indicator = true;
            show_cached_image_by_offset(-1);
          } else {
            display_draw_touchbar_indicator(TOUCHBAR_LEFT, true);
            Log_info("Back button hold");
            pending_indicator_side = TOUCHBAR_LEFT;
            pending_indicator_filled = true;
            has_pending_indicator = true;
            show_cached_image_by_offset(-1);
          }
          break;
        case 1:
          if (hold) {
            display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, true);
            Log_info("Middle button hold");
            pending_indicator_side = TOUCHBAR_MIDDLE;
            pending_indicator_filled = true;
            has_pending_indicator = true;
            // Log_info("Middle button held - OTG toggle");
            // if (otg_state) {
            //   otg_turn_off();
            //   showMessageWithLogo(OTG_TURNED_OFF); otg_state = false;
            // }
            // else {
            //   otg_turn_on();
            //   showMessageWithLogo(OTG_TURNED_ON);
            //   otg_state = true;
            // }
            // delay(1000);
            // showLastImageAndSleep();
          } else {
            display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, false);
            Log_info("Middle button tapped");
            pending_indicator_side = TOUCHBAR_MIDDLE;
            pending_indicator_filled = false;
            has_pending_indicator = true;
          }
          break;
        case 2:
          if (!hold) {
            display_draw_touchbar_indicator(TOUCHBAR_RIGHT, false);
            Log_info("Next button tapped");
            pending_indicator_side = TOUCHBAR_RIGHT;
            pending_indicator_filled = false;
            has_pending_indicator = true;
            show_cached_image_by_offset(+1);
          } else {
            display_draw_touchbar_indicator(TOUCHBAR_RIGHT, true);
            Log_info("Next button hold");
            pending_indicator_side = TOUCHBAR_RIGHT;
            pending_indicator_filled = true;
            has_pending_indicator = true;
            show_cached_image_by_offset(+1);
          }
          break;
        }
      } else {
        // Slide mode
        if ((slider_event == IQS323_GESTURE_TAP || slider_event == IQS323_GESTURE_HOLD)) {
          printf("CH: %d: Touch\n", i);
          switch (i) {
          case 0:
            display_draw_touchbar_indicator(TOUCHBAR_LEFT, slider_event == IQS323_GESTURE_HOLD);
            Log_info("Back button pressed");
            break;
          case 1:
            display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, slider_event == IQS323_GESTURE_HOLD);
            Log_info("Middle button pressed");
            // if (otg_state) {
            //   otg_turn_off();
            //   showMessageWithLogo(OTG_TURNED_OFF); otg_state = false;
            // }
            // else {
            //   otg_turn_on();
            //   showMessageWithLogo(OTG_TURNED_ON);
            //   otg_state = true;
            // }
            // delay(1000);
            // showLastImageAndSleep();
            break;
          case 2:
            display_draw_touchbar_indicator(TOUCHBAR_RIGHT, slider_event == IQS323_GESTURE_HOLD);
            Log_info("Next button pressed");
            break;
          }
        }
      }
    }
  }
}

void read_slider_coordinates(void)
{
  /* read slider coordinates from memory */
  uint16_t buffer = iqs323.sliderCoordinate();

  if(buffer != slider_position)
  {
    slider_position = buffer;
  }
}

/* Function to process Slider gesture events */
void read_gesture_event(void)
{
  /* Read slider bit to check if a slider event occurred */
  bool gesture_event = iqs323.getSliderEvent();
  printf("Gesture event: %d\n", gesture_event);
  if (gesture_event)
  {
    /* returns slider event that occurred (tap, swipe or flick) by reading event bits from MM */
    iqs323_gesture_events gesture_buffer = iqs323.getGestureType();
    printf("Gesture type: %d\n", gesture_buffer);
    if(gesture_buffer != IQS323_GESTURE_NONE)
    {
      slider_event = gesture_buffer;
      switch (slider_event)
      {
        case IQS323_GESTURE_UNKNOWN:
          Log_info("SLIDER: UNKNOWN (something went wrong?)");
          break;
        case IQS323_GESTURE_TAP:
          Log_info("SLIDER: Tap");
          break;
        case IQS323_GESTURE_SWIPE_NEGATIVE:
          Log_info("SLIDER: Swipe <-");
          if (!touchbar_tap_mode) {
            show_cached_image_by_offset(-1);
          }
          break;
        case IQS323_GESTURE_SWIPE_POSITIVE:
          Log_info("SLIDER: Swipe ->");
          if (!touchbar_tap_mode) {
            show_cached_image_by_offset(+1);
          }
          break;
        case IQS323_GESTURE_FLICK_NEGATIVE:
          Log_info("SLIDER: Flick <-");
          break;
        case IQS323_GESTURE_FLICK_POSITIVE:
          Log_info("SLIDER: Flick ->");
          break;
        case IQS323_GESTURE_HOLD:
          Log_info("SLIDER: Hold");
          break;
        case IQS323_GESTURE_NONE:
          Log_info("SLIDER: None");
          break;
      }

      /* Clear event if a finger is removed from slider after the event was processed */
      if (slider_position == 65535)
      {
        slider_event = IQS323_GESTURE_NONE;
      }
    }
  }
}
void process_iqs323_data(void)
{
  /* Read slider coordinates from memory */
  uint16_t buffer = iqs323.sliderCoordinate();

  if(buffer != slider_position)
  {
    slider_position = buffer;
  }

  Log_info("Slider position: %d", slider_position);

  iqs323_task_i2c_lock();

  /* Read gesture event if available */
  read_gesture_event();

  iqs323_task_i2c_unlock();

  if (!in_wifi_reset_confirmation && check_corners_gesture()) {
    handle_wifi_reset_confirmation();
    return;
  }

  iqs323_task_i2c_lock();

  /* Check channel touch states */
  check_channel_states();

  iqs323_task_i2c_unlock();
}
// ############################ SLIDER ################################

// ############################ ACCELERATOR ###########################
#include "accelerometer.h"
// ############################ ACCELERATOR ###########################

// ############################ esp32c5 modem #########################
#include "modem.h"
// ############################ esp32c5 modem #########################

#endif

/**
 * @brief Function to init business logic module
 * @param none
 * @return none
 */
void bl_init(void)
{
#ifdef BOARD_SEEED_STICKY
  pinMode(45, OUTPUT); // power hold (DATA)
  pinMode(46, OUTPUT); // power lock (CLK)
  digitalWrite(45, 1);
  digitalWrite(46, 0);
  digitalWrite(46, 1); // Hold the battery power enabled for after the user releases the power button
#endif
#ifdef BOARD_TRMNL_X
  uint32_t init_time = esp_cpu_get_cycle_count() / esp_rom_get_cpu_ticks_per_us();
#else
  uint32_t init_time = micros();
#endif
  startup_time = init_time/1000L; // convert to milliseconds
#ifdef DEV_FIRMWARE
  Serial.begin(115200);
  wait_for_serial();
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
#endif
  Log_info("BL init success");

  WifiCaptivePortal.setHostname(getWifiClientHostname());

#ifdef BOARD_TRMNL_X
  bool bModemNeeded = false;
  Log.info("%s [%d]: Checking if we need to use the ESP32-C5 modem...\r\n", __FILE__, __LINE__);
  if (WifiCaptivePortal.isSaved()) {
    // WiFi saved, connection
    WifiCredentials lastCreds = WifiCaptivePortal.getLastCredentials();
    bModemNeeded = lastCreds.is5GHz;
  } else {
    bModemNeeded = true; // captive portal needs modem for 5 GHz
  }
  Log.info("%s [%d]: modem needed = %d\n\r", __FILE__, __LINE__, bModemNeeded);
#endif // X
  pins_init();
  buzzer().init();
  sensor().init();
#ifdef BOARD_TRMNL_X
  // Debug: Print all wakeup_stub_iqs_status structure fields
  Log_info("wakeup_stub_iqs_status.status: 0x%02X 0x%02X", wakeup_stub_iqs_status.status[0], wakeup_stub_iqs_status.status[1]);
  Log_info("wakeup_stub_iqs_status.gestures: 0x%02X 0x%02X", wakeup_stub_iqs_status.gestures[0], wakeup_stub_iqs_status.gestures[1]);
  Log_info("wakeup_stub_iqs_status.slider_cords: 0x%02X 0x%02X", wakeup_stub_iqs_status.slider_cords[0], wakeup_stub_iqs_status.slider_cords[1]);
  Log_info("wakeup_stub_iqs_status.ch0_cnts: 0x%02X 0x%02X 0x%02X 0x%02X", wakeup_stub_iqs_status.ch0_cnts[0], wakeup_stub_iqs_status.ch0_cnts[1], wakeup_stub_iqs_status.ch0_cnts[2], wakeup_stub_iqs_status.ch0_cnts[3]);
  Log_info("wakeup_stub_iqs_status.ch1_cnts: 0x%02X 0x%02X 0x%02X 0x%02X", wakeup_stub_iqs_status.ch1_cnts[0], wakeup_stub_iqs_status.ch1_cnts[1], wakeup_stub_iqs_status.ch1_cnts[2], wakeup_stub_iqs_status.ch1_cnts[3]);
  Log_info("wakeup_stub_iqs_status.ch2_cnts: 0x%02X 0x%02X 0x%02X 0x%02X", wakeup_stub_iqs_status.ch2_cnts[0], wakeup_stub_iqs_status.ch2_cnts[1], wakeup_stub_iqs_status.ch2_cnts[2], wakeup_stub_iqs_status.ch2_cnts[3]);
#endif

#if defined(BOARD_SEEED_XIAO_ESP32C3)
  delay(2000);

  if (digitalRead(PIN_INTERRUPT) == LOW) {
    Log_info("Boot button pressed during startup, resetting WiFi credentials...");
    WifiCaptivePortal.resetSettings();
    Log_info("WiFi credentials reset completed");
  }
#endif

  wakeup_reason = esp_sleep_get_wakeup_cause();
  bool gpio_wakeup = (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO ||
                      wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 ||
                      wakeup_reason == ESP_SLEEP_WAKEUP_EXT1);
  Log.info("%s [%d]: Wake reason: %d\r\n", __FILE__, __LINE__, (int)wakeup_reason);

  Log_info("preferences start");
  bool res = preferences.begin("data", false);
  if (res)
  {
    Log_info("preferences init success (%d free entries)", preferences.freeEntries());
  }
  else
  {
    Log_fatal("preferences init failed");
    ESP.restart();
  }
  Log_info("preferences end");
  #ifndef BOARD_TRMNL_X
  bool double_click = false;
  if (gpio_wakeup)
  {
    Log_info("GPIO wakeup detected (%d)", wakeup_reason);
    auto button = read_button_presses();
    wait_for_serial();
    Log_info("GPIO wakeup (%d) -> button was read (%s)", wakeup_reason, ButtonPressResultNames[button]);
    switch (button)
    {
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
    }
    Log_info("button handling end");
  }
  else
  {
    wait_for_serial();
    Log_info("Non-GPIO wakeup (%d) -> didn't read buttons", wakeup_reason);
  }
#else // BOARD_TRMNL_X
  // Notify IQS323 task about wakeup type BEFORE starting the task

  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  iqs323_task_i2c_lock();
  display_init();
  otg_turn_off(); // Since OTG function was commented out, need to ensure that OTG is turned off
  iqs323_task_i2c_unlock();
  filesystem_init();

  Wire.setClock(100000);

  if (gpio_wakeup) {
    Log_info("GPIO wakeup detected (%d) - using wake stub data", wakeup_reason);
    iqs323_task_notify_gpio_wakeup(true);
  } else {
    Log_info("Non-GPIO wakeup (%d)", wakeup_reason);
  }
#endif // BOARD_TRMNL_X

#ifdef BOARD_TRMNL_X
  touchbar_tap_mode = preferences.getBool(PREFERENCES_TOUCHBAR_MODE_KEY, true);
  Log_info("Touchbar mode from preferences: %s", touchbar_tap_mode ? "Tap" : "Slide");

  // Start IQS323 task manager
  if (!iqs323_task_init(NULL)) {
    Log_error("IQS323 Task: Failed to start - rebooting");
    delay(1000);
    ESP.restart();
  }

  // Wait for IQS323 initialization to complete
  if (!iqs323_task_wait_ready(5000)) {
    Log_error("IQS323 Task: Initialization timeout - rebooting");
    delay(1000);
    ESP.restart();
  }

  if (gpio_wakeup) {
    process_iqs323_data();
  }

  // For future
  // iqs323_task_set_data_callback(process_iqs323_data);

  Log_info("init time: %ld us", init_time);
#else // BOARD_TRMNL_X

  if (double_click)
  { // special function reading
    if (preferences.isKey(PREFERENCES_SF_KEY))
    {
      Log.info("%s [%d]: SF saved. Reading...\r\n", __FILE__, __LINE__);
      special_function = (SPECIAL_FUNCTION)preferences.getUInt(PREFERENCES_SF_KEY, 0);
      Log.info("%s [%d]: Read special function - %d\r\n", __FILE__, __LINE__, special_function);
      switch (special_function)
      {
      case SF_IDENTIFY:
      {
        Log.info("%s [%d]: Identify special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_SLEEP:
      {
        Log.info("%s [%d]: Sleep special function...\r\n", __FILE__, __LINE__);
        // still in progress
      }
      break;
      case SF_ADD_WIFI:
      {
        Log.info("%s [%d]: Add WiFi function...\r\n", __FILE__, __LINE__);
        WifiCaptivePortal.startPortal();
      }
      break;
      case SF_RESTART_PLAYLIST:
      {
        Log.info("%s [%d]: Restart Playlist special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_REWIND:
      {
        Log.info("%s [%d]: Rewind special function...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_SEND_TO_ME:
      {
        Log.info("%s [%d]: Send to me special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      case SF_GUEST_MODE:
      {
        Log.info("%s [%d]: Guest Mode special function...It will be handled with API ping...\r\n", __FILE__, __LINE__);
      }
      break;
      default:
        break;
      }
    }
    else
    {
      Log_error("SF not saved");
    }
  }
  // Read the battery voltage BEFORE the display or WiFi is turned on
  vBatt = battery().readVoltage();

  // EPD init
  // EPD clear
  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  display_init();

  // Mount SPIFFS
  filesystem_init();
#endif // !BOARD_TRMNL_X

// #ifdef BOARD_TRMNL_X

//   int8_t rslt;
//   // I2C already initialized by IQS323 - do not call Wire.begin() again as it corrupts the bus on ESP32S3
//   Serial.printf("Using I2C bus already initialized (SDA: %d, SCL: %d)\n\n", SENSOR_SDA_PIN, SENSOR_SCL_PIN);

//   struct bma5_dev bma530_dev;

//   rslt = bma530_init_device(&bma530_dev);
//   if (rslt != BMA5_OK) {
//     Serial.println("Failed to initialize BMA530!");
//   }

//   rslt = bma530_configure_low_power_mode(&bma530_dev);
//   if (rslt != BMA5_OK) {
//     Serial.println("Failed to configure BMA530 low power mode!");
//   }

//   rslt = bma530_configure_orientation(&bma530_dev);
//   if (rslt != BMA5_OK) {
//     Serial.println("Failed to configure BMA530 orientation!");
//   }

//   // Configure INT1 pin
//   rslt = bma530_configure_int1(&bma530_dev);
//   if (rslt != BMA5_OK) {
//       Serial.println("Failed to configure BMA530 INT1!");
//   }

//   config_bma530_interrupt();

//   pinMode(TCA9535_INT, INPUT);

// #endif

#ifdef BOARD_TRMNL_X
  // Read the gauge before the panel draws load current, and before the logo
  // is drawn so its battery icon has a snapshot to read.
  battery_count = detect_battery_count();
  battery_charging = (power().chargingStatus() == ChargingStatus::CHARGING);
  Log_info("BATTERY COUNT: %d", battery_count);
  Log_info("BATTERY CHARGING: %s", battery_charging ? "YES" : "NO");

  battery().gaugeInit();
  vBatt = battery().readVoltage(); // Read the battery voltage BEFORE WiFi is turned on
#endif // BOARD_TRMNL_X

  if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
  {
    Log.info("%s [%d]: Display TRMNL logo start\r\n", __FILE__, __LINE__);

#ifdef BOARD_TRMNL_X

    if (!otg_message && WifiCaptivePortal.isSaved()) {
      display_show_image(storedLogoOrDefault(1), DEFAULT_IMAGE_SIZE, false, true);
      if (has_pending_indicator) {
        display_draw_touchbar_indicator(pending_indicator_side, pending_indicator_filled);
        has_pending_indicator = false;
      }
    }
    else if (!WifiCaptivePortal.isSaved()) {
      showMessageWithLogo(NONE);
    }
#else
    display_show_image(storedLogoOrDefault(1), DEFAULT_IMAGE_SIZE, false, true);
#endif // BOARD_TRMNL_X
    // Force the display to show the current playlist image after the loading screen
    // (even if it hasn't changed)
    DisplayedImage::clear();

    need_to_refresh_display = 1;
    preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
    Log.info("%s [%d]: Display TRMNL logo end\r\n", __FILE__, __LINE__);
    preferences.putString(PREFERENCES_FILENAME_KEY, "");
  }
  Log_info("Firmware version %s", Messages::firmware_version().c_str());
  Log_info("Arduino version %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Log_info("ESP-IDF version %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
  list_files();
  log_nvs_usage();

  // DEBUG - test message display
  // showMessageWithLogo(MSG_FORMAT_ERROR);
  // display_show_msg(storedLogoOrDefault(1), WIFI_CONNECT, "ABCDEF", true, Messages::firmware_version().c_str(), "Hello World!");
  // wifiErrorDeepSleep();
#ifdef BOARD_TRMNL_X
  if (bModemNeeded) {
    modem_reset_target(); // Must be done BEFORE the instantiation of the class since it expects the modem to be ready
    static Modem modemInstance(115200);
    if (modemInstance.isInitialized()) {
      g_modem = &modemInstance;
    } else {
      Log_info("Modem init failed — falling back to 2.4 GHz mode");
      g_modem = nullptr;
    }
  } else {
    g_modem = nullptr;
  }
    
  // Only scan when no credentials are saved (i.e. captive portal will be shown). 
  if (g_modem && !WifiCaptivePortal.isSaved())
  {
    // The modem is a separate radio and can see the device's own captive-portal
    // SoftAP over the air; exclude it so it never shows up as a connectable network.
    String ownApSsid = WifiCaptivePortal.getAPSSID();

    Log_info("No saved credentials — scanning networks via modem...");
    auto modemNets = g_modem->scanNetworks();
    Log_info("Modem found %d network(s)", modemNets.size());
    std::vector<ExternalNetwork> nets;
    for (auto& n : modemNets) {
      if (n.ssid == ownApSsid) continue;
      nets.push_back({n.ssid, n.rssi, n.open, n.is5GHz});
    }
    WifiCaptivePortal.setNetworks(nets);

    // Register callback so captive portal can connect 5 GHz networks via modem
    WifiCaptivePortal.setModemConnectCallback([](const String& ssid, const String& pass) {
      return g_modem->connectToNetwork(ssid, pass, getWifiClientHostname());
    });

    // Register callback so the captive portal's Refresh button can trigger a fresh modem scan
    WifiCaptivePortal.setModemScanCallback([ownApSsid]() {
      auto modemNets = g_modem->scanNetworks();
      Log_info("Modem re-scan found %d network(s)", modemNets.size());
      std::vector<ExternalNetwork> nets;
      for (auto& n : modemNets) {
        if (n.ssid == ownApSsid) continue;
        nets.push_back({n.ssid, n.rssi, n.open, n.is5GHz});
      }
      return nets;
    });

    String modemMac = g_modem->getMacAddress();
    if (!modemMac.isEmpty()) {
      WifiCaptivePortal.setModemMac(modemMac);
    }
  }
#endif // BOARD_TRMNL_X

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  MSG current_msg = NONE;

// uncdcomment this to hardcode WiFi credentials (useful for testing wifi errors, etc.)
// #define HARDCODED_WIFI
#ifdef HARDCODED_WIFI
  WifiCredentials hardcodedCreds = {.ssid = "ssid-goes-here", .pswd = "password-goes-here"};
  Log_info("Hardcoded WiFi: connecting to SSID '%s'", hardcodedCreds.ssid.c_str());
  auto connectResult = WifiCaptivePortal.connect(hardcodedCreds);
  Log_info("Hardcoded WiFi: connect result '%s'", wifiStatusStr(connectResult));
// goToSleep();
#else

  if (WifiCaptivePortal.isSaved())
  {
    // WiFi saved, connection
    Log.info("%s [%d]: WiFi saved\r\n", __FILE__, __LINE__);
    int connection_res = connectWithSavedCredentials() ? 1 : 0;

    Log.info("%s [%d]: Connection result: %d, WiFI Status: %d\r\n", __FILE__, __LINE__, connection_res, WiFi.status());

    // Check if connected
    if (connection_res)
    {
      String ip = String(WiFi.localIP());
      Log.info("%s [%d]:wifi_connection [DEBUG]: Connected: %s\r\n", __FILE__, __LINE__, ip.c_str());
      preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
    }
    else
    {
      if (current_msg != WIFI_FAILED)
      {
        showMessageWithLogo(WIFI_FAILED);
        current_msg = WIFI_FAILED;
      }

      Log_fatal_submit("Connection failed! WL Status: %d", WiFi.status());

      wifiErrorDeepSleep();
    }
  }
  else
  {
    // WiFi credentials are not saved - start captive portal
    Log.info("%s [%d]: WiFi NOT saved\r\n", __FILE__, __LINE__);

    Log_info("FW version %s", Messages::firmware_version().c_str());

    showMessageWithLogo(WIFI_CONNECT, "", false, Messages::firmware_version().c_str(), WifiCaptivePortal.getAPSSID());
#ifdef BOARD_TRMNL_X
    // set TAP mode as default
    iqs323_task_i2c_lock();
    iqs323.setGestureConfig(true, STOP);
    iqs323_task_i2c_unlock();
    touchbar_tap_mode = true;

    static uint32_t s_corners_start_ms = 0;
    WifiCaptivePortal.setPortalTickCallback([]() {
      if (in_power_off_confirmation) return;
      if (millis() < s_power_off_cooldown_until) return;
      iqs323_task_i2c_lock();
      if (iqs323.getRDYStatus()) {
        iqs323.updateInfoFlags(STOP);
      }
      bool left  = iqs323.channel_touchState(IQS323_CH0);
      bool right = iqs323.channel_touchState(IQS323_CH2);
      if (left && right) {
        if (!s_corners_detected) {
          s_corners_start_ms = millis();
          s_corners_detected = true;
        } else if (millis() - s_corners_start_ms >= 600) {
          s_corners_detected = false;
          handle_power_off_confirmation();
          // Only reached on cancel — confirmed path calls ESP.restart()
          s_power_off_cooldown_until = millis() + 2000;
          iqs323_task_i2c_unlock();
          showMessageWithLogo(WIFI_CONNECT, "", false, Messages::firmware_version().c_str(), WifiCaptivePortal.getAPSSID());
          return;
        }
      } else {
        s_corners_detected = false;
      }
      iqs323_task_i2c_unlock();
    });
#endif
    WifiCaptivePortal.setResetSettingsCallback(resetDeviceCredentials);
    res = WifiCaptivePortal.startPortal();
    if (!res)
    {
      WiFi.disconnect(true);

      showMessageWithLogo(WIFI_FAILED);

      Log_error("Failed to connect or hit timeout");

      // Go to deep sleep
      wifiErrorDeepSleep();
    }
    Log.info("%s [%d]: WiFi connected\r\n", __FILE__, __LINE__);
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

#endif

  // clock synchronization
  if (systemClock().setTimeFromNTP())
  {
    time_since_sleep = preferences.getUInt(PREFERENCES_LAST_SLEEP_TIME, 0);
    time_since_sleep = time_since_sleep ? systemClock().getTime() - time_since_sleep : 0; // may be can be used even if no sync
  }
  else
  {
    time_since_sleep = 0;
    Log.info("%s [%d]: Time wasn't synced.\r\n", __FILE__, __LINE__);
  }

  Log.info("%s [%d]: Time since last sleep: %d\r\n", __FILE__, __LINE__, time_since_sleep);

  if (preferences.isKey(PREFERENCES_API_KEY) && preferences.isKey(PREFERENCES_FRIENDLY_ID))
  {
    Log.info("%s [%d]: API key and friendly ID saved\r\n", __FILE__, __LINE__);
  }
  else
  {
    Log.info("%s [%d]: API key or friendly ID not saved\r\n", __FILE__, __LINE__);

    // attempt to get the API key and friendly ID
    DeviceSetupResult setup = deviceSetup().perform();

    // copy results into globals
    setup.imageUrl.toCharArray(filename, sizeof(filename));
    setup.message.toCharArray(message_buffer, sizeof(message_buffer));

    // perform post-setup actions
    if (setup.showSetupScreen)
    {
      display_show_msg(storedLogoOrDefault(0), FRIENDLY_ID, setup.friendlyId, true, "", setup.message);
      need_to_refresh_display = 0;
    }
    else if (setup.errorScreen != NONE)
    {
      showMessageWithLogo(setup.errorScreen);
    }

    if (setup.outcome == DeviceSetupOutcome::MacNotRegistered)
    {
      showMessageWithLogo(MAC_NOT_REGISTERED, setup.apiResponse);
      refreshInterval.applyDefault();
    }

    if (setup.shouldGoToSleep)
    {
      display_sleep();
      goToSleep();
    }
  }

  submitStoredLogs();

  log_retry = true;

  // OTA checking, image checking and drawing
  https_request_err_e request_result = downloadAndShow();
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, szHTTPErrors[request_result]);

  if (request_result == HTTPS_IMAGE_FILE_TOO_BIG)
  {
    iqs323_task_i2c_lock();
    showMessageWithLogo(MSG_TOO_BIG);
    iqs323_task_i2c_unlock();
  }

  if (!preferences.isKey(PREFERENCES_CONNECT_API_RETRY_COUNT))
  {
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }

  if (request_result != HTTPS_SUCCESS && request_result != HTTPS_NO_ERR && request_result != HTTPS_NO_REGISTER && request_result != HTTPS_RESET && request_result != HTTPS_PLUGIN_NOT_ATTACHED && current_msg != WIFI_FAILED)
  {
    uint8_t retries = preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT);
    iqs323_task_i2c_lock();

    switch (retries)
    {
    case 1:
    case 2:
    case 3:
    {
      uint32_t retry_sleep = refreshInterval.applyApiRetry(retries);
      Log.info("%s [%d]: retry: %d - time to sleep: %d\r\n", __FILE__, __LINE__, retries, retry_sleep);
      preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, ++retries);
      display_sleep();
      goToSleep();
      break;
    }

    default:
      Log.info("%s [%d]: Max retries done. Time to sleep: %d\r\n", __FILE__, __LINE__, refreshInterval.applyApiRetry(retries));
      preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, ++retries);
      break;
    }
    iqs323_task_i2c_unlock();
  }

  else
  {
    Log_info("Connection done successfully or WiFi failed. Retries counter reset.");
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }

  submitStoredLogs();

  if (request_result == HTTPS_NO_REGISTER && need_to_refresh_display == 1)
  {
    // show the image
    String friendly_id = preferences.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
    showMessageWithLogo(FRIENDLY_ID, friendly_id, true, "", String(message_buffer));
    need_to_refresh_display = 0;
  }

  // reset checking
  if (request_result == HTTPS_RESET)
  {
    Log.info("%s [%d]: Device reseting...\r\n", __FILE__, __LINE__);
    resetDeviceCredentials();
  }

  // OTA update checking
  if (firmwareUpdateService.isUpdateDue(
          apiDisplayResult.response.update_firmware,
          apiDisplayResult.response.firmware_url))
  {
    showMessageWithLogo(FW_UPDATE);
    FirmwareUpdateResult firmwareUpdateResult = firmwareUpdateService.performUpdate();
    if (firmwareUpdateResult.updated)
    {
      showMessageWithLogo(FW_UPDATE_SUCCESS);
      ESP.restart();
    }
    if (firmwareUpdateResult.failureMessage != NONE)
      showMessageWithLogo(firmwareUpdateResult.failureMessage);
  }

  // error handling
  switch (request_result)
  {
  case HTTPS_REQUEST_FAILED:
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_REQUEST_FAILED);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
  }
  break;
  case HTTPS_RESPONSE_CODE_INVALID:
  {
    showMessageWithLogo(WIFI_INTERNAL_ERROR);
  }
  break;
  case HTTPS_UNABLE_TO_CONNECT:
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_UNABLE_TO_CONNECT);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
  }
  break;
  case HTTPS_WRONG_IMAGE_FORMAT:
  {
    showMessageWithLogo(MSG_FORMAT_ERROR);
  }
  break;
  case HTTPS_WRONG_IMAGE_SIZE:
  {
    if (WiFi.RSSI() > WIFI_CONNECTION_RSSI)
    {
      showMessageWithLogo(API_SIZE_ERROR);
    }
    else
    {
      showMessageWithLogo(WIFI_WEAK);
    }
  }
  break;
  case HTTPS_CLIENT_FAILED:
  {
    showMessageWithLogo(WIFI_INTERNAL_ERROR);
  }
  break;
  case HTTPS_TIMED_OUT:
  {
    showMessageWithLogo(WIFI_IMAGE_TIMEOUT);
  }
  break;
  case HTTPS_PLUGIN_NOT_ATTACHED:
  {
    refreshInterval.applyFastPoll();
  }
  break;
  default:
    break;
  }

  // display go to sleep
  Log_info("%s [%d]: BL done, going to sleep...", __FILE__, __LINE__);
  display_sleep();
  goToSleep();
} /* bl_init() */

/**
 * @brief Function to process business logic module
 * @param none
 * @return none
 */
void bl_process(void)
{
}

void load_prev_image(void)
{
  uint8_t *buffer;
  size_t content_size = 0;
  if (content_size > 0) {
    // Decode it into the previous buffer
    Log.info("%s [%d]: Decoding previous image (%s) into the EPD 'old' buffer\r\n", __FILE__, __LINE__, DisplayedImage::get());
    png_to_epd(buffer, content_size, true);
  }
} /* load_prev_image() */

/**
 * @brief Function to reset the friendly id, API key, WiFi SSID and password
 * @param url Server URL address
 * @return none
 */
static void resetDeviceCredentials(void)
{
  Log.info("%s [%d]: The device will be reset now...\r\n", __FILE__, __LINE__);
  Log.info("%s [%d]: WiFi resetting...\r\n", __FILE__, __LINE__);
  WifiCaptivePortal.resetSettings();
  need_to_refresh_display = 1;
  bool res = preferences.clear();
  if (res)
    Log.info("%s [%d]: The device reset success. Restarting...\r\n", __FILE__, __LINE__);
  else
    Log.error("%s [%d]: The device resetting error. The device will be reset now...\r\n", __FILE__, __LINE__);
  preferences.end();
  ESP.restart();
}

/**
 * @brief Function to sleep preparing and go to sleep
 * @param none
 * @return none
 */
void goToSleep(void)
{
  Log.info("%s [%d]: go to sleep\r\n", __FILE__, __LINE__);
  submitStoredLogs();

// DEBUG - workaround to prevent crash in the WiFi stack of unknown origin
#ifndef BOARD_X_CLASS
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.mode(WIFI_OFF); 
#endif

#if BOARD_TRMNL_X
  Log_info("Preparing IQS323 for sleep via task...");

  // Use task manager for sleep preparation (sets event mode, checks I2C health)
  if (!iqs323_task_prepare_sleep(5000)) {
    Log.warning("IQS323 sleep preparation timeout - proceeding anyway\n");
  }

  // Configure gesture mode last so prepare_sleep's writeMM() cannot override it
  iqs323_task_i2c_lock();
  iqs323.setGestureConfig(touchbar_tap_mode, STOP);
  iqs323_task_i2c_unlock();

  // Cleanup the task before entering deep sleep
  iqs323_task_deinit();
  Log_info("IQS323 is ready for sleep.");

  esp_set_deep_sleep_wake_stub(*wakeup_stub);
  display_sleep();
  config_tca95535_pins_for_lp();
  config_gpio_for_lp();
#endif

  filesystem_deinit();
  uint32_t time_to_sleep = refreshInterval.seconds();
  iPrevWakeTime = millis() - startup_time; // save for statistics
  Log.info("%s [%d]: total awake time - %d ms\r\n", __FILE__, __LINE__, iPrevWakeTime); 
  Log.info("%s [%d]: time to sleep - %d\r\n", __FILE__, __LINE__, time_to_sleep);
  preferences.putUInt(PREFERENCES_LAST_SLEEP_TIME, systemClock().getTime());
  preferences.end();
  esp_sleep_enable_timer_wakeup((uint64_t)time_to_sleep * SLEEP_uS_TO_S_FACTOR);
  // Configure GPIO pin for wakeup
#if CONFIG_IDF_TARGET_ESP32
  #define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK(PIN_INTERRUPT), ESP_EXT1_WAKEUP_ALL_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined (CONFIG_IDF_TARGET_ESP32C5)
  pinMode(PIN_INTERRUPT, INPUT); // needed to not immediately wake up
  esp_deep_sleep_enable_gpio_wakeup(1 << PIN_INTERRUPT, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#else
#error "Unsupported ESP32 target for GPIO wakeup configuration"
#endif
#ifdef BOARD_XTEINK_X4
// The Xteink X4 has a high current draw in deep sleep (3-4mA), so allow the user to select
// if they want to completely shut down the power and only update with a physical button press
// or have short battery life (5-7 days) in the normal TRMNL wakeup mode
#ifdef X4_WAKE_ON_BUTTON
  pinMode(13, OUTPUT);
  digitalWrite(13, 0); // cut off the battery power
  delay(100); // allow it to settle before going into power-off
#else // Keep the battery power on and allow timed wakeup
  gpio_hold_en(GPIO_NUM_13); // MOSFET enabling the battery power
  gpio_deep_sleep_hold_en(); // Needed to keep the battery power enabled during RTC sleep
#endif
#endif
  esp_deep_sleep_start();
}

static void goToSleepButtonOnly(void)
{
  submitStoredLogs();
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.mode(WIFI_OFF);
  filesystem_deinit();
  preferences.end();
#if CONFIG_IDF_TARGET_ESP32
  #define BUTTON_PIN_BITMASK_BTN(GPIO) (1ULL << GPIO)
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK_BTN(PIN_INTERRUPT), ESP_EXT1_WAKEUP_ALL_LOW);
#elif defined( CONFIG_IDF_TARGET_ESP32C3 ) || defined ( CONFIG_IDF_TARGET_ESP32C5 )
  esp_deep_sleep_enable_gpio_wakeup(1 << PIN_INTERRUPT, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32S3
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#else
#error "Unsupported ESP32 target for GPIO wakeup configuration"
#endif
#ifdef BOARD_XTEINK_X4
  gpio_hold_en(GPIO_NUM_13);
  gpio_deep_sleep_hold_en();
#endif
  esp_deep_sleep_start();
}
void config_gpio_for_lp() {

#ifdef BOARD_TRMNL_X
  // XCL
  pinMode(GPIO_NUM_4, INPUT);

  // Data pins (d8 to d15)
  pinMode(GPIO_NUM_5, INPUT);
  pinMode(GPIO_NUM_6, INPUT);
  pinMode(GPIO_NUM_7, INPUT);
  pinMode(GPIO_NUM_15, INPUT);
  pinMode(GPIO_NUM_16, INPUT);
  pinMode(GPIO_NUM_17, INPUT);
  pinMode(GPIO_NUM_18, INPUT);
  pinMode(GPIO_NUM_8, INPUT);

  // D+ D-
  // pinMode(GPIO_NUM_19, INPUT);
  // pinMode(GPIO_NUM_20, INPUT);

  // Data pins (d0 to d7)
  pinMode(GPIO_NUM_9, INPUT);
  pinMode(GPIO_NUM_10, INPUT);
  pinMode(GPIO_NUM_11, INPUT);
  pinMode(GPIO_NUM_12, INPUT);
  pinMode(GPIO_NUM_13, INPUT);
  pinMode(GPIO_NUM_14, INPUT);
  pinMode(GPIO_NUM_21, INPUT);
  pinMode(GPIO_NUM_47, INPUT);

  // EP_STV
  pinMode(GPIO_NUM_48, INPUT);

  // CKV
  pinMode(GPIO_NUM_45, INPUT);

  // BTN1
  pinMode(GPIO_NUM_0, INPUT);

  // I2C
  pinMode(GPIO_NUM_39, INPUT); // SDA
  pinMode(GPIO_NUM_40, INPUT); // SCL

  // XSTL
  pinMode(GPIO_NUM_41, INPUT);

  // LEH
  pinMode(GPIO_NUM_42, INPUT);

  // UART0
  pinMode(GPIO_NUM_43, INPUT); // TXD
  pinMode(GPIO_NUM_44, INPUT); // RXD
  pinMode(GPIO_NUM_1, INPUT); // CTS
  pinMode(GPIO_NUM_2, INPUT); // RTS
#endif // BOARD_TRMNL_X
} /* config_gpio_for_lp() */

/**
 * @brief Function to submit a log string to the API
 * @param log_buffer pointer to the buffer that contains log note
 * @return bool true if successful, false if failed
 */
bool submitLogString(const char *log_buffer)
{
  String api_key = "";
  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    api_key = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log_info("%s key exists. Value - %s", PREFERENCES_API_KEY, api_key.c_str());
  }
  else
  {
    Log_info("%s key not exists.", PREFERENCES_API_KEY);
    return false;
  }

  LogApiInput input{api_key, log_buffer};
  return submitLogToApi(input, preferences.getString(PREFERENCES_API_URL, API_BASE_URL).c_str());
}

/**
 * @brief Function to store a log string locally
 * @param log_buffer pointer to the buffer that contains log note
 * @return bool true if successful, false if failed
 */
bool storeLogString(const char *log_buffer)
{
  LogStoreResult store_result = storedLogs.store_log(String(log_buffer));
  if (store_result.status != LogStoreResult::SUCCESS)
  {
    // Use the serial-only variant to avoid infinite recursion: Log_error here
    // would route back into log_impl → handle_store_submit → logWithAction →
    // storeLogString → store fails again → ...
    Log_error_serial("Failed to store log: %s", store_result.message);
    return false;
  }
  return true;
}


void submitStoredLogs(void)
{
  if (WiFi.isConnected() == false)
  {
    Log_info("WiFi not connected; not submitting stored logs.");
    return;
  }
  String log = storedLogs.gather_stored_logs();

  String api_key = "";
  if (preferences.isKey(PREFERENCES_API_KEY))
  {
    api_key = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, api_key.c_str());
  }
  else
  {
    Log.error("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  bool submitLogToApiResult = false;
  if (log.length() > 0)
  {
    Log.info("%s [%d]: log string - %s\r\n", __FILE__, __LINE__, log.c_str());
    Log.info("%s [%d]: need to send the log\r\n", __FILE__, __LINE__);

    LogApiInput input{api_key, log.c_str()};
    submitLogToApiResult = submitLogToApi(input, preferences.getString(PREFERENCES_API_URL, API_BASE_URL).c_str());
  }
  else
  {
    Log.info("%s [%d]: no needed to send the log\r\n", __FILE__, __LINE__);
  }
  if (submitLogToApiResult == true)
  {
    storedLogs.clear_stored_logs();
  }
}

void writeSpecialFunction(SPECIAL_FUNCTION function)
{
  if (preferences.isKey(PREFERENCES_SF_KEY))
  {
    Log.info("%s [%d]: SF saved. Reading...\r\n", __FILE__, __LINE__);
    if ((SPECIAL_FUNCTION)preferences.getUInt(PREFERENCES_SF_KEY, 0) == function)
    {
      Log.info("%s [%d]: No need to re-write\r\n", __FILE__, __LINE__);
    }
    else
    {
      Log.info("%s [%d]: Writing new special function\r\n", __FILE__, __LINE__);
      bool res = preferences.putUInt(PREFERENCES_SF_KEY, function);
      if (res)
        Log.info("%s [%d]: Written new special function successfully\r\n", __FILE__, __LINE__);
      else
        Log.error("%s [%d]: Writing new special function failed\r\n", __FILE__, __LINE__);
    }
  }
  else
  {
    Log.error("%s [%d]: SF not saved\r\n", __FILE__, __LINE__);
    bool res = preferences.putUInt(PREFERENCES_SF_KEY, function);
    if (res)
      Log.info("%s [%d]: Written new special function successfully\r\n", __FILE__, __LINE__);
    else
      Log.error("%s [%d]: Writing new special function failed\r\n", __FILE__, __LINE__);
  }
}

static void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message)
{
  display_show_msg(storedLogoOrDefault(0), message_type, friendly_id, id, fw_version, message);
  need_to_refresh_display = 1;
  preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
}

void showMessageWithLogo(MSG message_type)
{
  display_show_msg(storedLogoOrDefault(0), message_type);
}

/**
 * @brief Show a message with the logo using data from API setup response
 * @param message_type Type of message to display
 * @param apiResponse The API setup response containing the message
 * @return none
 */
static void showMessageWithLogo(MSG message_type, const ApiSetupResponse &apiResponse)
{
  display_show_msg(storedLogoOrDefault(0), message_type, "", false, "", apiResponse.message);
  need_to_refresh_display = 1;
  preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
}

// 0 = larger glyph for message screens
// 1 = loading screen (mostly blank, small glyph in lower right corner)
static uint8_t *storedLogoOrDefault(int iType)
{
//
// See if there are custom art assets in FLASH memory.
// The top 4K of FLASH would be reserved for this data.
// The images are stored as: logo_medium, loading
//
   uint32_t u32Size;
   //esp_flash_t chip;
   uint8_t *s;
   uint16_t u16Size;
   BRAND *pBrand;

   u32Size = ESP.getFlashChipSize();
   Log_info("%s [%d]: esp flash size: %" PRIu32 "\r\n", __FILE__, __LINE__, u32Size);
   if (u32Size != 0) {
   pBrand = (BRAND *)malloc(sizeof(BRAND)); // DEBUG - we can leak this memory for now
   esp_flash_init(NULL);
   esp_flash_read(NULL, (void *)pBrand, u32Size-sizeof(BRAND), sizeof(BRAND));
   if (*(uint16_t *)&pBrand->u8Images[0] == 0xBBBF /*BB_BITMAP_MARKER*/) {
      // Group5 compressed images are present, use them
      if (iType == 0) {
        return &pBrand->u8Images[0]; // the first image is the medium sized logo
      } else { // the second image is the loading screen with small logo
        // get the pointer to the loading image
        s = &pBrand->u8Images[0];
        u16Size = *(uint16_t *)&s[6]; // compressed image size
        s += u16Size + 8; // skip to loading image
        return s;
      }
   }
  }
#ifdef BOARD_X_CLASS
    return const_cast<uint8_t *>(logo_medium);
#else
  if (iType == 0) {
    return const_cast<uint8_t *>(logo_small);
  } else {
    // Force the loading screen to always use the slower update method because
    // we don't know (yet) if the panel can handle the faster update modes
    apiDisplayResult.response.maximum_compatibility = true;
    return const_cast<uint8_t *>(loading);
  }
#endif
}

// Chop up long names to fit within the SPIFFS 31 character limit


static void wifiErrorDeepSleep()
{
  if (!preferences.isKey(PREFERENCES_CONNECT_WIFI_RETRY_COUNT))
  {
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

  uint8_t retry_count = preferences.getInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT);

  Log_info("WIFI connection failed! Retry count: %d \n", retry_count);

  switch (retry_count)
  {
  case 1:
  case 2:
  case 3:
    refreshInterval.applyWifiRetry(retry_count);
    break;

  default:
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
    showMessageWithLogo(WIFI_RETRY_LIMIT);
    display_sleep();
    goToSleepButtonOnly();
    return;
  }
  retry_count++;
  preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, retry_count);

  display_sleep();
  goToSleep();
}

DeviceStatusStamp getDeviceStatusStamp()
{
  DeviceStatusStamp deviceStatus = {};

  deviceStatus.wifi_rssi_level = WiFi.RSSI();
  strncpy(deviceStatus.wifi_status, wifiStatusStr(WiFi.status()), sizeof(deviceStatus.wifi_status) - 1);
  deviceStatus.refresh_rate = refreshInterval.seconds(0);
  deviceStatus.time_since_last_sleep = time_since_sleep;
  snprintf(deviceStatus.current_fw_version, sizeof(deviceStatus.current_fw_version), "%s", FW_VERSION_STRING);
  parseSpecialFunctionToStr(deviceStatus.special_function, sizeof(deviceStatus.special_function), special_function);
  deviceStatus.battery_voltage = vBatt;
  parseWakeupReasonToStr(deviceStatus.wakeup_reason, sizeof(deviceStatus.wakeup_reason), esp_sleep_get_wakeup_cause());
  deviceStatus.free_heap_size = ESP.getFreeHeap();
  deviceStatus.max_alloc_size = ESP.getMaxAllocHeap();

  return deviceStatus;
}

void logWithAction(LogAction action, LogLevel level, const char *message, time_t time, int line, const char *file)
{
  uint32_t log_id = preferences.getUInt(PREFERENCES_LOG_ID_KEY, 1);

  LogWithDetails input = {
      .deviceStatusStamp = getDeviceStatusStamp(),
      .timestamp = time,
      .codeline = line,
      .sourceFile = file,
      .logMessage = message,
      .logId = log_id,
      .filenameCurrent = preferences.getString(PREFERENCES_FILENAME_KEY, ""),
      .filenameNew = new_filename,
      .logRetry = log_retry,
      .retryAttempt = log_retry ? preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT) : 0,
      .level = level};

  String json_string = serialize_log(input);

  switch (action)
  {
    case LOG_ACTION_STORE:
      storeLogString(json_string.c_str());
      break;
    case LOG_ACTION_SUBMIT:
      submitLogString(json_string.c_str());
      break;
    case LOG_ACTION_SUBMIT_OR_STORE:
      if (!submitLogString(json_string.c_str()))
      {
        Log_info("Was unable to send log to API; saving locally for later.");
        storeLogString(json_string.c_str());
      }
      break;
  }

  preferences.putUInt(PREFERENCES_LOG_ID_KEY, ++log_id);
}