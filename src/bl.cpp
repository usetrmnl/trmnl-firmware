#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <app_logger.h>
#include <bl.h>
#include <http_result_handling.h>
#include <sleep_session.h>
#include <wifi_network.h>
#include <wifi_session.h>
#include <power.h>
#include <config.h>
#include <battery.h>
#include <device_id.h>
#include <trmnl_log.h>
#include <types.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <pins.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <display.h>
#include <display_session.h>
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
#include <services/http_retry_request.h>
#include "driver/gpio.h"
#include <nvs.h>
#include <serialize_log.h>
#include <preferences_persistence.h>
#include <wifi-helpers.h>
#include <sys/time.h>
#include <misc/buzzer.h>
#include <misc/sensor.h>
#include <services/device_setup.h>
#include "messages.h"
#include "displayed_image.h"
#include <globals.h>
#include <status_screens.h>

unsigned long startup_time = 0;

// I2C bus pins, shared by the IQS323 touch controller and the BQ27427 battery gauge
#define SENSOR_SDA_PIN 39
#define SENSOR_SCL_PIN 40

#ifndef BOARD_TRMNL_X
// Create stub functions for the touchbar workaround
void iqs323_task_i2c_lock(void) {}
void iqs323_task_i2c_unlock(void) {}
#endif // !BOARD_TRMNL_X
void hw_config_init(void);
extern TRMNL_DEVICE *pDevice;

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
// ############################ IQS323 TASK #############################
#include "iqs323_task.h"
// ############################ IQS323 TASK #############################

// ############################ SLIDER ##################################
#include "touchbar_gesture.h"
#include "touchbar_actions.h"

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
  startup_time = init_time/1000L; // convert to milliseconds
#endif
#ifdef DEV_FIRMWARE
  Serial.begin(115200);
  wait_for_serial();
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
#endif
  Log_info("BL init success");

  wifiSessionInit();

  hw_config_init();
  pins_init();
  buzzer().init();
  sensor().init(pDevice);
#ifdef BOARD_TRMNL_X
  touchbar_log_wake_stub_status();
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
  bool should_show_error_now = (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER); // don't immediately show errors on timer wakeup
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
    ButtonPressResult button = read_button_presses();
    wait_for_serial();
    double_click = button_wake_act(button);
  }
  else
  {
    wait_for_serial();
    Log_info("Non-GPIO wakeup (%d) -> didn't read buttons", wakeup_reason);
  }
#else // BOARD_TRMNL_X
  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  iqs323_task_i2c_lock();
  display_init();
  otg_turn_off(); // Since OTG function was commented out, need to ensure that OTG is turned off
  iqs323_task_i2c_unlock();
  filesystem_init();

  Wire.setClock(100000);
#endif // BOARD_TRMNL_X

#ifdef BOARD_TRMNL_X
  touchbar_start(wakeup_reason, gpio_wakeup);

  Log_info("init time: %ld us", init_time);
#else // BOARD_TRMNL_X

  if (double_click)
    button_wake_run_special_function();
  // Read the battery voltage BEFORE the display or WiFi is turned on
  battery_voltage = battery().readVoltage(pDevice);

  // EPD init
  // EPD clear
  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  display_init();

  // Mount SPIFFS
  filesystem_init();
#endif // !BOARD_TRMNL_X

#ifdef BOARD_TRMNL_X
  // Read the gauge before the panel draws load current, and before the logo
  // is drawn so its battery icon has a snapshot to read.
  battery_count = detect_battery_count();
  battery_charging = (power().chargingStatus() == ChargingStatus::CHARGING);
  Log_info("BATTERY COUNT: %d", battery_count);
  Log_info("BATTERY CHARGING: %s", battery_charging ? "YES" : "NO");

  battery().gaugeInit();
  battery_voltage = battery().readVoltage(); // Read the battery voltage BEFORE WiFi is turned on
#endif // BOARD_TRMNL_X

  if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER)
  {
    showWakeLogo();
  }
  Log_info("Firmware version %s", Messages::firmware_version().c_str());
  Log_info("Arduino version %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Log_info("ESP-IDF version %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
  list_files();
  log_nvs_usage();

  wifiSessionConnect(should_show_error_now);

  updateTimeSinceSleep();

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
      // The server's image for an unregistered device when it sent one and this run saved it
      // (not a logo left on flash by an earlier setup), else the server's message under the logo.
      bool drawable = setup.imagePath.length() > 0;
#ifdef PARALLEL_EPD
      // FastEPD's display_show_image draws no uncompressed BMP (the X's 5 GHz modem path saves only those)
      drawable = drawable && !setup.imagePath.endsWith(".bmp");
#endif
      int image_size = 0;
      uint8_t *image = drawable ? display_read_file(setup.imagePath.c_str(), &image_size) : nullptr;
      if (image)
      {
        display_show_image(image, image_size, true);
        free(image);
      }
      else
        showMessageWithLogo(MAC_NOT_REGISTERED, setup.apiResponse);
      preferences.putBool(PREFERENCES_DEVICE_REGISTERED_KEY, false);
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
  https_request_err_e request_result = downloadAndShow(battery_voltage);
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, https_request_err_str(request_result));

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

  uint8_t retries = preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT);
  RequestFailureAction failure_action = requestFailureAction(request_result, should_show_error_now, retries);

  if (failure_action == RequestFailureAction::ResetCount)
  {
    if (request_result == HTTPS_IMAGE_DOWNLOAD_FAILED) // API answered, so no backoff
      Log_info("Image download failed; showing error and sleeping for the API refresh interval (%d s)", refreshInterval.seconds());
    else
      Log_info("Connection done successfully or WiFi failed. Retries counter reset.");
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, 1);
  }
  else if (failure_action == RequestFailureAction::ShowNow)
  {
    Log.info("%s [%d]: Manual wake - showing error now, no retry scheduled\r\n", __FILE__, __LINE__);
  }
  else
  {
    uint32_t retry_sleep = refreshInterval.applyApiRetry(retries);
    preferences.putInt(PREFERENCES_CONNECT_API_RETRY_COUNT, retries + 1);

    if (failure_action == RequestFailureAction::StayQuiet)
    {
      Log.info("%s [%d]: retry: %d - time to sleep: %d\r\n", __FILE__, __LINE__, retries, retry_sleep);
      iqs323_task_i2c_lock();
      display_sleep();
      goToSleep();
    }
    Log.info("%s [%d]: Max retries done. Time to sleep: %d\r\n", __FILE__, __LINE__, retry_sleep);
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

  if (request_result == HTTPS_PLUGIN_NOT_ATTACHED)
    refreshInterval.applyFastPoll();
  MSG error_screen = requestErrorScreen(request_result, WiFi.RSSI() <= WIFI_CONNECTION_RSSI);
  if (error_screen != NONE)
    showMessageWithLogo(error_screen);

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

/**
 * @brief Function to reset the friendly id, API key, WiFi SSID and password
 * @param url Server URL address
 * @return none
 */
void resetDeviceCredentials(void)
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
