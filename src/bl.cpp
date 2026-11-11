#include <Arduino.h>
#include <ArduinoLog.h>
#include <app_logger.h>
#include <bl.h>
#include <display.h>
#include <display_session.h>
#include <globals.h>
#include <request_handling.h>
#include <setup_session.h>
#include <sleep_session.h>
#include <status_screens.h>
#include <trmnl_log.h>
#include <wake_init.h>
#include <wifi_session.h>

#ifdef BOARD_TRMNL_X
#include <touchbar_actions.h>
#endif

// Fetches the screen from /api/display, shows it (or the error), then handles what the response asks for
static void showScreen(bool should_show_error_now) {
  https_request_err_e request_result = downloadAndShow(battery_voltage);
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, https_request_err_str(request_result));
  showImageTooBigIfNeeded(request_result);
  applyRetryPolicy(request_result, should_show_error_now); // may sleep
  submitStoredLogs();
  showFriendlyIdIfUnregistered(request_result);
  resetIfRequested(request_result);
  updateFirmwareIfDue();
  showRequestResult(request_result);
}

#ifdef LIVE_MODE
#include <WiFi.h>
uint32_t live_loop_ms = 0;
uint32_t live_wifi_disconnects = 0;
uint8_t live_wifi_last_reason = 0; // wifi_err_reason_t of the last disconnect; 0 = none yet
uint32_t live_wifi_last_disconnect_ms = 0;

String live_wifi_summary(void) {
  return "bssid=" + WiFi.BSSIDstr() + ";ch=" + String(WiFi.channel()) + ";rssi=" + String(WiFi.RSSI()) +
         ";drops=" + String(live_wifi_disconnects) + ";reason=" + String(live_wifi_last_reason) +
         ";ago_ms=" + String(live_wifi_disconnects ? millis() - live_wifi_last_disconnect_ms : 0);
}

// While the server answers keep_wifi_on, waits refresh_rate seconds with Wi-Fi connected and checks in again
static void runLiveSession(void) {
  if (!apiDisplayResult.response.keep_wifi_on) return;
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    live_wifi_disconnects++;
    live_wifi_last_reason = info.wifi_sta_disconnected.reason;
    live_wifi_last_disconnect_ms = millis();
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  Log_info("Staying awake between check-ins");
  uint32_t check_in_ms = apiDisplayResult.response.refresh_rate * 1000;
  uint32_t last_answer_ms = apiDisplayResult.received_ms;
  int failures = 0;
  while (failures < 3) { // a failed check-in has no answer to say otherwise, so it keeps the last one
    if (millis() - last_answer_ms < check_in_ms) delay(check_in_ms - (millis() - last_answer_ms));
    const uint32_t started = millis();
    https_request_err_e result = downloadAndShow(battery_voltage);
    live_loop_ms = millis() - started;
    if (result != HTTPS_SUCCESS && result != HTTPS_NO_ERR) {
      failures++;
      Log_error("Live check-in failed (%d), Wi-Fi %s", result, live_wifi_summary().c_str());
      continue;
    }
    failures = 0;
    last_answer_ms = apiDisplayResult.received_ms;
    if (!apiDisplayResult.response.keep_wifi_on) break;
    check_in_ms = apiDisplayResult.response.refresh_rate * 1000;
  }
  Log_info("Live session over");
}
#endif

/**
 * @brief Function to init business logic module
 * @param none
 * @return none
 */
void bl_init(void) {
  uint32_t init_time = bootEarly();
  wifiSessionInit();
  initPeripherals();
  WakeContext wake = readWakeContext();
  openPreferences();
  startBoardHardware(wake, init_time);
  if (wake.reason != ESP_SLEEP_WAKEUP_TIMER) showWakeLogo();
  logVersions();

  wifiSessionConnect(wake.showErrorsNow);
  updateTimeSinceSleep();
  ensureDeviceSetUp(); // may sleep
  submitStoredLogs();
  log_retry = true;

  // OTA checking, image checking and drawing
  showScreen(wake.showErrorsNow);
#ifdef LIVE_MODE
  runLiveSession();
#endif
#ifdef BOARD_TRMNL_X
  touchbar_stay_awake([] { showScreen(true); });
#endif

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
void bl_process(void) {}
