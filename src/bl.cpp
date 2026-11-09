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
uint32_t live_loop_ms = 0;
// Keeps Wi-Fi up and checks in again straight away while the server keeps the session open
static void runLiveSession(void) {
  const uint32_t seconds = apiDisplayResult.response.live_mode_seconds;
  if (seconds == 0) return;
  Log_info("Live session for %u s", seconds);
  const uint32_t ends_at = millis() + seconds * 1000;
  while (millis() < ends_at && apiDisplayResult.response.live_mode_seconds > 0) {
    const uint32_t started = millis();
    https_request_err_e result = downloadAndShow(battery_voltage);
    live_loop_ms = millis() - started;
    if (result != HTTPS_SUCCESS && result != HTTPS_NO_ERR) delay(2000);
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
