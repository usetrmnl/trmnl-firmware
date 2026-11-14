#include <Arduino.h>
#include <ArduinoLog.h>
#include <app_logger.h>
#include <bl.h>
#include <display.h>
#include <display_session.h>
#include <globals.h>
#include <http_result_handling.h>
#include <keep_wifi_on.h>
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

// Fetches the screen from /api/display, shows it (or the error, when asked to), then handles what the answer asks for
static https_request_err_e showScreen(bool should_show_error_now, bool draw_error_screen_on_failure = true) {
  https_request_err_e request_result = downloadAndShow(battery_voltage);
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, https_request_err_str(request_result));
  if (draw_error_screen_on_failure) showImageTooBigIfNeeded(request_result);
  applyRetryPolicy(request_result, should_show_error_now); // may sleep
  submitStoredLogs();
  showFriendlyIdIfUnregistered(request_result);
  resetIfRequested(request_result);
  updateFirmwareIfDue();
  if (draw_error_screen_on_failure || !requestOrImageFailed(request_result)) showRequestResult(request_result);
  return request_result;
}

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
#ifdef BOARD_TRMNL_X
  touchbar_stay_awake([] { showScreen(true); });
#else
  stay_awake_while_server_asks(
    [](bool draw_error_screen_on_failure) { return showScreen(true, draw_error_screen_on_failure); });
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
