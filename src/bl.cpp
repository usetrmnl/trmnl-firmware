#include <Arduino.h>
#include <ArduinoLog.h>
#include <app_logger.h>
#include <bl.h>
#include <board_config.h>
#include <button.h>
#include <display.h>
#include <display_session.h>
#include <globals.h>
#include <http_result_handling.h>
#include <open_connections.h>
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
static https_request_err_e showScreen(bool should_show_error_now, bool show_error_screen = true) {
  https_request_err_e request_result = downloadAndShow(battery_voltage);
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, https_request_err_str(request_result));
  showImageTooBigIfNeeded(request_result);
  applyRetryPolicy(request_result, should_show_error_now); // may sleep
  submitStoredLogs();
  showFriendlyIdIfUnregistered(request_result);
  resetIfRequested(request_result);
  updateFirmwareIfDue();
  if (show_error_screen || !requestIsFailure(request_result)) showRequestResult(request_result);
  return request_result;
}

#ifndef BOARD_TRMNL_X
static volatile bool button_pressed_while_awake = false;
static void IRAM_ATTR latch_button_press(void) { button_pressed_while_awake = true; }

// Waits refresh_rate_ms, or else refresh_rate seconds, from the last answer. The check-in after it reads as a timer wake, or as a
// button wake after a press, even one made during a check-in, which is handled as on a button wake.
static void waitAwakeForNextCheckIn(void) {
  const ApiDisplayResponse &answer = apiDisplayResult.response;
  // the floors stop a 0 from hammering the API
  const uint32_t wait_ms = answer.refresh_rate_ms ? max<uint32_t>(answer.refresh_rate_ms, 250) : max<uint32_t>(answer.refresh_rate, 1) * 1000;
  wakeup_reason = ESP_SLEEP_WAKEUP_TIMER;
  while (millis() - apiDisplayResult.received_ms < wait_ms) {
    if (button_pressed_while_awake) {
      button_pressed_while_awake = false;
      wakeup_reason = button_wakeup_cause();
      // read_button_presses() waits for a new press when the button is already up
      ButtonPressResult press = digitalRead(pDevice->interrupt_pin) == LOW ? read_button_presses() : ShortPress;
      if (button_wake_act(press)) button_wake_run_special_function();
      return;
    }
    delay(20);
  }
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
#ifndef BOARD_TRMNL_X // the X stays awake for short refreshes and taps in touchbar_stay_awake
  if (apiDisplayResult.response.keep_wifi_on) {
    attachInterrupt(digitalPinToInterrupt(pDevice->interrupt_pin), latch_button_press, FALLING);
  }
  OpenConnections::keep(apiDisplayResult.response.keep_wifi_on);
  while (apiDisplayResult.response.keep_wifi_on) { // a check-in that keeps failing clears it, so errors sleep as usual
    awake_time_limit_restart(); // the limit catches a device stuck awake, not one the server keeps checking in
    waitAwakeForNextCheckIn();
    https_request_err_e result = showScreen(true, false);
    // one bad check-in would otherwise end the session until the next scheduled wake; only the last try shows its error
    for (int retry = 1; requestIsFailure(result) && retry <= 3; retry++) {
      delay(2000);
      result = showScreen(true, retry == 3);
    }
  }
  OpenConnections::keep(false);
  detachInterrupt(digitalPinToInterrupt(pDevice->interrupt_pin));
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
