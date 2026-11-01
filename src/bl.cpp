#include <Arduino.h>
#include <ArduinoLog.h>
#include <app_logger.h>
#include <bl.h>
#include <display.h>
#include <display_session.h>
#include <globals.h>
#include <pairing_runtime.h>
#include <request_handling.h>
#include <setup_session.h>
#include <sleep_session.h>
#include <status_screens.h>
#include <trmnl_log.h>
#include <wake_init.h>
#include <wifi_session.h>

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
  const bool bleSetup = pairingUsesBle();
  ensureDeviceSetUp(); // may sleep
  submitStoredLogs();
  log_retry = true;

  // OTA checking, image checking and drawing
  https_request_err_e request_result = downloadAndShow(battery_voltage);
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, https_request_err_str(request_result));
  showImageTooBigIfNeeded(request_result);
  applyRetryPolicy(request_result, wake.showErrorsNow); // may sleep
  submitStoredLogs();
  showFriendlyIdIfUnregistered(request_result, bleSetup);
  resetIfRequested(request_result);
  updateFirmwareIfDue();
  showRequestResult(request_result);

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
