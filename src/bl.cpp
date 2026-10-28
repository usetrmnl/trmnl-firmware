#include <Arduino.h>
#include <ArduinoLog.h>
#include <app_logger.h>
#include <bl.h>
#include <display.h>
#include <display_session.h>
#include <globals.h>
#ifdef BOARD_TRMNL_X
#include "ethernet_config.h"
#include "connection_manager.h"
#endif // BOARD_TRMNL_X
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
void bl_init(void)
{
  uint32_t init_time = bootEarly();
  wifiSessionInit();
  initPeripherals();
  WakeContext wake = readWakeContext();
  openPreferences();
  startBoardHardware(wake, init_time);
  if (wake.reason != ESP_SLEEP_WAKEUP_TIMER)
    showWakeLogo();
  logVersions();

#ifdef BOARD_TRMNL_X
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  conn_mgr_init();
  if (preferences.getBool("eth_enabled", false)) {
    ethernet_start();
    Log_info("Waiting for Ethernet DHCP");
    for (int _eth_s = 1; _eth_s <= 5; _eth_s++) {
      if ((conn_mgr_wait_any(pdMS_TO_TICKS(1000)) & ETH_READY_BIT) != 0) break;
      Log_info("No Ethernet yet... (waiting %ds)", _eth_s);
      if (_eth_s == 5) {
        Log_info("Ethernet DHCP still not ready after %ds, giving up and trying WiFi", _eth_s);
      }
    }
  }

  // WiFi block — runs when Ethernet is not available.
  if (!conn_mgr_is_eth())
#endif // BOARD_TRMNL_X
    wifiSessionConnect(wake.showErrorsNow);
  updateTimeSinceSleep();
  ensureDeviceSetUp(); // may sleep
  submitStoredLogs();
  log_retry = true;

  // OTA checking, image checking and drawing
  https_request_err_e request_result = downloadAndShow(battery_voltage);
  Log.info("%s [%d]: request result - %s\r\n", __FILE__, __LINE__, https_request_err_str(request_result));
  showImageTooBigIfNeeded(request_result);
  applyRetryPolicy(request_result, wake.showErrorsNow); // may sleep
  submitStoredLogs();
  showFriendlyIdIfUnregistered(request_result);
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
void bl_process(void)
{
}
