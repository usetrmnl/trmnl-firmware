#include <Arduino.h>
#include <esp_idf_version.h>
#include <pairing_runtime.h>
#include <sdkconfig.h>
#if defined(TRMNL_BLE_PAIRING) && ESP_IDF_VERSION_MAJOR == 4
#include <atomic>
#include <esp_nimble_hci.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <phone_beacon.h>

namespace {
  std::atomic<bool> done{false}, found{false};
  uint8_t target[16];
  int scanEvent(ble_gap_event *event, void *) {
    if (event->type == BLE_GAP_EVENT_DISC && matchesPhoneBeacon(event->disc.data, event->disc.length_data, target)) {
      found = true;
      ble_gap_disc_cancel();
      done = true;
    } else if (event->type == BLE_GAP_EVENT_DISC_COMPLETE)
      done = true;
    return 0;
  }
  void scanReady() {
    uint8_t ownType;
    if (ble_hs_util_ensure_addr(0) || ble_hs_id_infer_auto(0, &ownType)) {
      done = true;
      return;
    }
    ble_gap_disc_params params = {};
    params.itvl = 0x10;
    params.window = 0x10; // continuous active scanning; include scan responses
    params.filter_duplicates = 0;
    if (ble_gap_disc(ownType, 500, &params, scanEvent, nullptr)) done = true;
  }
  void scanTask(void *) {
    nimble_port_run();
    nimble_port_freertos_deinit();
  }
} // namespace
bool scanForPhoneBeacon(const char *sessionId) {
  if (!phoneBeaconUuid(sessionId, target)) return false;
  done = false;
  found = false;
  if (esp_nimble_hci_and_controller_init() != ESP_OK) return false;
  nimble_port_init();
  ble_hs_cfg.sync_cb = scanReady;
  nimble_port_freertos_init(scanTask);
  const uint32_t started = millis();
  // Bound host initialization as well as the 500 ms radio scan.
  while (!done && millis() - started < 1500)
    delay(5);
  ble_gap_disc_cancel();
  if (nimble_port_stop() != 0) {
    // Never initialize a second host on top of a host that failed to stop.
    ESP.restart();
    return false;
  }
  nimble_port_deinit();
  esp_nimble_hci_and_controller_deinit();
  return found;
}
#else
// This probe is currently exercised on the IDF 4 OG/BWRY radio stack.
bool scanForPhoneBeacon(const char *) { return false; }
#endif
