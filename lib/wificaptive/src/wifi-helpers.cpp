#include "wifi-helpers.h"

#include <WiFi.h>

#include "esp_netif.h"

struct WifiStatusNode {
  const char *name;
  wl_status_t value;
};

const char *wifiStatusStr(wl_status_t wifi_status) {
  static const WifiStatusNode wifiStatusMap[] = {
      {"no_shield", WL_NO_SHIELD},
      {"idle_status", WL_IDLE_STATUS},
      {"no_ssid_avail", WL_NO_SSID_AVAIL},
      {"scan_completed", WL_SCAN_COMPLETED},
      {"connected", WL_CONNECTED},
      {"connect_failed", WL_CONNECT_FAILED},
      {"connection_lost", WL_CONNECTION_LOST},
      {"disconnected", WL_DISCONNECTED},
  };

  for (const WifiStatusNode &entry : wifiStatusMap) {
    if (wifi_status == entry.value) return entry.name;
  }
  // Callers strncpy this into a fixed-size log field, so it must never be null.
  // The 3.x core adds status values this table does not list.
  return "unknown";
}

void applyWifiHostname(const String &hostname) {
  if (hostname.length() == 0) return;

  WiFi.setHostname(hostname.c_str());
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif != nullptr) esp_netif_set_hostname(netif, hostname.c_str());
}
