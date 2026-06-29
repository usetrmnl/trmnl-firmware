#include "wifi-helpers.h"
#include "../../../include/config.h"
#include <WiFi.h>
#include <Preferences.h>
#include "esp_netif.h"

struct WifiStatusNode
{
  const char *name;
  wl_status_t value;
};

const char *wifiStatusStr(wl_status_t wifi_status)
{
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

  for (const WifiStatusNode &entry : wifiStatusMap)
  {
    if (wifi_status == entry.value)
      return entry.name;
  }
  return nullptr;
}

String getWifiClientHostname(void)
{
  Preferences prefs;
  if (!prefs.begin("data", true))
    return String(WIFI_CLIENT_HOSTNAME_PREFIX);

  String customHostName = prefs.getString(PREFERENCES_HOSTNAME, "");
  if (customHostName.length() > 0)
  {
    prefs.end();
    return customHostName;
  }

  String hostname = WIFI_CLIENT_HOSTNAME_PREFIX;
  String friendly_id = prefs.getString(PREFERENCES_FRIENDLY_ID, PREFERENCES_FRIENDLY_ID_DEFAULT);
  prefs.end();
  if (friendly_id.length() > 0)
    hostname += "-" + friendly_id;
  // Alternative solution
  // String mac = WiFi.macAddress(); // "AA:BB:CC:DD:EE:FF"
  // String suffix = mac.substring(12, 14) + mac.substring(15, 17);
  // suffix.toLowerCase();
  // hostname += "-" + suffix;
  return hostname;
}

void configureWifiHostname(void)
{
  String hostname = getWifiClientHostname();
  WiFi.setHostname(hostname.c_str());
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif != nullptr)
    esp_netif_set_hostname(netif, hostname.c_str());
}
