#include "wifi-helpers.h"

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

const char *wifiStatusStr(wl_status_t wifi_status)
{
  for (const WifiStatusNode &entry : wifiStatusMap)
  {
    if (wifi_status == entry.value)
    {
      return entry.name;
    }
  }
  return nullptr;
}