#include <WiFiType.h>
#include <Arduino.h>

struct WifiStatusNode
{
  const char *name;
  wl_status_t value;
};

const char *wifiStatusStr(wl_status_t wifi_status);