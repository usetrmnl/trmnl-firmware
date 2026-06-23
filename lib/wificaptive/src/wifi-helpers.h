#pragma once
#include <WiFiType.h>
#include <Arduino.h>

String getWifiClientHostname(void);
void configureWifiHostname(void);
const char *wifiStatusStr(wl_status_t wifi_status);
