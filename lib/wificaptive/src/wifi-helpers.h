#pragma once
#include <Arduino.h>
#include <WiFiType.h>

void applyWifiHostname(const String &hostname);
const char *wifiStatusStr(wl_status_t wifi_status);
