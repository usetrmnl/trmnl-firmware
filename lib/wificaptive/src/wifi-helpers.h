#pragma once
#include <WiFiType.h>
#include <Arduino.h>

void applyWifiHostname(const String &hostname);
const char *wifiStatusStr(wl_status_t wifi_status);
