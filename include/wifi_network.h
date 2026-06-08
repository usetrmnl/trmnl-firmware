#ifndef WIFI_NETWORK_H
#define WIFI_NETWORK_H

#include <Arduino.h>

struct WiFiStatus
{
  int rssi;    // signal strength in dBm
  String band; // "2.4" or "5"
};

/// @brief Resolve the current Wi-Fi status
WiFiStatus getWiFiStatus(void);

#endif // WIFI_NETWORK_H
