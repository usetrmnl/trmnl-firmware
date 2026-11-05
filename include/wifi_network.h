#ifndef WIFI_NETWORK_H
#define WIFI_NETWORK_H

#include <Arduino.h>

struct WiFiStatus {
  int rssi;    // signal strength in dBm
  String band; // "2.4" or "5"
};

/// @brief Resolve the current Wi-Fi status
WiFiStatus getWiFiStatus(void);

/// @brief Resolve the device hostname: custom hostname from preferences if set,
///        otherwise the board prefix plus friendly ID (e.g. "TRMNL-ABC123")
String getWifiClientHostname(void);

/// @brief Connect using credentials saved in WifiCaptivePortal
bool connectWithSavedCredentials(void);

/// @brief True when the TRMNL X's modem holds the network link (5 GHz), so requests go through it
bool usesModemNetwork(void);

/// @brief The modem's last signal reading, 0 when it has none or off the TRMNL X
int32_t modemLastSignalRssi(void);

/// @brief Ensure WiFi is connected, reconnecting with saved credentials if needed.
bool ensureWifiConnected(void);

#endif // WIFI_NETWORK_H
