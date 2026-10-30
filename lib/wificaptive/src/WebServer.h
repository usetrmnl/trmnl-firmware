#pragma once
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <functional>
#include <vector>

#include "WifiCaptivePage.h"
#include "wifi-types.h"

#define LocalIPURL "http://4.3.2.1"

/** These are the things that the captive portal web app needs to interact with */
struct WifiNetworkList {
  bool ready;
  std::vector<WifiNetwork> networks;
};
struct WifiOperationCallbacks {
  std::function<bool()> resetSettings;
  std::function<bool(const WifiCredentials, const String, const String)> setConnectionCredentials;
  std::function<WifiConnectionState()> getConnectionState;
  /// Copies a published list; force only queues a fresh scan on the main task.
  std::function<WifiNetworkList(bool force)> getNetworks;
};

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP, WifiOperationCallbacks callbacks,
                    const String &modemMac = "");