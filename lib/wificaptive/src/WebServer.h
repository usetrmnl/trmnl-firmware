#pragma once
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <functional>
#include <vector>

#include "WifiCaptivePage.h"
#include "wifi-types.h"

#define LocalIPURL "http://4.3.2.1"

/** These are the things that the captive portal web app needs to interact with */
struct WifiOperationCallbacks {
  std::function<void()> resetSettings;
    /// Starts a join: credentials, API server (applied only when `api_server_given`), band ("" = auto).
  std::function<void(const WifiCredentials, const String, bool, const String)> setConnectionCredentials;
  std::function<WifiConnectionState()> getConnectionState;
  std::function<std::vector<WifiNetwork>(bool)> getAnnotatedNetworks;
    /// Returns true when the network list is ready to serve (external list populated or WiFi scan complete).
  std::function<bool()> isNetworkListReady;
    /// Discards any cached scan results and starts a fresh scan (native WiFi or modem, whichever applies).
  std::function<void()> forceRescan;
    /// Closes the portal so the device goes back to its saved networks; false when none are saved.
  std::function<bool()> requestExit;
};

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP, WifiOperationCallbacks callbacks,
                    const String &modemMac = "");