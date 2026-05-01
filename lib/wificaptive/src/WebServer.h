#pragma once
#include <ESPAsyncWebServer.h>
#include <functional>
#include <vector>
#include <AsyncJson.h>
#include "wifi-types.h"
#include "WifiCaptivePage.h"

#define LocalIPURL "http://4.3.2.1"

/** These are the things that the captive portal web app needs to interact with */
struct WifiOperationCallbacks
{
    std::function<void()> resetSettings;
    std::function<void(const WifiCredentials, const String)> setConnectionCredentials;
    std::function<std::vector<WifiNetwork>(bool)> getAnnotatedNetworks;
    /// Returns true when the network list is ready to serve (external list populated or WiFi scan complete).
    std::function<bool()> isNetworkListReady;
};

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP, WifiOperationCallbacks callbacks);