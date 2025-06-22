#pragma once
#include <ESPAsyncWebServer.h>
#include <functional>
#include <vector>
#include <AsyncJson.h>
#include "WifiCaptivePage.h"
#include "wifi-types.h"

#define LocalIPURL "http://4.3.2.1"

/** These are the things that the captive portal web app needs to interact with */
struct WifiOperationCallbacks
{
    std::function<void()> resetSettings;
    std::function<void(const WifiCredentials, const String)> setConnectionCredentials;
    std::function<std::vector<Network>(bool)> getAnnotatedNetworks;
};

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP, WifiOperationCallbacks callbacks);