#pragma once

#include <WiFiType.h>
#include <functional>
#include "wifi-helpers.h"
#include "wifi-types.h"

WifiConnectionResult initiateConnectionAndWaitForOutcome(const WifiCredentials credentials);
wl_status_t waitForConnectResult(uint32_t timeout);
void disableWpa2Enterprise();

// Registers a callback that is invoked repeatedly while waiting for a WiFi
// connection to complete. Lets the caller poll hardware (e.g. the button) during
// the otherwise blocking connect attempt. Pass nullptr to clear.
void setConnectTickCallback(std::function<void()> cb);
