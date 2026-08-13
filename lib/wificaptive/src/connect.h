#pragma once

#include <WiFiType.h>

#include "wifi-helpers.h"
#include "wifi-types.h"

WifiConnectionResult initiateConnectionAndWaitForOutcome(const WifiCredentials credentials);
wl_status_t waitForConnectResult(uint32_t timeout);
void disableWpa2Enterprise();

// What went wrong on the last disconnect, phrased for the screen. An offline device cannot ask the
// server, so the wording is duplicated from Devices::WifiFailure in core rather than shared.
const char *lastWifiFailureDescription();
