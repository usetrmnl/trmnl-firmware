#pragma once

#include "wifi-types.h"
#include <WiFiType.h>

WifiConnectionResult initiateConnectionAndWaitForOutcome(const WifiCredentials credentials);
wl_status_t waitForConnectResult(uint32_t timeout);
void disableWpa2Enterprise();
