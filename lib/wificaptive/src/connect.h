#pragma once

#include <WiFiType.h>
#include "wifi-types.h"

WifiConnectionResult initiateConnectionAndWaitForOutcome(const WifiCredentials credentials);
wl_status_t waitForConnectResult(uint32_t timeout);