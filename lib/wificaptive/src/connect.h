#pragma once

#include <WiFiType.h>
#include "wifi-types.h"

WifiConnectionResult initiateConnectionAndWaitForOutcome(const WifiCredentials credentials);
wl_status_t waitForConnectResult(uint32_t timeout, uint32_t pollMs = 100);
void disableWpa2Enterprise();

// T1 — tap fast path: associate directly using a cached channel/BSSID.
wl_status_t fastConnectAndWait(const WifiCredentials &credentials);
void saveFastConnectCache(const String &ssid);

// T2 — cached DHCP lease. g_fast_lease_applied is set when this wake associated
// using a cached lease; recoverFromStaleLease() re-runs DHCP if it was stale.
extern bool g_fast_lease_applied;
bool recoverFromStaleLease(const WifiCredentials &credentials);

