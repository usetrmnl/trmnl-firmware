#pragma once

/**
 * Boot-time Wi-Fi session: hostname, optional X modem prep, connect with
 * saved credentials, or captive portal.
 *
 * Call order from bl_init:
 *   1. wifiSessionInit()     — early (before SF_ADD_WIFI / portal use)
 *   2. wifiSessionConnect()  — after display/filesystem/battery init
 *
 * On connection failure, wifiSessionConnect shows WIFI_FAILED and deep-sleeps
 * via wifiErrorDeepSleep (does not return on that path).
 */

/** Set captive-portal hostname from preferences / friendly ID. */
void wifiSessionInit(void);

/**
 * Prepare modem (X), set STA mode, then auto-connect or start captive portal.
 * Blocks until connected or sleep is entered on failure.
 */
void wifiSessionConnect(void);
