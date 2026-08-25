#pragma once

/**
 * Display session helpers for the /api/display refresh path.
 *
 * GME step: loadApiDisplayInputs only. downloadAndShow / response handling
 * remain in bl.cpp until follow-up PRs.
 */

#include <Preferences.h>
#include <api_types.h>

/**
 * Build ApiDisplayInputs from NVS + live device state (globals, Wi-Fi, power,
 * battery). Uses the pre-WiFi vBatt snapshot when set by bl_init.
 */
ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences);
