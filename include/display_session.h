#pragma once

/**
 * Display session: /api/display fetch, response handling, and image download.
 *
 * Extracted from bl.cpp (network + image orchestration). Business init, sleep,
 * captive portal, and touchbar handling remain in bl.cpp and call into this
 * module for the server refresh path.
 */

#include <Preferences.h>
#include <api_types.h>
#include <types.h>

/** Human-readable names for https_request_err_e (index matches enum). */
extern const char *szHTTPErrors[];

/**
 * Build ApiDisplayInputs from NVS + live device state.
 * Relies on bl.cpp globals: wakeup_reason, special_function, battery, etc.
 */
ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences);

/**
 * Ping /api/display, interpret response, download/show image if needed.
 * @return https_request_err_e result for bl_init to branch on
 */
https_request_err_e downloadAndShow();

/**
 * Apply ApiDisplayResponse to session flags (status, filename, refresh rate,
 * special functions, reset/firmware). May load local images for SF_REWIND etc.
 */
https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);
