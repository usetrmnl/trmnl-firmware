#pragma once

/**
 * Display session helpers for the /api/display refresh path.
 *
 * Combines G2 (loadApiDisplayInputs) and G4 (handleApiDisplayResponse).
 * downloadAndShow may still live in bl.cpp until this branch completes.
 */

#include <Preferences.h>
#include <api_types.h>
#include <types.h>

/**
 * Build ApiDisplayInputs from NVS + live device state (globals, Wi-Fi, power,
 * battery). Uses the pre-WiFi vBatt snapshot when set by bl_init.
 */
ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences);

/**
 * Apply ApiDisplayResponse to session flags (status, filename, refresh rate,
 * special functions, reset/firmware). May load local images for SF_REWIND /
 * SF_SEND_TO_ME.
 */
https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);
