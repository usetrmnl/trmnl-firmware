#pragma once

// The /api/display refresh path: ask the server what to show, then draw it.

#include <api-client/display.h>
#include <types.h>

/**
 * Apply ApiDisplayResponse to session flags (status, filename, refresh rate,
 * special functions, reset/firmware). May load local images for SF_REWIND /
 * SF_SEND_TO_ME.
 */
https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);

// Asks /api/display what to show and shows it: a screen wipe, the cached image, or a new image
// downloaded, drawn and cached. Returns the request result bl_init acts on.
https_request_err_e downloadAndShow(float batteryVoltage);
