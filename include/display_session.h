#pragma once

/**
 * Display session: /api/display inputs, response handling, and download/show.
 *
 * Combines G2 (inputs), G4 (response), and G5 (downloadAndShow).
 */

#include <Preferences.h>
#include <api_types.h>
#include <types.h>

ApiDisplayInputs loadApiDisplayInputs(Preferences &preferences);

https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);

/**
 * Fetch /api/display, apply response, download/show image when needed.
 * Called from bl_init; may recurse once for screen-wiper playlist advance.
 */
https_request_err_e downloadAndShow(void);
