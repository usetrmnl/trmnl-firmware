#pragma once

/**
 * Display session helpers for the /api/display refresh path.
 *
 * GME2 G2: handleApiDisplayResponse. downloadAndShow / inputs loading may
 * still live in bl.cpp until follow-up PRs.
 */

#include <api-client/display.h>
#include <types.h>

/**
 * Apply ApiDisplayResponse to session flags (status, filename, refresh rate,
 * special functions, reset/firmware). May load local images for SF_REWIND /
 * SF_SEND_TO_ME.
 */
https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);
