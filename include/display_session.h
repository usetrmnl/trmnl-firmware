#pragma once

/**
 * Display session helpers for the /api/display refresh path.
 *
 * GME: handleApiDisplayResponse moved here. downloadAndShow / inputs loading
 * may still live in bl.cpp until follow-up PRs.
 */

#include <api_types.h>
#include <types.h>

/**
 * Apply ApiDisplayResponse to session flags (status, filename, refresh rate,
 * special functions, reset/firmware). May load local images for SF_REWIND /
 * SF_SEND_TO_ME.
 */
https_request_err_e handleApiDisplayResponse(ApiDisplayResponse &apiResponse);
