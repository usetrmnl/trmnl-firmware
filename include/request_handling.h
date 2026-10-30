#pragma once

#include <types.h>

//
// What bl_init() does with the result of one /api/display request and image download
//

// Shows the "image too large" screen for HTTPS_IMAGE_FILE_TOO_BIG
void showImageTooBigIfNeeded(https_request_err_e result);

// Counts failed wakes and schedules the retry backoff; on a quiet (timer wake) failure, sleeps
// right away instead of showing an error
void applyRetryPolicy(https_request_err_e result, bool showErrorsNow);

// Shows the friendly ID setup screen when the server says the device isn't claimed yet
void showFriendlyIdIfUnregistered(https_request_err_e result, bool bleSetup = false);

// Clears the credentials and restarts when the server asked for a reset
void resetIfRequested(https_request_err_e result);

// Downloads and installs new firmware when the server offered an update that is due; restarts on success
void updateFirmwareIfDue(void);

// Polls fast while no plugin is attached, and shows the error screen for a failed request
void showRequestResult(https_request_err_e result);
