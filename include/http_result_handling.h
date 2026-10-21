#pragma once

#include <display.h>
#include <refresh_interval.h>
#include <stdint.h>
#include <types.h>

// What bl_init does with the result of downloadAndShow(): whether it counts as a failure, what that
// does to the stored failure count, and which error screen to draw. bl_init draws and sleeps.

enum class RequestFailureAction {
  ResetCount, // the request worked (or only the image host failed): count back to 1
  ShowNow,    // manual wake: show the error now, leave the count alone
  StayQuiet,  // timer wake: keep the current image and sleep on the retry interval
  ShowError,  // timer wake, MAX_QUIET_SLOW_RETRIES reached (as for WiFi failures): show the error
};

// Whether the result counts as a failed request. An image host failure doesn't: the API answered.
bool requestIsFailure(https_request_err_e result);

// failedWakes is the stored 1-based count before this wake's attempt.
RequestFailureAction requestFailureAction(https_request_err_e result, bool showErrorNow, uint8_t failedWakes);

// The error screen for the result, or NONE. weakSignal swaps the connection errors for WIFI_WEAK.
MSG requestErrorScreen(https_request_err_e result, bool weakSignal);
