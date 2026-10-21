#ifndef REQUEST_RESULT_H
#define REQUEST_RESULT_H

#include <display.h>
#include <types.h>

// What bl_init does with the result of downloadAndShow(). It draws and sleeps itself.

// Whether the result counts against the API retry count (lib/trmnl api_retry.h). An image host
// failure does not: the API answered, so there is no backoff.
bool requestFailed(https_request_err_e result);

// The error screen for the result, or NONE. weakSignal swaps the connection errors for WIFI_WEAK.
MSG requestErrorScreen(https_request_err_e result, bool weakSignal);

#endif
