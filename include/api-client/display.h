#include <types.h>
#include <api_types.h>
#include <HTTPClient.h>

struct ApiDisplayResult
{
  https_request_err_e error;
  ApiDisplayResponse response;
  String error_detail;
};

void addHeaders(HTTPClient &https, ApiDisplayInputs &apiDisplayInputs);

ApiDisplayResult fetchApiDisplay(ApiDisplayInputs &apiDisplayInputs);

// T3: tap-path-only /api/display fetch over raw esp_tls with TLS session
// resumption. The session ticket is persisted in RTC RAM across deep sleep, so
// a tap wake does a 1-RTT resumed handshake (skips cert chain + signature
// verify) instead of a full one. Returns HTTPS_NO_ERR only on a clean 200;
// any other status, redirect, or transport error returns an error so the
// caller falls back to fetchApiDisplay() (the proven HTTPClient path).
ApiDisplayResult fetchApiDisplayResumable(ApiDisplayInputs &apiDisplayInputs);