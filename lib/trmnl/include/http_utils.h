#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Transport-agnostic pieces of HTTP request policy shared by the Wi-Fi and
// modem paths of HttpRetryRequest (src/services/http_retry_request.cpp).
// Kept free of HTTPClient so they compile and test natively.

// "scheme://host[:port]" of a URL, or the URL unchanged if it has no scheme
// or no path after the host.
String httpOriginOf(const String &url);

// Absolute Location values are returned as-is; relative ones are appended to
// `origin`.
String resolveRedirectLocation(const String &location, const String &origin);

// True when `url` starts with `base`. Used to decide whether auth headers may
// be sent to a given hop. An empty base never matches.
bool isSameOrigin(const String &url, const String &base);

// Content type from the first bytes of a body: "image/bmp", "image/png",
// "image/jpeg", or nullptr when unrecognised.
const char *sniffImageContentType(const uint8_t *data, size_t len);

// True if a request that ended with this code is worth retrying right away:
// transport failures (code <= 0, including HTTPClient's negative errors and the
// modem's 0 / -1), 408, and any 5xx. Other 4xx are deterministic and are not.
bool shouldFastRetryCode(int code);