#pragma once

#include <Arduino.h>
#include <vector>
#include <utility>
#include <api_types.h>

// A single HTTP request header as a (name, value) pair.
using HttpHeader = std::pair<String, String>;

// An ordered list of request headers. This is the single source of truth for
// the headers sent to each TRMNL API endpoint, shared by both transports:
//   - applyHeaders()  -> HTTPClient    (Wi-Fi path, see http_client.h)
//   - formatHeaders() -> raw string    (TRMNL X modem / 5 GHz path)
using HttpHeaderList = std::vector<HttpHeader>;

// Canonical header sets for each request endpoint
HttpHeaderList buildDisplayHeaders(const ApiDisplayInputs &inputs);
HttpHeaderList buildSetupHeaders(const ApiSetupInputs &inputs);
HttpHeaderList buildLogHeaders(const ApiLogInputs &inputs);

// Format a header list as a newline-separated "Name: value" string with no
// trailing newline, as expected by Modem::httpGet() (TRMNL X 5 GHz path).
String formatHeaders(const HttpHeaderList &headers);

// Log for debugging
void logHeaders(const HttpHeaderList &headers);
