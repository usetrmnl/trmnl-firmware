#pragma once

#include <HTTPClient.h>
#include <api-client/request_headers.h>
#include <api_types.h>
#include <types.h>

struct ApiDisplayResult {
  https_request_err_e error;
  ApiDisplayResponse response;
  String error_detail;
};

// Display headers plus the optional SENSORS header.
HttpHeaderList buildDisplayRequestHeaders(ApiDisplayInputs &apiDisplayInputs);

// GET /api/display with retries over Wi-Fi or the TRMNL X modem; parses the JSON body.
ApiDisplayResult fetchApiDisplay(ApiDisplayInputs &apiDisplayInputs);