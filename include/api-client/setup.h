#pragma once

#include <HTTPClient.h>
#include <api_types.h>
#include <types.h>

struct ApiSetupResult {
  https_request_err_e error;
  ApiSetupResponse response;
  String error_detail;
};

ApiSetupResult fetchApiSetup(ApiSetupInputs &apiSetupInputs);