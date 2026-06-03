#pragma once

#include <types.h>
#include <api_types.h>
#include <HTTPClient.h>

struct ApiSetupResult
{
  https_request_err_e error;
  ApiSetupResponse response;
  String error_detail;
};

ApiSetupResult fetchApiSetup(ApiSetupInputs &apiSetupInputs);