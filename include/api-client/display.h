#include <HTTPClient.h>
#include <api_types.h>
#include <types.h>

struct ApiDisplayResult {
  https_request_err_e error;
  ApiDisplayResponse response;
  String error_detail;
};

void addHeaders(HTTPClient &https, ApiDisplayInputs &apiDisplayInputs);

ApiDisplayResult fetchApiDisplay(ApiDisplayInputs &apiDisplayInputs);