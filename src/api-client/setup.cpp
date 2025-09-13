#include <api-client/setup.h>
#include <HTTPClient.h>
#include <trmnl_log.h>
#include <WiFiClientSecure.h>
#include <config.h>
#include <api_response_parsing.h>
#include <http_client.h>

void addSetupHeaders(HTTPClient &https, ApiSetupInputs &inputs)
{
  Log_info("Added headers:\n\r"
           "ID: %s\n\r"
           "FW-Version: %s\r\n",
           inputs.macAddress.c_str(),
           inputs.firmwareVersion.c_str());

  https.addHeader("ID", inputs.macAddress);
  https.addHeader("Content-Type", "application/json");
  https.addHeader("FW-Version", inputs.firmwareVersion);
  https.addHeader("Model",inputs.model);
}

ApiSetupResult fetchApiSetup(ApiSetupInputs &apiSetupInputs)
{
  return withHttp(
      apiSetupInputs.baseUrl + "/api/setup/",
      [&apiSetupInputs](HTTPClient *https, HttpError error) -> ApiSetupResult
      {
        if (error == HttpError::HTTPCLIENT_WIFICLIENT_ERROR)
        {
          Log_error("Unable to create WiFiClient");
          return ApiSetupResult{
              .error = https_request_err_e::HTTPS_UNABLE_TO_CONNECT,
              .response = {},
              .error_detail = "Unable to create WiFiClient",
          };
        }
        if (error == HttpError::HTTPCLIENT_HTTPCLIENT_ERROR)
        {
          Log_error("Unable to create HTTPClient");
          return ApiSetupResult{
              .error = https_request_err_e::HTTPS_UNABLE_TO_CONNECT,
              .response = {},
              .error_detail = "Unable to create HTTPClient",
          };
        }

        https->setTimeout(15000);
        https->setConnectTimeout(15000);

        addSetupHeaders(*https, apiSetupInputs);

        delay(5);

        int httpCode = https->GET();

        if (httpCode < 0)
        {
          Log_error("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());

          return ApiSetupResult{
              .error = https_request_err_e::HTTPS_RESPONSE_CODE_INVALID,
              .response = {},
              .error_detail = "HTTP Client failed with error: " + https->errorToString(httpCode) +
                              "(" + String(httpCode) + ")"};
        }

        // HTTP header has been send and Server response header has been handled
        Log_info("GET... code: %d", httpCode);

        if (httpCode == HTTP_CODE_OK)
        {
          String payload = https->getString();
          size_t size = https->getSize();
          Log_info("Content size: %d", size);
          Log_info("Payload - %s", payload.c_str());

          auto apiResponse = parseResponse_apiSetup(payload);

          if (apiResponse.outcome == ApiSetupOutcome::DeserializationError)
          {
            return ApiSetupResult{
                .error = https_request_err_e::HTTPS_JSON_PARSING_ERR,
                .response = {},
                .error_detail = "JSON deserialization error"};
          }
          else
          {
            return ApiSetupResult{
                .error = https_request_err_e::HTTPS_NO_ERR,
                .response = apiResponse,
                .error_detail = ""};
          }
        }
        else
        {
          // For non-200 responses, we still return the response for the caller to handle
          // This allows handling of 404 (MAC not registered) and other status codes
          return ApiSetupResult{
              .error = https_request_err_e::HTTPS_NO_ERR,
              .response = {.outcome = ApiSetupOutcome::StatusError, .status = (uint16_t)httpCode},
              .error_detail = ""};
        }
      });
}