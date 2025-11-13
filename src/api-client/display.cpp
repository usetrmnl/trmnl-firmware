#include <api-client/display.h>
#include <HTTPClient.h>
#include <trmnl_log.h>
#include <WiFiClientSecure.h>
#include <config.h>
#include <api_response_parsing.h>
#include <http_client.h>

void addHeaders(HTTPClient &https, ApiDisplayInputs &inputs)
{
  Log_info("Added headers:\n\r"
           "ID: %s\n\r"
           "Special function: %d\n\r"
           "Access-Token: %s\n\r"
           "Refresh_Rate: %s\n\r"
           "Battery-Voltage: %s\n\r"
           "FW-Version: %s\r\n"
           "Model: %s\r\n"
           "RSSI: %s\r\n",
           inputs.macAddress.c_str(),
           inputs.specialFunction,
           inputs.apiKey.c_str(),
           String(inputs.refreshRate).c_str(),
           String(inputs.batteryVoltage).c_str(),
           inputs.firmwareVersion.c_str(),
           inputs.model.c_str(),
           String(inputs.rssi));

  https.addHeader("ID", inputs.macAddress);
  https.addHeader("Content-Type", "application/json");
  https.addHeader("Access-Token", inputs.apiKey);
  https.addHeader("Refresh-Rate", String(inputs.refreshRate));
  https.addHeader("Battery-Voltage", String(inputs.batteryVoltage));
  https.addHeader("FW-Version", inputs.firmwareVersion);
  https.addHeader("Model", String(inputs.model));
  https.addHeader("RSSI", String(inputs.rssi));
  https.addHeader("Width", String(inputs.displayWidth));
  https.addHeader("Height", String(inputs.displayHeight));

  if (inputs.specialFunction != SF_NONE)
  {
    Log_info("Add special function: true (%d)", inputs.specialFunction);
    https.addHeader("special_function", "true");
  }
}

ApiDisplayResult fetchApiDisplay(ApiDisplayInputs &apiDisplayInputs)
{

  return withHttp(
      apiDisplayInputs.baseUrl + "/api/display",
      [&apiDisplayInputs](HTTPClient *https, HttpError error) -> ApiDisplayResult
      {
        if (error == HttpError::HTTPCLIENT_WIFICLIENT_ERROR)
        {
          Log_error("Unable to create WiFiClient");
          return ApiDisplayResult{
              .error = https_request_err_e::HTTPS_UNABLE_TO_CONNECT,
              .response = {},
              .error_detail = "Unable to create WiFiClient",
          };
        }
        if (error == HttpError::HTTPCLIENT_HTTPCLIENT_ERROR)
        {
          Log_error("Unable to create HTTPClient");
          return ApiDisplayResult{
              .error = https_request_err_e::HTTPS_UNABLE_TO_CONNECT,
              .response = {},
              .error_detail = "Unable to create HTTPClient",
          };
        }

        https->setTimeout(15000);
        https->setConnectTimeout(15000);

        addHeaders(*https, apiDisplayInputs);

        delay(5);

        Log_info("Start location: %s", https->getLocation().c_str());
        int httpCode = https->GET();
        if(httpCode == HTTP_CODE_PERMANENT_REDIRECT ||httpCode == HTTP_CODE_TEMPORARY_REDIRECT){
              https->end();
              https->begin(API_BASE_URL + https->getLocation());
              Log_info("Redirected to: %s", https->getLocation().c_str());
              https->setTimeout(15000);
              https->setConnectTimeout(15000);
              addHeaders(*https, apiDisplayInputs);
              httpCode = https->GET();
            }

        if (httpCode < 0 ||
            !(httpCode == HTTP_CODE_OK ||
              httpCode == HTTP_CODE_MOVED_PERMANENTLY ||
              httpCode == HTTP_CODE_TOO_MANY_REQUESTS))
        {
          Log_error("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());

          return ApiDisplayResult{
              .error = https_request_err_e::HTTPS_RESPONSE_CODE_INVALID,
              .response = {},
              .error_detail = "HTTP Client failed with error: " + https->errorToString(httpCode) +
                              "(" + String(httpCode) + ")"};
        }

        // HTTP header has been send and Server response header has been handled
        Log_info("GET... code: %d", httpCode);

        String payload = https->getString();
        size_t size = https->getSize();
        Log_info("Content size: %d", size);
        Log_info("Free heap size: %d", ESP.getMaxAllocHeap());
        Log_info("Payload - %s", payload.c_str());

        auto apiResponse = parseResponse_apiDisplay(payload);

        if (apiResponse.outcome == ApiDisplayOutcome::DeserializationError)
        {
          return ApiDisplayResult{
              .error = https_request_err_e::HTTPS_JSON_PARSING_ERR,
              .response = {},
              .error_detail = "JSON parse failed with error: " +
                              apiResponse.error_detail};
        }
        else
        {
          return ApiDisplayResult{
              .error = https_request_err_e::HTTPS_NO_ERR,
              .response = apiResponse,
              .error_detail = ""};
        }
      });
}