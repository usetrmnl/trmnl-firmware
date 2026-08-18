#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <api-client/display.h>
#include <api-client/request_headers.h>
#include <api_response_parsing.h>
#include <config.h>
#include <globals.h>
#include <http_client.h>
#include <inttypes.h>
#include <misc/sensor.h>
#include <trmnl_log.h>
extern RTC_DATA_ATTR int iPrevWakeTime; // total wake time of the last cycle (for statistics collection)
extern RTC_DATA_ATTR bool bUsedCachedImage; // if the last image displayed was read from cache (for statistics collection)
#ifdef SENSOR_SDA
extern int lastCO2, lastSCDTemp, lastTemp, lastSCDHumid, lastHumid, lastPressure, lastType, lastTime;
const char *szDevices[] = {"None", "AHT20", "BMP180", "BME280", "BMP388", "SHT3X", "HDC1080", "HTS221", "MCP9808","BME68x","SHTC3"};
const char *szMakers[] = {"None", "ASAIR", "Bosch", "Bosch", "Bosch", "Sensirion", "TI", "STMicro","MicroChip","Bosch","Sensirion"};
#endif // SENSOR_SDA

void addHeaders(HTTPClient &https, ApiDisplayInputs &inputs) {
  HttpHeaderList headers = buildDisplayHeaders(inputs);

  char *szTemp;
  if (sensor().buildSensorsHeader(&szTemp)) {
    headers.push_back({"SENSORS", szTemp});
    free(szTemp);
  } else {
    Log_info("%s [%d] Sensor data not available", __FILE__, __LINE__);
  }

  applyHeaders(https, headers);
  logHeaders(headers);
}

ApiDisplayResult fetchApiDisplay(ApiDisplayInputs &apiDisplayInputs) {

  return withHttp(
    apiDisplayInputs.baseUrl + "/api/display",
    [&apiDisplayInputs](HTTPClient *https, HttpError error) -> ApiDisplayResult {
      if (error == HttpError::HTTPCLIENT_WIFICLIENT_ERROR) {
        Log_error("Unable to create WiFiClient");
        return ApiDisplayResult{
            .error = https_request_err_e::HTTPS_UNABLE_TO_CONNECT,
            .response = {},
            .error_detail = "Unable to create WiFiClient",
        };
      }
      if (error == HttpError::HTTPCLIENT_HTTPCLIENT_ERROR) {
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
      if (httpCode == HTTP_CODE_PERMANENT_REDIRECT || httpCode == HTTP_CODE_TEMPORARY_REDIRECT) {
        String location = https->getLocation();
        https->end();
        String redirectUrl = (location.startsWith("http://") || location.startsWith("https://"))
                               ? location
                               : (apiDisplayInputs.baseUrl + location);
        https->begin(redirectUrl);
        Log_info("Redirected to: %s", redirectUrl.c_str());
        https->setTimeout(15000);
        https->setConnectTimeout(15000);
        addHeaders(*https, apiDisplayInputs);
        httpCode = https->GET();
      }

      if (httpCode < 0 || !(httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY ||
                            httpCode == HTTP_CODE_TOO_MANY_REQUESTS)) {
        Log_error("[HTTPS] GET... failed, error: %s", https->errorToString(httpCode).c_str());

        return ApiDisplayResult{
            .error = https_request_err_e::HTTPS_RESPONSE_CODE_INVALID,
            .response = {},
            .error_detail =
              "HTTP Client failed with error: " + https->errorToString(httpCode) + "(" + String(httpCode) + ")"};
      }

        // HTTP header has been send and Server response header has been handled
      Log_info("GET... code: %d", httpCode);

      String payload = https->getString();
      size_t size = https->getSize();
      Log_info("Content size: %d", size);
      Log_info("Free heap size: %" PRIu32, ESP.getMaxAllocHeap());
      Log_info("Payload - %s", payload.c_str());

      auto apiResponse = parseResponse_apiDisplay(payload);

      if (apiResponse.outcome == ApiDisplayOutcome::DeserializationError) {
        return ApiDisplayResult{
            .error = https_request_err_e::HTTPS_JSON_PARSING_ERR,
            .response = {},
            .error_detail = "JSON parse failed with error: " + apiResponse.error_detail};
      } else {
        return ApiDisplayResult{
            .error = https_request_err_e::HTTPS_NO_ERR, .response = apiResponse, .error_detail = ""};
      }
    },
    /*resumable=*/true);
}
