#include <api-client/display.h>
#include <api-client/request_headers.h>
#include <api_response_parsing.h>
#include <config.h>
#include <globals.h>
#include <inttypes.h>
#include <misc/sensor.h>
#include <services/http_retry_request.h>
#include <trmnl_log.h>
extern RTC_DATA_ATTR int iPrevWakeTime; // total wake time of the last cycle (for statistics collection)
extern RTC_DATA_ATTR bool
  bUsedCachedImage; // if the last image displayed was read from cache (for statistics collection)
#ifdef SENSOR_SDA
extern int lastCO2, lastSCDTemp, lastTemp, lastSCDHumid, lastHumid, lastPressure, lastType, lastTime;
const char *szDevices[] = {"None",    "AHT20",  "BMP180",  "BME280", "BMP388", "SHT3X",
                           "HDC1080", "HTS221", "MCP9808", "BME68x", "SHTC3"};
const char *szMakers[] = {"None", "ASAIR",   "Bosch",     "Bosch", "Bosch",    "Sensirion",
                          "TI",   "STMicro", "MicroChip", "Bosch", "Sensirion"};
#endif // SENSOR_SDA

HttpHeaderList buildDisplayRequestHeaders(ApiDisplayInputs &inputs) {
  HttpHeaderList headers = buildDisplayHeaders(inputs);

  char *szTemp;
  if (sensor().buildSensorsHeader(&szTemp)) {
    headers.push_back({"SENSORS", szTemp});
    free(szTemp);
  } else {
    Log_info("%s [%d] Sensor data not available", __FILE__, __LINE__);
  }

  return headers;
}

ApiDisplayResult fetchApiDisplay(ApiDisplayInputs &apiDisplayInputs) {
  HttpRetryRequestConfig config;
  config.url = apiDisplayInputs.baseUrl + "/api/display";
  config.apiBaseUrl = apiDisplayInputs.baseUrl;
  config.headers = buildDisplayRequestHeaders(apiDisplayInputs);

  // Retries, redirects and transport selection (Wi-Fi vs. TRMNL X modem) live in HttpRetryRequest.
  HttpRetryRequest request(config);
  https_request_err_e err = request.execute();
  if (err != HTTPS_NO_ERR) {
    return ApiDisplayResult{.error = err, .response = {}, .error_detail = request.errorDetail()};
  }

  String payload = request.bodyAsString();
  Log_info("Content size: %" PRIu32, request.bodySize());
  Log_info("Free heap size: %" PRIu32, ESP.getMaxAllocHeap());
  Log_info("Payload - %s", payload.c_str());

  auto apiResponse = parseResponse_apiDisplay(payload);

  if (apiResponse.outcome == ApiDisplayOutcome::DeserializationError) {
    return ApiDisplayResult{
        .error = https_request_err_e::HTTPS_JSON_PARSING_ERR,
        .response = {},
        .error_detail = "JSON parse failed with error: " + apiResponse.error_detail};
  }
  return ApiDisplayResult{.error = https_request_err_e::HTTPS_NO_ERR, .response = apiResponse, .error_detail = ""};
}
