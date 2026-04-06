#include <api-client/display.h>
#include <HTTPClient.h>
#include <trmnl_log.h>
#include <WiFiClientSecure.h>
#include <config.h>
#include <api_response_parsing.h>
#include <http_client.h>
#include <DEV_Config.h>
#ifdef SENSOR_SDA
extern RTC_DATA_ATTR int lastCO2, lastSCDTemp, lastTemp, lastSCDHumid, lastHumid, lastPressure, lastType, lastTime;
const char *szDevices[] = {"None", "AHT20", "BMP180", "BME280", "BMP388", "SHT3X", "HDC1080", "HTS221", "MCP9808","BME68x","SHTC3"};
const char *szMakers[] = {"None", "ASAIR", "Bosch", "Bosch", "Bosch", "Sensirion", "TI", "STMicro","MicroChip","Bosch","Sensirion"};
#endif // SENSOR_SDA

void addHeaders(HTTPClient &https, ApiDisplayInputs &inputs)
{
  Log_info("Added headers:\n\r"
           "ID: %s\n\r"
           "Special function: %d\n\r"
           "Update-Source: %s\n\r"
           "Access-Token: %s\n\r"
           "Refresh_Rate: %s\n\r"
           "Battery-Voltage: %s\n\r"
           "FW-Version: %s\r\n"
           "Model: %s\r\n"
           "RSSI: %s\r\n"
           "temperature-profile:true\r\n",
           inputs.macAddress.c_str(),
           inputs.specialFunction,
           inputs.updateSource.c_str(),
           inputs.apiKey.c_str(),
           String(inputs.refreshRate).c_str(),
           String(inputs.batteryVoltage).c_str(),
           inputs.firmwareVersion.c_str(),
           inputs.model.c_str(),
           String(inputs.rssi));

  https.addHeader("ID", inputs.macAddress);
  https.addHeader("Content-Type", "application/json");
  https.addHeader("Update-Source", inputs.updateSource);
  https.addHeader("Access-Token", inputs.apiKey);
  https.addHeader("Refresh-Rate", String(inputs.refreshRate));
  https.addHeader("Battery-Voltage", String(inputs.batteryVoltage));
  https.addHeader("FW-Version", inputs.firmwareVersion);
  https.addHeader("Model", String(inputs.model));
  https.addHeader("RSSI", String(inputs.rssi));
  https.addHeader("temperature-profile", "true");
  https.addHeader("Width", String(inputs.displayWidth));
  https.addHeader("Height", String(inputs.displayHeight));
#ifdef SENSOR_SDA
  char *szTemp, szPart[128];
  szTemp = (char *)malloc(1024); // make sure we have enough space, but don't use the stack because it's small
  if (lastCO2 != 0) { // valid data from SCD4x for CO2, Temperature and Humidity
    // create the multi-value string to pass as a HTTP header
    sprintf(szTemp, "make=Sensirion;model=SCD41;kind=carbon_dioxide;value=%d;unit=parts_per_million;created_at=%d,make=Sensirion;model=SCD41;kind=temperature;value=%f;unit=celsius;created_at=%d,make=Sensirion;model=SCD41;kind=humidity;value=%d;unit=percent;created_at=%d", lastCO2, lastTime, (float)lastSCDTemp / 10.0f, lastTime, lastSCDHumid, lastTime);
    Log_info("%s [%d] Adding SCD41 data to api request: CO2: %d, Temp: %d.%dC, Humidity: %d%%", __FILE__, __LINE__, lastCO2, lastSCDTemp/10, lastSCDTemp % 10, lastSCDHumid);
  }
  if (lastType >= 0 && lastTemp != 0) { // we have data from another bb_temperature supported sensor too; add it
    if (lastCO2 != 0) {
      strcat(szTemp, ","); // separate from CO2 data
    } else {
      szTemp[0] = 0;
    }
    Log_info("%s [%d] Adding bb_temperature data to api request: pressure: %d, Temp: %d.%dC, Humidity: %d%%", __FILE__, __LINE__, lastPressure, lastTemp/10, lastTemp % 10, lastHumid);
    sprintf(szPart, "make=%s;model=%s;kind=temperature;value=%f;unit=celsius;created_at=%d",szMakers[lastType], szDevices[lastType], (float)lastTemp / 10.0f, lastTime);
    strcat(szTemp, szPart);
    if (lastHumid > 0) { // add humidity
      sprintf(szPart, ",make=%s;model=%s;kind=humidity;value=%d;unit=percent;created_at=%d",szMakers[lastType], szDevices[lastType], lastHumid, lastTime);
      strcat(szTemp, szPart);
    }
    if (lastPressure > 0) {
      sprintf(szPart, ",make=%s;model=%s;kind=pressure;value=%d;unit=hectopascal;created_at=%d",szMakers[lastType], szDevices[lastType], lastPressure, lastTime);
      strcat(szTemp, szPart);
    }
  }
  if (lastCO2 != 0 || lastType >= 0) {
    https.addHeader("SENSORS", szTemp);
  } else {
    Log_info("%s [%d] Sensor data not available", __FILE__, __LINE__);
  }
  free(szTemp);
#endif // SENSOR_SDA

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