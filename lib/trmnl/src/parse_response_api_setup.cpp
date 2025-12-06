#include <ArduinoJson.h>
#include "api_response_parsing.h"
#include <trmnl_log.h>

ApiSetupResponse parseResponse_apiSetup(String &payload)
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error)
  {
    Log_error("JSON deserialization error.");
    return {.outcome = ApiSetupOutcome::DeserializationError};
  }

  ApiSetupResponse response;
  response.status = doc["status"].as<int>();
  response.message = doc["message"] | "";

  if (response.status != 200)
  {
    Log_info("status FAIL.");
    response.outcome = ApiSetupOutcome::StatusError;
    return response;
  }

  response.outcome = ApiSetupOutcome::Ok;
  response.api_key = doc["api_key"] | "";
  response.friendly_id = doc["friendly_id"] | "";
  response.image_url = doc["image_url"] | "";
  return response;
}