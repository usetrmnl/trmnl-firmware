
#include <ArduinoJson.h>
#include <special_function.h>
#include <trmnl_log.h>

#include "api_response_parsing.h"

ApiDisplayResponse parseResponse_apiDisplay(String &payload) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Log_error("JSON deserialization error.");
    return ApiDisplayResponse{
        .outcome = ApiDisplayOutcome::DeserializationError,
        .error_detail = error.c_str(),
        .status = 0,
        .image_url = "",
        .image_url_timeout = 0,
        .filename = "",
        .update_firmware = false,
        .maximum_compatibility = false,
        .firmware_url = "",
        .refresh_rate = 0,
        .temp_profile = 0,
        .reset_firmware = false,
        .special_function = SF_NONE,
        .action = "",
        .touchbar_mode = "",
        .v1_action = V1_ACTION_FULL,
        .frame_id = "",
        .full_url = "",
        .regions_url = "",
        .full_mode = V1_FULL_FULL,
        .sleep_mode = V1_SLEEP_DEEP,
        .ota_wait = false};
  }
  // protocol v1: missing/unknown values fall back to the stock behaviour (full refresh via image_url)
  String v1ActionStr = doc["action"] | "";
  V1Action v1Action = V1_ACTION_FULL;
  if (v1ActionStr == "none")
    v1Action = V1_ACTION_NONE;
  else if (v1ActionStr == "partial")
    v1Action = V1_ACTION_PARTIAL;
  String fullModeStr = doc["full_mode"] | "";
  String sleepModeStr = doc["sleep_mode"] | "";
  String imageUrl = doc["image_url"] | "";
  String fullUrl = doc["full_url"] | "";
  String frameId = doc["frame_id"] | "";
  String fileName = doc["filename"] | "";
  String special_function_str = doc["special_function"];
  // Convert the temperature profile ("default", "a", "b", "c")
  // into an integer value (0,1,2,3)
  String tp = doc["temperature_profile"];
  uint32_t u32TP = 0; // default
  if (tp == "a")
    u32TP = 1;
  else if (tp == "b")
    u32TP = 2;
//     else if (tp == "c") u32TP = 3;

  return ApiDisplayResponse{
      .outcome = ApiDisplayOutcome::Ok,
      .error_detail = "",
      .status = doc["status"],
      .image_url = doc["image_url"] | "",
      .image_url_timeout = doc["image_url_timeout"],
      .filename = doc["filename"] | "",
      .update_firmware = doc["update_firmware"],
      // server doesn't return this flag if device.firmware_version <= 1.6.2
      .maximum_compatibility = doc["maximum_compatibility"] | false,
      .firmware_url = doc["firmware_url"] | "",
      .refresh_rate = doc["refresh_rate"],
      .temp_profile = u32TP,
      .reset_firmware = doc["reset_firmware"],
      .special_function = parseSpecialFunction(special_function_str),
      .action = doc["action"] | "",
      .touchbar_mode = doc["touchbar_mode"] | "",
      .v1_action = v1Action,
      .frame_id = frameId.length() ? frameId : fileName,
      .full_url = fullUrl.length() ? fullUrl : imageUrl,
      .regions_url = doc["regions_url"] | "",
      .full_mode = (fullModeStr == "fast") ? V1_FULL_FAST : V1_FULL_FULL,
      .sleep_mode = (sleepModeStr == "light") ? V1_SLEEP_LIGHT : V1_SLEEP_DEEP,
      .ota_wait = doc["ota_wait"] | false};
}
