
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
        .verbose_logging = "",
        .keep_wifi_on = false,
        .full_refresh_every = DEFAULT_FULL_REFRESH_EVERY,
        .refresh_rate_ms = 0,
        .live_stock = false,
        .image_data = "",
        .knob_overlap = 0, .knob_win_rows = 0, .knob_pre = 60};
  }
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

  int full_refresh_every = doc["full_refresh_every"] | DEFAULT_FULL_REFRESH_EVERY;

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
      .verbose_logging = doc["verbose_logging"] | "",
      .keep_wifi_on = doc["keep_wifi_on"] | false,
      .full_refresh_every = full_refresh_every >= 1 ? full_refresh_every : DEFAULT_FULL_REFRESH_EVERY,
      .refresh_rate_ms = doc["refresh_rate_ms"] | 0u,
      .live_stock = doc["live_stock"] | false,
      .image_data = doc["image_data"] | "",
      .knob_overlap = doc["live_knobs"]["overlap"] | 0,
      .knob_win_rows = doc["live_knobs"]["win_rows"] | 0,
      .knob_pre = doc["live_knobs"]["pre"] | 60};
}
