#include <ArduinoJson.h>
#include "serialize_log.h"
#include <trmnl_log.h>

String serialize_log(const LogWithDetails &input)
{
  JsonDocument json_log;

  json_log["creation_timestamp"] = input.timestamp;

  json_log["device_status_stamp"]["wifi_rssi_level"] = input.deviceStatusStamp.wifi_rssi_level;
  json_log["device_status_stamp"]["wifi_status"] = input.deviceStatusStamp.wifi_status;
  json_log["device_status_stamp"]["refresh_rate"] = input.deviceStatusStamp.refresh_rate;
  json_log["device_status_stamp"]["time_since_last_sleep_start"] = input.deviceStatusStamp.time_since_last_sleep;
  json_log["device_status_stamp"]["current_fw_version"] = input.deviceStatusStamp.current_fw_version;
  json_log["device_status_stamp"]["special_function"] = input.deviceStatusStamp.special_function;
  json_log["device_status_stamp"]["battery_voltage"] = input.deviceStatusStamp.battery_voltage;
  json_log["device_status_stamp"]["wakeup_reason"] = input.deviceStatusStamp.wakeup_reason;
  json_log["device_status_stamp"]["free_heap_size"] = input.deviceStatusStamp.free_heap_size;
  json_log["device_status_stamp"]["max_alloc_size"] = input.deviceStatusStamp.max_alloc_size;

  json_log["log_id"] = input.logId;
  json_log["log_message"] = input.logMessage;
  json_log["log_codeline"] = input.codeline;
  json_log["log_sourcefile"] = input.sourceFile;

  json_log["additional_info"]["filename_current"] = input.filenameCurrent;
  json_log["additional_info"]["filename_new"] = input.filenameNew;

  if (input.logRetry)
  {
    json_log["additional_info"]["retry_attempt"] = input.retryAttempt;
  }

  String json_string;
  serializeJson(json_log, json_string);
  return json_string;
}