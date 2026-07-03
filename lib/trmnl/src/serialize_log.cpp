#include "serialize_log.h"
#include <ArduinoJson.h>
#include <trmnl_log.h>

String serialize_log(const LogWithDetails &input) {
  JsonDocument json_log;

  json_log["created_at"] = input.timestamp;
  json_log["id"] = input.logId;
  json_log["message"] = input.logMessage;
  json_log["source_line"] = input.codeline;
  json_log["source_path"] = input.sourceFile;

  json_log["wifi_signal"] = input.deviceStatusStamp.wifi_rssi_level;
  json_log["wifi_status"] = input.deviceStatusStamp.wifi_status;
  json_log["refresh_rate"] = input.deviceStatusStamp.refresh_rate;
  json_log["sleep_duration"] = input.deviceStatusStamp.time_since_last_sleep;
  json_log["firmware_version"] = input.deviceStatusStamp.current_fw_version;
  json_log["special_function"] = input.deviceStatusStamp.special_function;
  json_log["battery_voltage"] = input.deviceStatusStamp.battery_voltage;
  json_log["wake_reason"] = input.deviceStatusStamp.wakeup_reason;
  json_log["free_heap_size"] = input.deviceStatusStamp.free_heap_size;
  json_log["max_alloc_size"] = input.deviceStatusStamp.max_alloc_size;

  switch (input.level) {
  case LOG_VERBOSE:
    json_log["level"] = "debug";
    break;
  case LOG_INFO:
    json_log["level"] = "info";
    break;
  case LOG_WARN:
    json_log["level"] = "warn";
    break;
  case LOG_ERROR:
    json_log["level"] = "error";
    break;
  case LOG_FATAL:
    json_log["level"] = "fatal";
    break;
  default:
    json_log["level"] = "info";
    break;
  }

  if (input.logRetry) {
    json_log["retry"] = input.retryAttempt;
  }

  String json_string;
  serializeJson(json_log, json_string);
  return json_string;
}