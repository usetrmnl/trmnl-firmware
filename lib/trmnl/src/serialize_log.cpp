#include "serialize_log.h"

#include <ArduinoJson.h>
#include <trmnl_log.h>

static const char *level_name(LogLevel level) {
  switch (level) {
  case LOG_VERBOSE:
    return "debug";
  case LOG_INFO:
    return "info";
  case LOG_WARN:
    return "warn";
  case LOG_ERROR:
    return "error";
  case LOG_FATAL:
    return "fatal";
  default:
    return "info";
  }
}

static void add_entry_fields(JsonDocument &json_log, const LogWithDetails &input) {
  json_log["created_at"] = input.timestamp;
  json_log["id"] = input.logId;
  json_log["message"] = input.logMessage;
  json_log["source_line"] = input.codeline;
  json_log["source_path"] = input.sourceFile;
}

static void add_stamp_fields(JsonDocument &json_log, const DeviceStatusStamp &stamp) {
  json_log["wifi_signal"] = stamp.wifi_rssi_level;
  json_log["wifi_status"] = stamp.wifi_status;
  json_log["refresh_rate"] = stamp.refresh_rate;
  json_log["sleep_duration"] = stamp.time_since_last_sleep;
  json_log["firmware_version"] = stamp.current_fw_version;
  json_log["special_function"] = stamp.special_function;
  json_log["battery_voltage"] = stamp.battery_voltage;
  json_log["wake_reason"] = stamp.wakeup_reason;
  json_log["free_heap_size"] = stamp.free_heap_size;
  json_log["max_alloc_size"] = stamp.max_alloc_size;
}

String serialize_log(const LogWithDetails &input) {
  JsonDocument json_log;

  add_entry_fields(json_log, input);
  add_stamp_fields(json_log, input.deviceStatusStamp);
  json_log["level"] = level_name(input.level);

  if (input.logRetry) {
    json_log["retry"] = input.retryAttempt;
  }

  String json_string;
  serializeJson(json_log, json_string);
  return json_string;
}

String serialize_log_lean(const LogWithDetails &input) {
  JsonDocument json_log;

  add_entry_fields(json_log, input);
  json_log["level"] = level_name(input.level);

  if (input.logRetry) {
    json_log["retry"] = input.retryAttempt;
  }

  String json_string;
  serializeJson(json_log, json_string);
  return json_string;
}

String serialize_device_stamp(const DeviceStatusStamp &stamp, time_t timestamp) {
  JsonDocument json_log;

  json_log["type"] = "stamp";
  json_log["created_at"] = timestamp;
  add_stamp_fields(json_log, stamp);

  String json_string;
  serializeJson(json_log, json_string);
  return json_string;
}
