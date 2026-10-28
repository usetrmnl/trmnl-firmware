#include <ArduinoLog.h>
#include <WiFi.h>
#include <api-client/submit_log.h>
#include <app_logger.h>
#include <config.h>
#include <cstdarg>
#include <cstdio>
#include <globals.h>
#include <logging_parcers.h>
#include <misc/clock.h>
#include <serialize_log.h>
#include <special_function.h>
#include <stored_logs.h>
#include <string_utils.h>
#include <trmnl_log.h>
#include <wifi-helpers.h>

enum LogAction { LOG_ACTION_STORE, LOG_ACTION_SUBMIT, LOG_ACTION_SUBMIT_OR_STORE };

/**
 * @brief Function to submit a log string to the API
 * @param log_buffer pointer to the buffer that contains log note
 * @return bool true if successful, false if failed
 */
static bool submitLogString(const char *log_buffer) {
  String api_key = "";
  if (preferences.isKey(PREFERENCES_API_KEY)) {
    api_key = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log_info("%s key exists. Value - %s", PREFERENCES_API_KEY, api_key.c_str());
  } else {
    Log_info("%s key not exists.", PREFERENCES_API_KEY);
    return false;
  }

  LogApiInput input{api_key, log_buffer};
  return submitLogToApi(input, preferences.getString(PREFERENCES_API_URL, API_BASE_URL).c_str());
}

/**
 * @brief Function to store a log string locally
 * @param log_buffer pointer to the buffer that contains log note
 * @return bool true if successful, false if failed
 */
static bool storeLogString(const char *log_buffer) {
  LogStoreResult store_result = storedLogs.store_log(String(log_buffer));
  if (store_result.status != LogStoreResult::SUCCESS) {
    // Use the serial-only variant to avoid infinite recursion: Log_error here
    // would route back into log_impl → handle_store_submit → logWithAction →
    // storeLogString → store fails again → ...
    Log_error_serial("Failed to store log: %s", store_result.message);
    return false;
  }
  return true;
}

void submitStoredLogs(void) {
  if (WiFi.isConnected() == false) {
    Log_info("WiFi not connected; not submitting stored logs.");
    return;
  }
  String log = storedLogs.gather_stored_logs();

  String api_key = "";
  if (preferences.isKey(PREFERENCES_API_KEY)) {
    api_key = preferences.getString(PREFERENCES_API_KEY, PREFERENCES_API_KEY_DEFAULT);
    Log.info("%s [%d]: %s key exists. Value - %s\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY, api_key.c_str());
  } else {
    Log.error("%s [%d]: %s key not exists.\r\n", __FILE__, __LINE__, PREFERENCES_API_KEY);
  }

  bool submitLogToApiResult = false;
  if (log.length() > 0) {
    Log.info("%s [%d]: log string - %s\r\n", __FILE__, __LINE__, log.c_str());
    Log.info("%s [%d]: need to send the log\r\n", __FILE__, __LINE__);

    LogApiInput input{api_key, log.c_str()};
    submitLogToApiResult = submitLogToApi(input, preferences.getString(PREFERENCES_API_URL, API_BASE_URL).c_str());
  } else {
    Log.info("%s [%d]: no needed to send the log\r\n", __FILE__, __LINE__);
  }
  if (submitLogToApiResult == true) {
    storedLogs.clear_stored_logs();
  }
}

static DeviceStatusStamp getDeviceStatusStamp() {
  DeviceStatusStamp deviceStatus = {};

  deviceStatus.wifi_rssi_level = WiFi.RSSI();
  strncpy(deviceStatus.wifi_status, wifiStatusStr(WiFi.status()), sizeof(deviceStatus.wifi_status) - 1);
  deviceStatus.refresh_rate = refreshInterval.seconds(0);
  deviceStatus.time_since_last_sleep = time_since_sleep;
  snprintf(deviceStatus.current_fw_version, sizeof(deviceStatus.current_fw_version), "%s", FW_VERSION_STRING);
  parseSpecialFunctionToStr(deviceStatus.special_function, sizeof(deviceStatus.special_function), special_function);
  deviceStatus.battery_voltage = battery_voltage;
  parseWakeupReasonToStr(deviceStatus.wakeup_reason, sizeof(deviceStatus.wakeup_reason), esp_sleep_get_wakeup_cause());
  deviceStatus.free_heap_size = ESP.getFreeHeap();
  deviceStatus.max_alloc_size = ESP.getMaxAllocHeap();

  return deviceStatus;
}

static void logWithAction(LogAction action, LogLevel level, const char *message, time_t time, int line,
                          const char *file) {
  uint32_t log_id = preferences.getUInt(PREFERENCES_LOG_ID_KEY, 1);

  LogWithDetails input = {
      .deviceStatusStamp = getDeviceStatusStamp(),
      .timestamp = time,
      .codeline = line,
      .sourceFile = file,
      .logMessage = message,
      .logId = log_id,
      .filenameCurrent = preferences.getString(PREFERENCES_FILENAME_KEY, ""),
      .filenameNew = new_filename,
      .logRetry = log_retry,
      .retryAttempt = log_retry ? preferences.getInt(PREFERENCES_CONNECT_API_RETRY_COUNT) : 0,
      .level = level};

  String json_string = serialize_log(input);

  switch (action) {
  case LOG_ACTION_STORE:
    storeLogString(json_string.c_str());
    break;
  case LOG_ACTION_SUBMIT:
    submitLogString(json_string.c_str());
    break;
  case LOG_ACTION_SUBMIT_OR_STORE:
    if (!submitLogString(json_string.c_str())) {
      Log_info("Was unable to send log to API; saving locally for later.");
      storeLogString(json_string.c_str());
    }
    break;
  }

  preferences.putUInt(PREFERENCES_LOG_ID_KEY, ++log_id);
}

/// Logs at or above this severity will be sent to the server
static LogLevel store_submit_threshold = LogLevel::LOG_ERROR;

static void handle_store_submit(LogLevel level, const char *clean_message, const char *file, int line,
                                LogMode mode = LOG_STORE_ONLY) {
  if (level >= store_submit_threshold) {
    if (mode == LOG_STORE_ONLY) {
      logWithAction(LOG_ACTION_STORE, level, clean_message, systemClock().getTime(), line, file);
    } else {
      logWithAction(LOG_ACTION_SUBMIT_OR_STORE, level, clean_message, systemClock().getTime(), line, file);
    }
  }
}

void log_impl(LogLevel level, LogMode mode, const char *file, int line, const char *format, ...) {
  const int MAX_USER_MESSAGE = 512;

  va_list args;
  va_start(args, format);

    // Format user message with truncation (heap to avoid stack overflow in deep call chains)
  char *user_message = (char *)malloc(MAX_USER_MESSAGE);
  if (!user_message) {
    va_end(args);
    return;
  }
  format_message_truncated(user_message, MAX_USER_MESSAGE, format, args);
  va_end(args);

    // Measure exact length needed for serial buffer
  int serial_len = snprintf(nullptr, 0, "%s [%d]: %s", file, line, user_message) + 1;
  char *serial_buffer = (char *)malloc(serial_len);
  if (!serial_buffer) {
    free(user_message);
    return;
  }
  snprintf(serial_buffer, serial_len, "%s [%d]: %s", file, line, user_message);

// This mode is not handled correctly by underlying implementation,
// so shortcut it here
  if (mode == LOG_SERIAL_ONLY) {
    Serial.println(serial_buffer);
    return;
  }

  switch (level) {
  case LOG_VERBOSE:
    Log.verboseln(serial_buffer);
    break;
  case LOG_INFO:
    Log.infoln(serial_buffer);
    break;
  case LOG_WARN:
    Log.warningln(serial_buffer);
    break;
  case LOG_ERROR:
    Log.errorln(serial_buffer);
    break;
  case LOG_FATAL:
    Log.fatalln(serial_buffer);
    break;
  }
  free(serial_buffer);

  if (mode != LOG_SERIAL_ONLY) {
    handle_store_submit(level, user_message, file, line, mode);
  }
  free(user_message);
}
