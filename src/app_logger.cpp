#include <ArduinoLog.h>
#include <bl.h>
#include <cstdarg>
#include <cstdio>
#include <debug_log_mode.h>
#include <globals.h>
#include <misc/clock.h>
#include <stored_logs.h>
#include <string_utils.h>
#include <trmnl_log.h>

/// Logs at or above this severity will be sent to the server.
///
/// Stays at LOG_ERROR until app_logger_begin() raises it, because verbose
/// capture needs the filesystem: routed to the ten NVS slots instead, a verbose
/// boot would evict the errors those slots exist to hold.
static LogLevel store_submit_threshold = LogLevel::LOG_ERROR;

void app_logger_begin(void) {
  store_submit_threshold =
      debug_log_threshold(preferences.getUInt(PREFERENCES_LOG_EXPIRES_AT_KEY, 0), systemClock().getTime());
}

bool app_logger_capturing(void) { return store_submit_threshold == LogLevel::LOG_VERBOSE; }

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
