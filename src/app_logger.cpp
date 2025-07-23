#include <trmnl_log.h>
#include <ArduinoLog.h>
#include <cstdarg>
#include <cstdio>
#include <bl.h>
#include <stored_logs.h>
#include <string_utils.h>

extern StoredLogs storedLogs;

/// Logs at or above this severity will be sent to the server
static LogLevel store_submit_threshold = LogLevel::LOG_ERROR;

static void handle_store_submit(LogLevel level, const char *clean_message, const char* file, int line, LogMode mode = LOG_STORE_ONLY)
{
    if (level >= store_submit_threshold)
    {
        if (mode == LOG_STORE_ONLY) {
            logWithAction(LOG_ACTION_STORE, clean_message, getTime(), line, file);
        } else {
            logWithAction(LOG_ACTION_SUBMIT_OR_STORE, clean_message, getTime(), line, file);
        }
    }
}

void log_impl(LogLevel level, LogMode mode, const char* file, int line, const char* format, ...) {
    const int MAX_USER_MESSAGE = 512;
    
    va_list args;
    va_start(args, format);
    
    // Format user message with truncation
    char* user_message = (char*)alloca(MAX_USER_MESSAGE);
    format_message_truncated(user_message, MAX_USER_MESSAGE, format, args);
    va_end(args);
    
    // Measure exact length needed for serial buffer
    int serial_len = snprintf(nullptr, 0, "%s [%d]: %s", file, line, user_message) + 1;
    char* serial_buffer = (char*)alloca(serial_len);
    snprintf(serial_buffer, serial_len, "%s [%d]: %s", file, line, user_message);
    
    switch (level) {
    case LOG_VERBOSE:
        Log.verboseln(serial_buffer);
        break;
    case LOG_INFO:
        Log.infoln(serial_buffer);
        break;
    case LOG_ERROR:
        Log.errorln(serial_buffer);
        break;
    case LOG_FATAL:
        Log.fatalln(serial_buffer);
        break;
    }

    if (mode != LOG_SERIAL_ONLY)
    {
        handle_store_submit(level, user_message, file, line, mode);
    }
}
