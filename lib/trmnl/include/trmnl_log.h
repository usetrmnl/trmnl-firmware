
#pragma once

enum LogLevel {
    LOG_VERBOSE = 0,
    LOG_INFO = 1,
    LOG_ERROR = 2,
    LOG_FATAL = 3
};

enum LogMode
{
    LOG_SERIAL_ONLY,
    LOG_STORE_ONLY,
    LOG_SUBMIT_OR_STORE
};

void log_impl(LogLevel level, LogMode mode, const char* file, int line, const char* format, ...);

#define _LOG_IMPL(level, mode, format, ...) \
    log_impl(level, mode, __FILE__, __LINE__, format, ##__VA_ARGS__)

/**
 * Standard variants (serial + store locally for later submission)
 */
/** Log to serial and store locally for later submission */
#define Log_verbose(format, ...) _LOG_IMPL(LOG_VERBOSE, LOG_STORE_ONLY, format, ##__VA_ARGS__)
/** Log to serial and store locally for later submission */
#define Log_info(format, ...) _LOG_IMPL(LOG_INFO, LOG_STORE_ONLY, format, ##__VA_ARGS__)
/** Log to serial and store locally for later submission */
#define Log_error(format, ...) _LOG_IMPL(LOG_ERROR, LOG_STORE_ONLY, format, ##__VA_ARGS__)
/** Log to serial and store locally for later submission */
#define Log_fatal(format, ...) _LOG_IMPL(LOG_FATAL, LOG_STORE_ONLY, format, ##__VA_ARGS__)

/*
    Submit variants (serial + immediate submission / storage - use with caution due to HTTPClient stack overflow risk)
*/
/** Log to serial and attempt to submit to API, else store locally. */
#define Log_verbose_submit(format, ...) _LOG_IMPL(LOG_VERBOSE, LOG_SUBMIT_OR_STORE, format, ##__VA_ARGS__)
/** Log to serial and attempt to submit to API, else store locally. */
#define Log_info_submit(format, ...) _LOG_IMPL(LOG_INFO, LOG_SUBMIT_OR_STORE, format, ##__VA_ARGS__)
/** Log to serial and attempt to submit to API, else store locally. */
#define Log_error_submit(format, ...) _LOG_IMPL(LOG_ERROR, LOG_SUBMIT_OR_STORE, format, ##__VA_ARGS__)
/** Log to serial and attempt to submit to API, else store locally. */
#define Log_fatal_submit(format, ...) _LOG_IMPL(LOG_FATAL, LOG_SUBMIT_OR_STORE, format, ##__VA_ARGS__)

/*
    Serial-only variants (direct to serial, no storage, no submission)

    These are useful for early logging (before NVS or network is up), or
    from within logging code itself to avoid recursion.
*/
/** Log to serial only (no storage, no submission) */
#define Log_verbose_serial(format, ...) _LOG_IMPL(LOG_VERBOSE, LOG_SERIAL_ONLY, format, ##__VA_ARGS__)
/** Log to serial only (no storage, no submission) */
#define Log_info_serial(format, ...) _LOG_IMPL(LOG_INFO, LOG_SERIAL_ONLY, format, ##__VA_ARGS__)
/** Log to serial only (no storage, no submission) */
#define Log_error_serial(format, ...) _LOG_IMPL(LOG_ERROR, LOG_SERIAL_ONLY, format, ##__VA_ARGS__)
/** Log to serial only (no storage, no submission) */
#define Log_fatal_serial(format, ...) _LOG_IMPL(LOG_FATAL, LOG_SERIAL_ONLY, format, ##__VA_ARGS__)
