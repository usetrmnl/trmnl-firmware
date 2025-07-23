#include <trmnl_log.h>
#include <cstdarg>
#include <cstdio>

#ifdef PIO_UNIT_TESTING

// Unified logging function for tests
void log_impl(LogLevel level, LogMode mode, const char* file, int line, const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    const char* level_str = (level == LOG_VERBOSE) ? "verbose" : 
                           (level == LOG_INFO) ? "info" : 
                           (level == LOG_ERROR) ? "error" : "fatal";
    
    printf("  [test log:%s] %s:%d ", level_str, file, line);
    vprintf(format, args);
    printf("\n");
    
    va_end(args);
}

#endif