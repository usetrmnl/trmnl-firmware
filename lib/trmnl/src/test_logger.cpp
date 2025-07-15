#include <trmnl_log.h>
#include <cstdarg>
#include <cstdio>

#ifdef PIO_UNIT_TESTING
class TestLogger : public Logger {
private:
    void log_with_level(const char* level, const char* format, va_list args) {
        printf("  [test log:%s] ", level);
        vprintf(format, args);
        printf("\n");
    }

public:
    void verbose(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        log_with_level("verbose", format, args);
        va_end(args);
    }
    
    void info(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        log_with_level("info", format, args);
        va_end(args);
    }
    
    void error(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        log_with_level("error", format, args);
        va_end(args);
    }
    
    void fatal(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        log_with_level("fatal", format, args);
        va_end(args);
    }
};

Logger* g_logger = new TestLogger();
#endif