#include <trmnl_log.h>
#include <ArduinoLog.h>
#include <cstdarg>
#include <cstdio>

class AppLogger : public Logger {
private:
    static const size_t BUFFER_SIZE = 1024;

public:
    void verbose(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        char buffer[BUFFER_SIZE];
        vsnprintf(buffer, BUFFER_SIZE, format, args);
        Log.verbose(buffer);
        va_end(args);
    }
    
    void info(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        char buffer[BUFFER_SIZE];
        vsnprintf(buffer, BUFFER_SIZE, format, args);
        Log.info(buffer);
        va_end(args);
    }
    
    void error(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        char buffer[BUFFER_SIZE];
        vsnprintf(buffer, BUFFER_SIZE, format, args);
        Log.error(buffer);
        va_end(args);
    }
    
    void fatal(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        char buffer[BUFFER_SIZE];
        vsnprintf(buffer, BUFFER_SIZE, format, args);
        Log.fatal(buffer);
        va_end(args);
    }
};

Logger* g_logger = new AppLogger();