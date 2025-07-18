
#pragma once

class Logger {
public:
    virtual ~Logger() = default;
    virtual void verbose(const char* format, ...) = 0;
    virtual void info(const char* format, ...) = 0;
    virtual void error(const char* format, ...) = 0;
    virtual void fatal(const char* format, ...) = 0;
};

extern Logger* g_logger;

#define Log_verbose(format, ...) g_logger->verbose("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define Log_info(format, ...) g_logger->info("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define Log_error(format, ...) g_logger->error("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define Log_fatal(format, ...) g_logger->fatal("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)
