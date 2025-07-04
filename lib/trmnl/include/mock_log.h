
#pragma once

#include <cstdio>
#include <cstdarg>

/** Reimplementation of ArduinoLog for use by tests */
class Logging
{
private:
  void logImpl(const char *level, const char *msg, va_list args)
  {
    printf("[%s] ", level);
    vprintf(msg, args);
  }

public:
  Logging() {}

  void fatal(const char *msg, ...)
  {
    va_list args;
    va_start(args, msg);
    logImpl("FATAL", msg, args);
    va_end(args);
  }

  void error(const char *msg, ...)
  {
    va_list args;
    va_start(args, msg);
    logImpl("ERROR", msg, args);
    va_end(args);
  }

  void info(const char *msg, ...)
  {
    va_list args;
    va_start(args, msg);
    logImpl("INFO", msg, args);
    va_end(args);
  }

  void verbose(const char *msg, ...)
  {
    va_list args;
    va_start(args, msg);
    logImpl("VERBOSE", msg, args);
    va_end(args);
  };
};

extern Logging Log;