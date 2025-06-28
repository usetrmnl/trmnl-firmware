#pragma once

#include <Arduino.h>
#include <stdint.h>

/** interface */
class Persistence
{
public:
  virtual bool recordExists(const char *key) = 0;

  virtual String readString(const char *key, const String defaultValue) = 0;

  virtual uint32_t readUint(const char *key, const uint32_t defaultValue) = 0;

  virtual size_t writeUint(const char *key, const uint32_t value) = 0;

  virtual size_t writeString(const char *key, const char *value) = 0;

  virtual uint8_t readUChar(const char *key, const uint8_t defaultValue) = 0;

  virtual size_t writeUChar(const char *key, const uint8_t value) = 0;

  virtual bool readBool(const char *key, const bool defaultValue) = 0;

  virtual size_t writeBool(const char *key, const bool value) = 0;

  virtual bool clear() = 0;

  virtual bool remove(const char *key) = 0;
};