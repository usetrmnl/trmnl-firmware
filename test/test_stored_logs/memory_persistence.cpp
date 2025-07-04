#include <unity.h>
#include <stored_logs.h>
#include <unordered_map>
#include <string>
#include <memory_persistence.h>

bool MemoryPersistence::recordExists(const char *key)
{
  return storage.find(key) != storage.end();
}

String MemoryPersistence::readString(const char *key, const String defaultValue)
{
  auto it = storage.find(key);
  if (it != storage.end())
  {
    return String(it->second.c_str());
  }
  return defaultValue;
}

uint32_t MemoryPersistence::readUint(const char *key, const uint32_t defaultValue)
{
  auto it = storage.find(key);
  if (it != storage.end())
  {
    try
    {
      return std::stoul(it->second);
    }
    catch (...)
    {
      return defaultValue;
    }
  }
  return defaultValue;
}

size_t MemoryPersistence::writeUint(const char *key, const uint32_t value)
{
  storage[key] = std::to_string(value);
  return sizeof(uint32_t);
}

size_t MemoryPersistence::writeString(const char *key, const char *value)
{
  storage[key] = value;
  return strlen(value);
}

uint8_t MemoryPersistence::readUChar(const char *key, const uint8_t defaultValue)
{
  auto it = storage.find(key);
  if (it != storage.end())
  {
    try
    {
      return static_cast<uint8_t>(std::stoi(it->second));
    }
    catch (...)
    {
      return defaultValue;
    }
  }
  return defaultValue;
}

size_t MemoryPersistence::writeUChar(const char *key, const uint8_t value)
{
  storage[key] = std::to_string(static_cast<int>(value));
  return sizeof(uint8_t);
}

bool MemoryPersistence::readBool(const char *key, const bool defaultValue)
{
  auto it = storage.find(key);
  if (it != storage.end())
  {
    return it->second == "true";
  }
  return defaultValue;
}

size_t MemoryPersistence::writeBool(const char *key, const bool value)
{
  storage[key] = value ? "true" : "false";
  return sizeof(bool);
}

bool MemoryPersistence::clear()
{
  storage.clear();
  return true;
}

bool MemoryPersistence::remove(const char *key)
{
  return storage.erase(key) > 0;
}

size_t MemoryPersistence::size()
{
  return storage.size();
}