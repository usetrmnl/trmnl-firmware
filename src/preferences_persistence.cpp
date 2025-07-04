
#include <Preferences.h>
#include <ArduinoLog.h>
#include <preferences_persistence.h>

PreferencesPersistence::PreferencesPersistence(Preferences &preferences) : _preferences(preferences) {}

bool PreferencesPersistence::recordExists(const char *key)
{
  return _preferences.isKey(key);
}

String PreferencesPersistence::readString(const char *key, const String defaultValue)
{
  return _preferences.getString(key, defaultValue);
}

uint32_t PreferencesPersistence::readUint(const char *key, const uint32_t defaultValue)
{
  return _preferences.getUInt(key, defaultValue);
}

size_t PreferencesPersistence::writeUint(const char *key, const uint32_t value)
{
  return _preferences.putUInt(key, value);
}

size_t PreferencesPersistence::writeString(const char *key, const char *value)
{
  return _preferences.putString(key, value);
}

uint8_t PreferencesPersistence::readUChar(const char *key, const uint8_t defaultValue)
{
  return _preferences.getUChar(key, defaultValue);
}

size_t PreferencesPersistence::writeUChar(const char *key, const uint8_t value)
{
  return _preferences.putUChar(key, value);
}

bool PreferencesPersistence::readBool(const char *key, const bool defaultValue)
{
  return _preferences.getBool(key, defaultValue);
}

size_t PreferencesPersistence::writeBool(const char *key, const bool value)
{
  return _preferences.putBool(key, value);
}

bool PreferencesPersistence::clear()
{
  return _preferences.clear();
}

bool PreferencesPersistence::remove(const char *key)
{
  return _preferences.remove(key);
}
