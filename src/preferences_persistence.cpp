
#include <Arduino.h>
#include <ArduinoLog.h>
#include <Preferences.h>
#include <nvs.h>
#include <preferences_persistence.h>
#include <trmnl_log.h>

PreferencesPersistence::PreferencesPersistence(Preferences &preferences) : _preferences(preferences) {}

bool PreferencesPersistence::recordExists(const char *key) { return _preferences.isKey(key); }

String PreferencesPersistence::readString(const char *key, const String defaultValue) {
  return _preferences.getString(key, defaultValue);
}

uint32_t PreferencesPersistence::readUint(const char *key, const uint32_t defaultValue) {
  return _preferences.getUInt(key, defaultValue);
}

size_t PreferencesPersistence::writeUint(const char *key, const uint32_t value) {
  return _preferences.putUInt(key, value);
}

size_t PreferencesPersistence::writeString(const char *key, const char *value) {
  return _preferences.putString(key, value);
}

uint8_t PreferencesPersistence::readUChar(const char *key, const uint8_t defaultValue) {
  return _preferences.getUChar(key, defaultValue);
}

size_t PreferencesPersistence::writeUChar(const char *key, const uint8_t value) {
  return _preferences.putUChar(key, value);
}

bool PreferencesPersistence::readBool(const char *key, const bool defaultValue) {
  return _preferences.getBool(key, defaultValue);
}

size_t PreferencesPersistence::writeBool(const char *key, const bool value) { return _preferences.putBool(key, value); }

bool PreferencesPersistence::clear() { return _preferences.clear(); }

bool PreferencesPersistence::remove(const char *key) { return _preferences.remove(key); }

void log_nvs_usage() {
  nvs_stats_t nv;
  esp_err_t ret = nvs_get_stats(NULL, &nv);
  if (ret == ESP_OK) {
    float percent = (float)nv.used_entries / (float)nv.total_entries * 100.0f;
    char percent_str[16];
    dtostrf(percent, 0, 2, percent_str); // 2 decimal places
    Log_info("NVS Usage: %d/%d entries (%s%%)", nv.used_entries, nv.total_entries, percent_str);
  } else {
    Log_error("Failed to get NVS stats: %s", esp_err_to_name(ret));
  }
}
