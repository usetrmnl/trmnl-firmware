
#include <persistence_interface.h>
#include <Preferences.h>

/**
 * NVMRAM-backed persistence on ESP32
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html
 */
class PreferencesPersistence : public Persistence
{
public:
  PreferencesPersistence(Preferences &preferences);

  bool recordExists(const char *key) override;

  String readString(const char *key, const String defaultValue) override;

  uint32_t readUint(const char *key, const uint32_t defaultValue) override;

  size_t writeUint(const char *key, const uint32_t value) override;

  size_t writeString(const char *key, const char *value) override;

  uint8_t readUChar(const char *key, const uint8_t defaultValue) override;

  size_t writeUChar(const char *key, const uint8_t value) override;

  bool readBool(const char *key, const bool defaultValue) override;

  size_t writeBool(const char *key, const bool value) override;

  bool clear() override;

  bool remove(const char *key) override;

private:
  Preferences& _preferences;
};