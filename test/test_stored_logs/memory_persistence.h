#include <unity.h>
// #include <stored_logs.h>
#include <unordered_map>
#include <string>
#include <persistence_interface.h>

class MemoryPersistence : public Persistence
{
public:
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

  size_t size();

private:
  std::unordered_map<std::string, std::string> storage;
};