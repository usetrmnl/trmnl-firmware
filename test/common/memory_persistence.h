#pragma once

#ifndef PIO_UNIT_TESTING
#error "MemoryPersistence is a test-only fake and must not be included in firmware builds"
#endif

#include <persistence_interface.h>
#include <string>
#include <unordered_map>

// In-memory Persistence fake shared by native unit tests. test/common/ is
// exposed to test builds as a library via `lib_extra_dirs = test` in the
// native envs; firmware envs never see it.
class MemoryPersistence : public Persistence {
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

  // Number of write* calls made; lets tests assert that redundant writes are skipped.
  uint32_t writeCount = 0;

private:
  std::unordered_map<std::string, std::string> storage;
};
