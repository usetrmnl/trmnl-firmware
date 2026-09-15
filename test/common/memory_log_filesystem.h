#pragma once

#ifndef PIO_UNIT_TESTING
#error "MemoryLogFileSystem is a test-only fake and must not be included in firmware builds"
#endif

#include <debug_log_capture.h>
#include <string>
#include <unordered_map>

// In-memory LogFileSystem fake shared by native unit tests. test/common/ is
// exposed to test builds as a library via `lib_extra_dirs = test` in the
// native envs; firmware envs never see it.
class MemoryLogFileSystem : public LogFileSystem {
public:
  bool exists(const char *path) override;
  size_t size(const char *path) override;
  bool append(const char *path, const String &record) override;
  String read(const char *path) override;
  bool remove(const char *path) override;
  bool rename(const char *from, const char *to) override;

  // Set to fail the next rename, so tests can cover a rotation that goes wrong.
  bool failRename = false;

private:
  std::unordered_map<std::string, std::string> files;
};
