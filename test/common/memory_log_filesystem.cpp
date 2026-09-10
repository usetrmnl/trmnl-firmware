#include "memory_log_filesystem.h"

bool MemoryLogFileSystem::exists(const char *path) { return files.count(path) > 0; }

size_t MemoryLogFileSystem::size(const char *path) {
  auto found = files.find(path);
  return found == files.end() ? 0 : found->second.size();
}

bool MemoryLogFileSystem::append(const char *path, const String &record) {
  files[path] += std::string(record.c_str()) + "\n";
  return true;
}

String MemoryLogFileSystem::read(const char *path) {
  auto found = files.find(path);
  return found == files.end() ? String("") : String(found->second.c_str());
}

bool MemoryLogFileSystem::remove(const char *path) { return files.erase(path) > 0; }

bool MemoryLogFileSystem::rename(const char *from, const char *to) {
  if (failRename) return false;

  auto found = files.find(from);
  if (found == files.end()) return false;

  files[to] = found->second;
  files.erase(found);
  return true;
}
