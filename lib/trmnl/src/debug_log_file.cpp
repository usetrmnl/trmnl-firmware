#include <debug_log_file.h>

DebugLogFile::DebugLogFile(LogFileSystem &fs, const char *older_path, const char *newer_path, size_t max_bytes)
    : fs(fs), older_path(older_path), newer_path(newer_path), max_bytes(max_bytes) {}

bool DebugLogFile::append(const String &record) {
  if (!fs.exists(newer_path)) {
    // Only the older file is in play until it fills.
    if (fs.size(older_path) < max_bytes) {
      return fs.append(older_path, record);
    }
    return fs.append(newer_path, record);
  }

  if (fs.size(newer_path) >= max_bytes) {
    fs.remove(older_path);
    if (!fs.rename(newer_path, older_path)) {
      return false;
    }
  }
  return fs.append(newer_path, record);
}

String DebugLogFile::gather() {
  String records = fs.read(older_path) + fs.read(newer_path);

  // Records are stored one per line; the log API wants them comma-joined.
  records.trim();
  records.replace("\n", ",");
  return records;
}

void DebugLogFile::clear() {
  fs.remove(older_path);
  fs.remove(newer_path);
}
