#include <debug_log_capture.h>

DebugLogCapture::DebugLogCapture(LogFileSystem &fs, const char *older_path, const char *newer_path, size_t max_bytes)
    : fs(fs), older_path(older_path), newer_path(newer_path), max_bytes(max_bytes), capturing(false) {}

void DebugLogCapture::begin(uint32_t expires_at, uint32_t now) {
  // An unsynced clock cannot confirm the window is still open. Not collecting
  // is the safer default, since the next synced cycle starts collection again,
  // whereas a device left collecting cannot be reached to stop it.
  capturing = now != 0 && expires_at > now;
}

bool DebugLogCapture::active() const { return capturing; }

bool DebugLogCapture::store(const String &record) {
  if (!fs.exists(newer_path)) {
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

String DebugLogCapture::gather() {
  String records = fs.read(older_path) + fs.read(newer_path);

  records.trim();
  records.replace("\n", ",");
  return records;
}

void DebugLogCapture::clear() {
  fs.remove(older_path);
  fs.remove(newer_path);
}

uint32_t DebugLogCapture::next_expiry(uint32_t stored, uint32_t from_response) {
  return from_response == 0 ? stored : from_response;
}
