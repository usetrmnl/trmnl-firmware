#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

/// @brief File operations DebugLogCapture depends on, so its rotation logic can
///        run against an in-memory implementation in tests.
class LogFileSystem {
public:
  virtual ~LogFileSystem() = default;

  virtual bool exists(const char *path) = 0;

  /// @brief Size in bytes, or 0 when the file is absent.
  virtual size_t size(const char *path) = 0;

  /// @brief Append one record followed by a newline.
  virtual bool append(const char *path, const String &record) = 0;

  /// @brief Whole file contents, or "" when the file is absent.
  virtual String read(const char *path) = 0;

  virtual bool remove(const char *path) = 0;

  virtual bool rename(const char *from, const char *to) = 0;
};

/// @brief Collects verbose log records while the server's deadline is in the
///        future, storing them across two files of bounded size.
///
/// The device normally discards anything below error level, so this is how the
/// full trail leading to a failure gets off the device.
///
/// A deadline rather than an on/off flag: a device that stops receiving
/// successful responses is a common reason to be debugging one, and a flag
/// would leave it collecting indefinitely with no way to reach it.
///
/// Two files rather than one: when the newer file reaches max_bytes the older
/// is deleted and the newer takes its place, so total size stays under twice
/// max_bytes and the most recent records are the ones kept. Dropping records
/// individually would mean rewriting the file on every append, which wears the
/// flash and loses everything if power is lost partway through.
///
/// Which file is active is read from the filesystem rather than held in memory,
/// so collection continues across deep sleep.
class DebugLogCapture {
public:
  DebugLogCapture(LogFileSystem &fs, const char *older_path, const char *newer_path, size_t max_bytes);

  /// @brief Fix whether this wake cycle collects. Call once per wake, after the
  ///        filesystem is mounted.
  /// @param expires_at epoch seconds to collect until, 0 when never set
  /// @param now epoch seconds, 0 when the clock has not synced
  void begin(uint32_t expires_at, uint32_t now);

  bool active() const;

  bool store(const String &record);

  /// @brief Retained records, oldest first, comma joined for the log API.
  String gather();

  void clear();

  /// @brief The expiry to persist, given the stored value and what the latest
  ///        display response carried.
  ///
  /// A response without the field keeps the stored value. Error responses omit
  /// it, so treating a missing field as a cancellation would stop collection on
  /// the devices most likely to be under investigation. The server ends a
  /// window early by sending a past timestamp instead.
  ///
  /// @param stored currently persisted expiry
  /// @param from_response expiry from the display response, 0 when absent
  static uint32_t next_expiry(uint32_t stored, uint32_t from_response);

private:
  LogFileSystem &fs;
  const char *older_path;
  const char *newer_path;
  size_t max_bytes;
  bool capturing;
};
