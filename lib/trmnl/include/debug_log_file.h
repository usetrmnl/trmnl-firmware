#pragma once

#include <Arduino.h>
#include <stddef.h>

/// @brief The file operations DebugLogFile needs. Implemented over the board's
///        filesystem in firmware builds, and in memory in tests.
class LogFileSystem {
public:
  virtual ~LogFileSystem() = default;

  virtual bool exists(const char *path) = 0;

  /// @brief Size in bytes, or 0 when the file is absent.
  virtual size_t size(const char *path) = 0;

  /// @brief Append one record, followed by a newline.
  virtual bool append(const char *path, const String &record) = 0;

  /// @brief Whole file contents, or "" when the file is absent.
  virtual String read(const char *path) = 0;

  virtual bool remove(const char *path) = 0;

  virtual bool rename(const char *from, const char *to) = 0;
};

/// @brief Bounded debug log store built from two files.
///
/// Records append to the newer file. When it fills, the older file is dropped
/// and the newer one takes its place, so retention sits between one and two
/// times `max_bytes` and the newest records always survive - the lines just
/// before a fault are the ones worth having.
///
/// Evicting a whole file at a time avoids rewriting to drop a single line,
/// which would churn flash on every append and lose everything to a power cut
/// mid-rewrite.
///
/// Which file is active is derived from what is on disk, so it survives deep
/// sleep with no extra stored state.
class DebugLogFile {
public:
  DebugLogFile(LogFileSystem &fs, const char *older_path, const char *newer_path, size_t max_bytes);

  bool append(const String &record);

  /// @brief All retained records, oldest first, comma-joined for the log API.
  String gather();

  void clear();

private:
  LogFileSystem &fs;
  const char *older_path;
  const char *newer_path;
  size_t max_bytes;
};
