"""Host regression test for the firmware logger, without ESP32 dependencies.

Run with: python3 scripts/tests/test_serial_log_memory.py
The final log_impl definition is compiled unchanged with tracked allocations and
platform stubs; the native suite otherwise uses lib/trmnl/src/test_logger.cpp.
Four test groups cover 47 mode/level/switch, repetition, failure and truncation scenarios.
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
HARNESS = r'''
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <trmnl_log.h>

static std::set<void *> live;
static int attempts = 0;
static int fail_at = 0;
static int serial_calls = 0;
static int normal_calls = 0;
static int dispatch_calls = 0;
static bool serial_logging_enabled = false;
static std::string output;
static std::string message;
static LogLevel expected_level;
static LogMode expected_mode;

static void *tracked_malloc(size_t size) {
  if (++attempts == fail_at) return nullptr;
  void *pointer = std::malloc(size);
  if (pointer) live.insert(pointer);
  return pointer;
}

static void tracked_free(void *pointer) {
  if (pointer && live.erase(pointer) != 1) std::abort();
  std::free(pointer);
}

struct SerialStub {
  void println(const char *message) {
    ++serial_calls;
    output = message;
  }
} Serial;

struct LogStub {
  void record(LogLevel level, const char *format, const char *text) {
    if (level != expected_level || strcmp(format, "%s")) std::abort();
    ++normal_calls;
    output = text;
  }
  void verboseln(const char *format, const char *text) { record(LOG_VERBOSE, format, text); }
  void infoln(const char *format, const char *text) { record(LOG_INFO, format, text); }
  void warningln(const char *format, const char *text) { record(LOG_WARN, format, text); }
  void errorln(const char *format, const char *text) { record(LOG_ERROR, format, text); }
  void fatalln(const char *format, const char *text) { record(LOG_FATAL, format, text); }
} Log;

static void format_message_truncated(char *buffer, int size, const char *format, va_list args) {
  if (vsnprintf(buffer, size, format, args) >= size) strcpy(buffer + size - 4, "...");
}

static void handle_store_submit(LogLevel level, const char *text, const char *file, int line, LogMode mode) {
  ++dispatch_calls;
  if (level != expected_level || mode != expected_mode || message != text ||
      strcmp(file, "src/example.cpp") || line != 17) std::abort();
}

#define malloc tracked_malloc
#define free tracked_free
// LOGGER_DEFINITION
#undef malloc
#undef free

int main(int argc, char **argv) {
  if (argc != 7) return 2;
  expected_mode = static_cast<LogMode>(atoi(argv[1]));
  expected_level = static_cast<LogLevel>(atoi(argv[2]));
  serial_logging_enabled = atoi(argv[3]);
  fail_at = atoi(argv[4]);
  const int count = atoi(argv[5]);
  const bool long_message = atoi(argv[6]);
  const std::string input = long_message ? std::string(600, 'x') : "message 42 with 100% literal";
  message = long_message ? std::string(508, 'x') + "..." : input;
  for (int i = 0; i < count; ++i) {
    log_impl(expected_level, expected_mode, "src/example.cpp", 17, "%s", input.c_str());
  }
  const bool serial_only = expected_mode == LOG_SERIAL_ONLY;
  const int expected_serial = !fail_at && serial_logging_enabled && serial_only ? count : 0;
  const int expected_normal = !fail_at && serial_logging_enabled && !serial_only ? count : 0;
  const int expected_dispatch = !fail_at && !serial_only ? count : 0;
  const bool correct = live.empty() && serial_calls == expected_serial &&
                       normal_calls == expected_normal && dispatch_calls == expected_dispatch &&
                       attempts == (fail_at ? fail_at : count * 2) &&
                       (!(expected_serial || expected_normal) || output == "src/example.cpp [17]: " + message);
  printf("live allocations: %zu; serial: %d; normal: %d; dispatch: %d\n",
         live.size(), serial_calls, normal_calls, dispatch_calls);
  for (void *pointer : live) std::free(pointer);
  return correct ? 0 : 1;
}
'''


class SerialLogMemoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which(os.environ.get("CXX", "c++"))
        if compiler is None:
            raise RuntimeError("A host C++ compiler is required")
        source = (ROOT / "src/app_logger.cpp").read_text()
        marker = "\nvoid log_impl("
        if source.count(marker) != 1:
            raise RuntimeError("Expected one final log_impl definition in app_logger.cpp")
        definition = "void log_impl(" + source.partition(marker)[2]
        cls.directory = tempfile.TemporaryDirectory(prefix="trmnl-log-test-")
        cls.addClassCleanup(cls.directory.cleanup)
        path = Path(cls.directory.name)
        harness = path / "logger.cpp"
        harness.write_text(HARNESS.replace("// LOGGER_DEFINITION", definition))
        cls.binary = path / "logger-test"
        compiled = subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-I", str(ROOT / "lib/trmnl/include"), str(harness), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60,
        )
        if compiled.returncode:
            raise RuntimeError(compiled.stdout + compiled.stderr)

    def check_scenario(self, mode=0, level=3, enabled=1, fail_at=0, repeat=1, long_message=0):
        result = subprocess.run(
            [str(self.binary), *map(str, (mode, level, enabled, fail_at, repeat, long_message))],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)

    def test_modes_levels_and_runtime_logging_switch(self):
        for mode in range(3):
            for level in range(5):
                for enabled in (0, 1):
                    with self.subTest(mode=mode, level=level, enabled=enabled):
                        self.check_scenario(mode=mode, level=level, enabled=enabled)

    def test_repeated_serial_logging_with_switch_on_or_off(self):
        for level in range(5):
            for enabled in (0, 1):
                with self.subTest(level=level, enabled=enabled):
                    self.check_scenario(level=level, enabled=enabled, repeat=100)

    def test_allocation_failures_in_each_mode(self):
        for mode in range(3):
            for fail_at in (1, 2):
                with self.subTest(mode=mode, fail_at=fail_at):
                    self.check_scenario(mode=mode, fail_at=fail_at)

    def test_truncated_serial_message_releases_buffers(self):
        self.check_scenario(long_message=1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
