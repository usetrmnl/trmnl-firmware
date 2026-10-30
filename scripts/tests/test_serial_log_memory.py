"""Host regression test for the firmware logger, without ESP32 dependencies.

Run with: python3 scripts/tests/test_serial_log_memory.py
The final log_impl definition is compiled unchanged with tracked allocations and
platform stubs; the native suite otherwise uses lib/trmnl/src/test_logger.cpp.
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
static int store_calls = 0;
static std::string output;

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
  void verboseln(const char *) { ++normal_calls; }
  void infoln(const char *) { ++normal_calls; }
  void warningln(const char *) { ++normal_calls; }
  void errorln(const char *) { ++normal_calls; }
  void fatalln(const char *) { ++normal_calls; }
} Log;

static void format_message_truncated(char *buffer, int size, const char *format, va_list args) {
  if (vsnprintf(buffer, size, format, args) >= size) strcpy(buffer + size - 4, "...");
}

static void handle_store_submit(LogLevel, const char *message, const char *file, int line, LogMode) {
  ++store_calls;
  if (strcmp(message, "message 42") || strcmp(file, "src/example.cpp") || line != 17) std::abort();
}

#define malloc tracked_malloc
#define free tracked_free
// LOGGER_DEFINITION
#undef malloc
#undef free

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string scenario = argv[1];
  if (scenario == "oom-first") fail_at = 1;
  if (scenario == "oom-second") fail_at = 2;
  const int count = scenario == "repeated" ? 100 : 1;
  const bool normal = scenario == "normal";
  const LogMode mode = normal ? LOG_STORE_ONLY : LOG_SERIAL_ONLY;
  for (int i = 0; i < count; ++i) {
    log_impl(LOG_ERROR, mode, "src/example.cpp", 17, "message %d", 42);
  }
  const int expected_serial = fail_at || normal ? 0 : count;
  const bool correct = live.empty() && serial_calls == expected_serial &&
                       normal_calls == (normal ? 1 : 0) && store_calls == (normal ? 1 : 0) &&
                       (!expected_serial || output == "src/example.cpp [17]: message 42");
  printf("live allocations: %zu; serial: %d; normal: %d; store: %d\n",
         live.size(), serial_calls, normal_calls, store_calls);
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

    def check_scenario(self, scenario):
        result = subprocess.run(
            [str(self.binary), scenario], capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)

    def test_serial_output_releases_both_buffers(self):
        self.check_scenario("serial")

    def test_repeated_serial_output_does_not_accumulate_allocations(self):
        self.check_scenario("repeated")

    def test_first_allocation_failure_is_safe(self):
        self.check_scenario("oom-first")

    def test_second_allocation_failure_releases_first_buffer(self):
        self.check_scenario("oom-second")

    def test_normal_logging_still_stores_and_releases_buffers(self):
        self.check_scenario("normal")


if __name__ == "__main__":
    unittest.main(verbosity=2)
