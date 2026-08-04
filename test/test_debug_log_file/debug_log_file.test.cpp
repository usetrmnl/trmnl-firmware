#include "debug_log_file.h"

#include <memory_log_filesystem.h>
#include <unity.h>

// Records are stored with a trailing newline, so "aaa" occupies 4 bytes.
static const size_t MAX_BYTES = 10;

void test_gathers_records_comma_joined() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  subject.append("one");
  subject.append("two");

  TEST_ASSERT_EQUAL_STRING("one,two", subject.gather().c_str());
}

void test_gather_is_empty_when_nothing_stored() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  TEST_ASSERT_EQUAL_STRING("", subject.gather().c_str());
}

void test_spills_into_second_file_once_first_is_full() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  subject.append("aaa"); // 4
  subject.append("bbb"); // 8
  TEST_ASSERT_FALSE(fs.exists("/b"));

  subject.append("ccc"); // 12, tips the first file over the limit
  TEST_ASSERT_FALSE(fs.exists("/b"));

  subject.append("ddd"); // first file is over, so this starts the second
  TEST_ASSERT_TRUE(fs.exists("/b"));

  TEST_ASSERT_EQUAL_STRING("aaa,bbb,ccc,ddd", subject.gather().c_str());
}

void test_drops_oldest_half_and_keeps_newest_when_both_files_fill() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  subject.append("aaa");
  subject.append("bbb");
  subject.append("ccc"); // first file now over the limit
  subject.append("ddd"); // starts the second file
  subject.append("eee");
  subject.append("fff"); // second file now over the limit

  TEST_ASSERT_EQUAL_STRING("aaa,bbb,ccc,ddd,eee,fff", subject.gather().c_str());

  // The next append evicts the oldest file and promotes the newer one.
  subject.append("ggg");

  TEST_ASSERT_EQUAL_STRING("ddd,eee,fff,ggg", subject.gather().c_str());
}

void test_eviction_is_bounded_over_many_appends() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  for (int i = 0; i < 200; i++) {
    subject.append("xxx");
  }

  // Retention stays within two files' worth regardless of how much is written.
  TEST_ASSERT_LESS_OR_EQUAL(MAX_BYTES * 2, fs.size("/a") + fs.size("/b"));
}

void test_newest_record_always_survives_eviction() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  for (int i = 0; i < 50; i++) {
    subject.append("old");
  }
  subject.append("crash");

  String gathered = subject.gather();
  TEST_ASSERT_TRUE(gathered.endsWith("crash"));
}

void test_active_file_is_derived_from_disk_so_it_survives_a_restart() {
  MemoryLogFileSystem fs;

  {
    DebugLogFile before_sleep(fs, "/a", "/b", MAX_BYTES);
    before_sleep.append("aaa");
    before_sleep.append("bbb");
    before_sleep.append("ccc");
    before_sleep.append("ddd"); // second file is now active
  }

  // A fresh instance holds no state of its own, standing in for a wake from sleep.
  DebugLogFile after_wake(fs, "/a", "/b", MAX_BYTES);
  after_wake.append("eee");

  TEST_ASSERT_EQUAL_STRING("aaa,bbb,ccc,ddd,eee", after_wake.gather().c_str());
}

void test_clear_removes_both_files() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  subject.append("aaa");
  subject.append("bbb");
  subject.append("ccc");
  subject.append("ddd");

  subject.clear();

  TEST_ASSERT_FALSE(fs.exists("/a"));
  TEST_ASSERT_FALSE(fs.exists("/b"));
}

void test_reports_failure_when_rotation_cannot_rename() {
  MemoryLogFileSystem fs;
  DebugLogFile subject(fs, "/a", "/b", MAX_BYTES);

  subject.append("aaa");
  subject.append("bbb");
  subject.append("ccc");
  subject.append("ddd");
  subject.append("eee");
  subject.append("fff");

  fs.failRename = true;
  TEST_ASSERT_FALSE(subject.append("ggg"));
}

void setUp(void) {}

void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_gathers_records_comma_joined);
  RUN_TEST(test_gather_is_empty_when_nothing_stored);
  RUN_TEST(test_spills_into_second_file_once_first_is_full);
  RUN_TEST(test_drops_oldest_half_and_keeps_newest_when_both_files_fill);
  RUN_TEST(test_eviction_is_bounded_over_many_appends);
  RUN_TEST(test_newest_record_always_survives_eviction);
  RUN_TEST(test_active_file_is_derived_from_disk_so_it_survives_a_restart);
  RUN_TEST(test_clear_removes_both_files);
  RUN_TEST(test_reports_failure_when_rotation_cannot_rename);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
