#include "debug_log_capture.h"

#include <memory_log_filesystem.h>
#include <unity.h>

// Records are stored with a trailing newline, so "aaa" occupies 4 bytes and
// three records overrun a 10 byte file.
static const size_t MAX_BYTES = 10;
static const uint32_t NOW = 1609459200;
static const uint32_t UNSYNCED_CLOCK = 0;

static MemoryLogFileSystem fs;

static DebugLogCapture subject() { return DebugLogCapture(fs, "/a", "/b", MAX_BYTES); }

static DebugLogCapture active_subject() {
  DebugLogCapture capture = subject();
  capture.begin(NOW + 3600, NOW);
  return capture;
}

static void fill_both_files(DebugLogCapture &capture) {
  capture.store("aaa");
  capture.store("bbb");
  capture.store("ccc"); // older file is now over the limit
  capture.store("ddd"); // starts the newer file
  capture.store("eee");
  capture.store("fff"); // newer file is now over the limit
}

void test_begin_activates_while_the_deadline_is_ahead() {
  DebugLogCapture capture = subject();

  capture.begin(NOW + 3600, NOW);

  TEST_ASSERT_TRUE(capture.active());
}

void test_begin_stays_inactive_once_the_deadline_passes() {
  DebugLogCapture capture = subject();

  capture.begin(NOW - 1, NOW);

  TEST_ASSERT_FALSE(capture.active());
}

void test_begin_stays_inactive_exactly_at_the_deadline() {
  DebugLogCapture capture = subject();

  capture.begin(NOW, NOW);

  TEST_ASSERT_FALSE(capture.active());
}

void test_begin_stays_inactive_when_no_deadline_was_set() {
  DebugLogCapture capture = subject();

  capture.begin(0, NOW);

  TEST_ASSERT_FALSE(capture.active());
}

void test_begin_stays_inactive_when_the_clock_is_unsynced() {
  DebugLogCapture capture = subject();

  capture.begin(NOW + 3600, UNSYNCED_CLOCK);

  TEST_ASSERT_FALSE(capture.active());
}

void test_is_inactive_before_begin_is_called() {
  TEST_ASSERT_FALSE(subject().active());
}

void test_next_expiry_takes_the_value_the_response_carried() {
  TEST_ASSERT_EQUAL_UINT32(NOW + 7200, DebugLogCapture::next_expiry(NOW + 3600, NOW + 7200));
}

void test_next_expiry_keeps_the_stored_value_when_the_response_omits_it() {
  TEST_ASSERT_EQUAL_UINT32(NOW + 3600, DebugLogCapture::next_expiry(NOW + 3600, 0));
}

void test_next_expiry_accepts_a_past_value_so_a_window_can_end_early() {
  TEST_ASSERT_EQUAL_UINT32(1, DebugLogCapture::next_expiry(NOW + 3600, 1));
}

void test_gather_joins_records_with_commas() {
  DebugLogCapture capture = active_subject();

  capture.store("one");
  capture.store("two");

  TEST_ASSERT_EQUAL_STRING("one,two", capture.gather().c_str());
}

void test_gather_answers_empty_when_nothing_is_stored() {
  TEST_ASSERT_EQUAL_STRING("", active_subject().gather().c_str());
}

void test_store_starts_the_newer_file_once_the_older_is_full() {
  DebugLogCapture capture = active_subject();

  capture.store("aaa");
  capture.store("bbb");
  capture.store("ccc");
  capture.store("ddd");

  TEST_ASSERT_TRUE(fs.exists("/b"));
}

void test_store_keeps_everything_until_both_files_are_full() {
  DebugLogCapture capture = active_subject();

  fill_both_files(capture);

  TEST_ASSERT_EQUAL_STRING("aaa,bbb,ccc,ddd,eee,fff", capture.gather().c_str());
}

void test_store_drops_the_older_file_when_both_are_full() {
  DebugLogCapture capture = active_subject();
  fill_both_files(capture);

  capture.store("ggg");

  TEST_ASSERT_EQUAL_STRING("ddd,eee,fff,ggg", capture.gather().c_str());
}

void test_store_keeps_total_size_bounded() {
  DebugLogCapture capture = active_subject();

  for (int i = 0; i < 200; i++) {
    capture.store("xxx");
  }

  TEST_ASSERT_LESS_OR_EQUAL(MAX_BYTES * 2, fs.size("/a") + fs.size("/b"));
}

void test_store_keeps_the_most_recent_record() {
  DebugLogCapture capture = active_subject();

  for (int i = 0; i < 50; i++) {
    capture.store("old");
  }
  capture.store("last");

  TEST_ASSERT_TRUE(capture.gather().endsWith("last"));
}

void test_store_answers_false_when_the_files_cannot_be_rotated() {
  DebugLogCapture capture = active_subject();
  fill_both_files(capture);
  fs.failRename = true;

  TEST_ASSERT_FALSE(capture.store("ggg"));
}

void test_store_resumes_into_the_same_file_after_a_restart() {
  DebugLogCapture before_sleep = active_subject();
  before_sleep.store("aaa");
  before_sleep.store("bbb");
  before_sleep.store("ccc");
  before_sleep.store("ddd");

  DebugLogCapture after_wake = active_subject();
  after_wake.store("eee");

  TEST_ASSERT_EQUAL_STRING("aaa,bbb,ccc,ddd,eee", after_wake.gather().c_str());
}

void test_clear_removes_the_older_file() {
  DebugLogCapture capture = active_subject();
  fill_both_files(capture);

  capture.clear();

  TEST_ASSERT_FALSE(fs.exists("/a"));
}

void test_clear_removes_the_newer_file() {
  DebugLogCapture capture = active_subject();
  fill_both_files(capture);

  capture.clear();

  TEST_ASSERT_FALSE(fs.exists("/b"));
}

void setUp(void) { fs = MemoryLogFileSystem(); }

void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_begin_activates_while_the_deadline_is_ahead);
  RUN_TEST(test_begin_stays_inactive_once_the_deadline_passes);
  RUN_TEST(test_begin_stays_inactive_exactly_at_the_deadline);
  RUN_TEST(test_begin_stays_inactive_when_no_deadline_was_set);
  RUN_TEST(test_begin_stays_inactive_when_the_clock_is_unsynced);
  RUN_TEST(test_is_inactive_before_begin_is_called);
  RUN_TEST(test_next_expiry_takes_the_value_the_response_carried);
  RUN_TEST(test_next_expiry_keeps_the_stored_value_when_the_response_omits_it);
  RUN_TEST(test_next_expiry_accepts_a_past_value_so_a_window_can_end_early);
  RUN_TEST(test_gather_joins_records_with_commas);
  RUN_TEST(test_gather_answers_empty_when_nothing_is_stored);
  RUN_TEST(test_store_starts_the_newer_file_once_the_older_is_full);
  RUN_TEST(test_store_keeps_everything_until_both_files_are_full);
  RUN_TEST(test_store_drops_the_older_file_when_both_are_full);
  RUN_TEST(test_store_keeps_total_size_bounded);
  RUN_TEST(test_store_keeps_the_most_recent_record);
  RUN_TEST(test_store_answers_false_when_the_files_cannot_be_rotated);
  RUN_TEST(test_store_resumes_into_the_same_file_after_a_restart);
  RUN_TEST(test_clear_removes_the_older_file);
  RUN_TEST(test_clear_removes_the_newer_file);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
