#include "debug_log_mode.h"

#include <unity.h>

static const uint32_t NOW = 1609459200;
static const uint32_t UNSYNCED_CLOCK = 0;

void test_captures_verbose_while_the_deadline_is_in_the_future() {
  TEST_ASSERT_EQUAL(LOG_VERBOSE, debug_log_threshold(NOW + 3600, NOW));
}

void test_stops_capturing_once_the_deadline_passes() {
  TEST_ASSERT_EQUAL(LOG_ERROR, debug_log_threshold(NOW - 1, NOW));
}

void test_stops_capturing_exactly_at_the_deadline() {
  TEST_ASSERT_EQUAL(LOG_ERROR, debug_log_threshold(NOW, NOW));
}

void test_does_not_capture_when_no_deadline_was_ever_set() {
  TEST_ASSERT_EQUAL(LOG_ERROR, debug_log_threshold(0, NOW));
}

void test_does_not_capture_when_the_clock_is_unsynced() {
  // The window cannot be verified, and a device stuck verbose drains unattended.
  TEST_ASSERT_EQUAL(LOG_ERROR, debug_log_threshold(NOW + 3600, UNSYNCED_CLOCK));
}

void test_a_response_carrying_an_expiry_replaces_what_is_stored() {
  TEST_ASSERT_EQUAL_UINT32(NOW + 7200, debug_log_expiry_to_store(NOW + 3600, NOW + 7200));
}

void test_a_response_without_the_field_leaves_the_stored_expiry_alone() {
  // Error responses omit the field; treating that as a cancel would drop a
  // device out of capture just when it is being watched.
  TEST_ASSERT_EQUAL_UINT32(NOW + 3600, debug_log_expiry_to_store(NOW + 3600, 0));
}

void test_the_server_can_cancel_early_with_a_past_expiry() {
  TEST_ASSERT_EQUAL_UINT32(1, debug_log_expiry_to_store(NOW + 3600, 1));
  TEST_ASSERT_EQUAL(LOG_ERROR, debug_log_threshold(1, NOW));
}

void setUp(void) {}

void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_captures_verbose_while_the_deadline_is_in_the_future);
  RUN_TEST(test_stops_capturing_once_the_deadline_passes);
  RUN_TEST(test_stops_capturing_exactly_at_the_deadline);
  RUN_TEST(test_does_not_capture_when_no_deadline_was_ever_set);
  RUN_TEST(test_does_not_capture_when_the_clock_is_unsynced);
  RUN_TEST(test_a_response_carrying_an_expiry_replaces_what_is_stored);
  RUN_TEST(test_a_response_without_the_field_leaves_the_stored_expiry_alone);
  RUN_TEST(test_the_server_can_cancel_early_with_a_past_expiry);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
