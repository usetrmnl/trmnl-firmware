#include <sleep_duration.h>
#include <unity.h>

void test_elapsed_sleep_duration() { TEST_ASSERT_EQUAL_UINT32(900, sleepDurationSeconds(1700000900UL, 1700000000UL)); }

void test_no_previous_sleep() { TEST_ASSERT_EQUAL_UINT32(0, sleepDurationSeconds(1700000000UL, 0)); }

void test_unavailable_clock() { TEST_ASSERT_EQUAL_UINT32(0, sleepDurationSeconds(0, 1700000000UL)); }

void test_clock_moved_backwards() { TEST_ASSERT_EQUAL_UINT32(0, sleepDurationSeconds(1700000000UL, 1700000900UL)); }

void test_equal_timestamps() { TEST_ASSERT_EQUAL_UINT32(0, sleepDurationSeconds(1700000000UL, 1700000000UL)); }

void test_future_saved_timestamp() { TEST_ASSERT_EQUAL_UINT32(0, sleepDurationSeconds(1700000000UL, UINT32_MAX)); }

void test_large_valid_duration() { TEST_ASSERT_EQUAL_UINT32(UINT32_MAX - 1, sleepDurationSeconds(UINT32_MAX, 1)); }

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_elapsed_sleep_duration);
  RUN_TEST(test_no_previous_sleep);
  RUN_TEST(test_unavailable_clock);
  RUN_TEST(test_clock_moved_backwards);
  RUN_TEST(test_equal_timestamps);
  RUN_TEST(test_future_saved_timestamp);
  RUN_TEST(test_large_valid_duration);
  return UNITY_END();
}
