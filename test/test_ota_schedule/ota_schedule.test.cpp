#include <ota_schedule.h>
#include <unity.h>

void test_first_attempt_when_never_tried(void) {
  TEST_ASSERT_TRUE(otaAttemptDue(1700000000UL, 0));
  TEST_ASSERT_TRUE(otaAttemptDue(0, 0));
}

void test_skip_when_clock_not_synced_after_prior_attempt(void) { TEST_ASSERT_FALSE(otaAttemptDue(0, 1700000000UL)); }

void test_skip_before_retry_interval_elapsed(void) {
  const uint32_t lastOta = 1700000000UL;
  TEST_ASSERT_FALSE(otaAttemptDue(lastOta + OTA_RETRY_INTERVAL_SECONDS - 1, lastOta));
}

void test_attempt_when_retry_interval_elapsed(void) {
  const uint32_t lastOta = 1700000000UL;
  TEST_ASSERT_TRUE(otaAttemptDue(lastOta + OTA_RETRY_INTERVAL_SECONDS, lastOta));
}

void test_attempt_well_after_retry_interval(void) {
  const uint32_t lastOta = 1700000000UL;
  TEST_ASSERT_TRUE(otaAttemptDue(lastOta + OTA_RETRY_INTERVAL_SECONDS + 3600, lastOta));
}

void test_unsynced_clock_does_not_underflow_into_due(void) { TEST_ASSERT_FALSE(otaAttemptDue(0, 1)); }

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_first_attempt_when_never_tried);
  RUN_TEST(test_skip_when_clock_not_synced_after_prior_attempt);
  RUN_TEST(test_skip_before_retry_interval_elapsed);
  RUN_TEST(test_attempt_when_retry_interval_elapsed);
  RUN_TEST(test_attempt_well_after_retry_interval);
  RUN_TEST(test_unsynced_clock_does_not_underflow_into_due);
  UNITY_END();
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  process();
  return 0;
}
