#include <api_retry.h>
#include <unity.h>

void test_success_resets(void) {
  TEST_ASSERT_TRUE(apiRetryStep(false, false, 3) == ApiRetryStep::Reset);
  TEST_ASSERT_TRUE(apiRetryStep(false, true, 9) == ApiRetryStep::Reset);
}

void test_manual_wake_failure_shows_now(void) {
  TEST_ASSERT_TRUE(apiRetryStep(true, true, 1) == ApiRetryStep::ShowNow);
  TEST_ASSERT_TRUE(apiRetryStep(true, true, 9) == ApiRetryStep::ShowNow);
}

void test_timer_wake_failure_retries_quietly_up_to_the_limit(void) {
  TEST_ASSERT_TRUE(apiRetryStep(true, false, 1) == ApiRetryStep::RetryQuietly);
  TEST_ASSERT_TRUE(apiRetryStep(true, false, MAX_QUIET_API_RETRIES) == ApiRetryStep::RetryQuietly);
}

void test_timer_wake_failure_gives_up_after_the_limit(void) {
  TEST_ASSERT_TRUE(apiRetryStep(true, false, MAX_QUIET_API_RETRIES + 1) == ApiRetryStep::GiveUp);
  TEST_ASSERT_TRUE(apiRetryStep(true, false, 255) == ApiRetryStep::GiveUp);
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_success_resets);
  RUN_TEST(test_manual_wake_failure_shows_now);
  RUN_TEST(test_timer_wake_failure_retries_quietly_up_to_the_limit);
  RUN_TEST(test_timer_wake_failure_gives_up_after_the_limit);
  UNITY_END();
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  process();
  return 0;
}
