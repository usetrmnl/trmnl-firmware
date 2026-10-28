#include "../../src/http_result_handling.cpp" // the native env builds lib/trmnl only, so the test brings the module

#include <unity.h>

void test_success_and_non_errors_are_not_failures(void) {
  TEST_ASSERT_FALSE(requestIsFailure(HTTPS_SUCCESS));
  TEST_ASSERT_FALSE(requestIsFailure(HTTPS_NO_ERR));
  TEST_ASSERT_FALSE(requestIsFailure(HTTPS_NO_REGISTER));
  TEST_ASSERT_FALSE(requestIsFailure(HTTPS_RESET));
  TEST_ASSERT_FALSE(requestIsFailure(HTTPS_PLUGIN_NOT_ATTACHED));
}

void test_image_host_failure_is_not_a_failure(void) {
  TEST_ASSERT_FALSE(requestIsFailure(HTTPS_IMAGE_DOWNLOAD_FAILED));
}

void test_api_errors_are_failures(void) {
  TEST_ASSERT_TRUE(requestIsFailure(HTTPS_UNABLE_TO_CONNECT));
  TEST_ASSERT_TRUE(requestIsFailure(HTTPS_TIMED_OUT));
  TEST_ASSERT_TRUE(requestIsFailure(HTTPS_IMAGE_FILE_TOO_BIG));
}

void test_success_resets_the_count(void) {
  TEST_ASSERT_TRUE(requestFailureAction(HTTPS_SUCCESS, false, 3) == RequestFailureAction::ResetCount);
  TEST_ASSERT_TRUE(requestFailureAction(HTTPS_IMAGE_DOWNLOAD_FAILED, false, 2) == RequestFailureAction::ResetCount);
}

void test_manual_wake_failure_shows_now(void) {
  TEST_ASSERT_TRUE(requestFailureAction(HTTPS_UNABLE_TO_CONNECT, true, 2) == RequestFailureAction::ShowNow);
}

void test_timer_wake_failure_stays_quiet_up_to_the_limit(void) {
  TEST_ASSERT_TRUE(requestFailureAction(HTTPS_TIMED_OUT, false, 1) == RequestFailureAction::StayQuiet);
  TEST_ASSERT_TRUE(requestFailureAction(HTTPS_TIMED_OUT, false, MAX_QUIET_SLOW_RETRIES - 1) ==
                   RequestFailureAction::StayQuiet);
}

void test_timer_wake_failure_shows_the_error_after_the_limit(void) {
  TEST_ASSERT_TRUE(requestFailureAction(HTTPS_TIMED_OUT, false, MAX_QUIET_SLOW_RETRIES) ==
                   RequestFailureAction::ShowError);
}

void test_error_screens(void) {
  TEST_ASSERT_EQUAL(WIFI_INTERNAL_ERROR, requestErrorScreen(HTTPS_RESPONSE_CODE_INVALID, false));
  TEST_ASSERT_EQUAL(WIFI_INTERNAL_ERROR, requestErrorScreen(HTTPS_CLIENT_FAILED, true));
  TEST_ASSERT_EQUAL(IMAGE_DOWNLOAD_FAILED, requestErrorScreen(HTTPS_IMAGE_DOWNLOAD_FAILED, false));
  TEST_ASSERT_EQUAL(MSG_FORMAT_ERROR, requestErrorScreen(HTTPS_WRONG_IMAGE_FORMAT, false));
  TEST_ASSERT_EQUAL(WIFI_IMAGE_TIMEOUT, requestErrorScreen(HTTPS_TIMED_OUT, false));
  TEST_ASSERT_EQUAL(NONE, requestErrorScreen(HTTPS_SUCCESS, false));
}

void test_weak_signal_blames_the_wifi(void) {
  TEST_ASSERT_EQUAL(API_UNABLE_TO_CONNECT, requestErrorScreen(HTTPS_UNABLE_TO_CONNECT, false));
  TEST_ASSERT_EQUAL(WIFI_WEAK, requestErrorScreen(HTTPS_UNABLE_TO_CONNECT, true));
  TEST_ASSERT_EQUAL(API_SIZE_ERROR, requestErrorScreen(HTTPS_WRONG_IMAGE_SIZE, false));
  TEST_ASSERT_EQUAL(WIFI_WEAK, requestErrorScreen(HTTPS_WRONG_IMAGE_SIZE, true));
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_success_and_non_errors_are_not_failures);
  RUN_TEST(test_image_host_failure_is_not_a_failure);
  RUN_TEST(test_api_errors_are_failures);
  RUN_TEST(test_success_resets_the_count);
  RUN_TEST(test_manual_wake_failure_shows_now);
  RUN_TEST(test_timer_wake_failure_stays_quiet_up_to_the_limit);
  RUN_TEST(test_timer_wake_failure_shows_the_error_after_the_limit);
  RUN_TEST(test_error_screens);
  RUN_TEST(test_weak_signal_blames_the_wifi);
  UNITY_END();
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  process();
  return 0;
}
