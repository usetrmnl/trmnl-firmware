#include <api_response_parsing.h>
#include <bmp.h>
#include <unity.h>

void assert_response_equal(ApiSetupResponse expected, ApiSetupResponse actual) {
  TEST_ASSERT_EQUAL(expected.outcome, actual.outcome);
  TEST_ASSERT_EQUAL(expected.status, actual.status);
  TEST_ASSERT_EQUAL_STRING(expected.api_key.c_str(), actual.api_key.c_str());
  TEST_ASSERT_EQUAL_STRING(expected.friendly_id.c_str(), actual.friendly_id.c_str());
  TEST_ASSERT_EQUAL_STRING(expected.image_url.c_str(), actual.image_url.c_str());
  TEST_ASSERT_EQUAL_STRING(expected.message.c_str(), actual.message.c_str());
}

void test_parseResponse_apiSetup_success(void) {
  String input = "{\"status\":200,\"api_key\":\"1234\",\"friendly_id\":\"5678\",\"image_url\":\"http://example.com/"
                 "foo.bmp\",\"message\":\"hello\"}";

  ApiSetupResponse expected = {
      .outcome = ApiSetupOutcome::Ok,
      .status = 200,
      .api_key = "1234",
      .friendly_id = "5678",
      .image_url = "http://example.com/foo.bmp",
      .message = "hello"};

  assert_response_equal(expected, parseResponse_apiSetup(input));
}

void test_parseResponse_apiSetup_missing_fields(void) {
  String input = "{\"status\":200}";

  ApiSetupResponse expected = {
      .outcome = ApiSetupOutcome::Ok, .status = 200, .api_key = "", .friendly_id = "", .image_url = "", .message = ""};

  assert_response_equal(expected, parseResponse_apiSetup(input));
}

void test_parseResponse_apiSetup_statusError(void) {
  String input = "{\"status\":0}";

  ApiSetupResponse expected = {.outcome = ApiSetupOutcome::StatusError, .status = 0};

  assert_response_equal(expected, parseResponse_apiSetup(input));
}

void test_parseResponse_apiSetup_deserializationError(void) {
  String input = "invalid json";

  ApiSetupResponse expected = {.outcome = ApiSetupOutcome::DeserializationError};

  assert_response_equal(expected, parseResponse_apiSetup(input));
}

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_parseResponse_apiSetup_success);
  RUN_TEST(test_parseResponse_apiSetup_statusError);
  RUN_TEST(test_parseResponse_apiSetup_deserializationError);
  RUN_TEST(test_parseResponse_apiSetup_missing_fields);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}