#include <api_response_parsing.h>
#include <bmp.h>
#include <unity.h>

void assert_response_equal(ApiDisplayResponse expected, ApiDisplayResponse actual) {
  TEST_ASSERT_EQUAL(expected.outcome, actual.outcome);
  TEST_ASSERT_EQUAL_STRING(expected.image_url.c_str(), actual.image_url.c_str());
  TEST_ASSERT_EQUAL(expected.update_firmware, actual.update_firmware);
  TEST_ASSERT_EQUAL_STRING(expected.firmware_url.c_str(), actual.firmware_url.c_str());
  TEST_ASSERT_EQUAL_UINT64(expected.refresh_rate, actual.refresh_rate);
  TEST_ASSERT_EQUAL(expected.reset_firmware, actual.reset_firmware);
  TEST_ASSERT_EQUAL(expected.special_function, actual.special_function);
  TEST_ASSERT_EQUAL_STRING(expected.action.c_str(), actual.action.c_str());
}

void test_parseResponse_apiDisplay_success(void) {
  String input =
    "{\"status\":200,\"image_url\":\"http://example.com/"
    "foo.bmp\",\"filename\":\"empty_state\",\"update_firmware\":true,\"firmware_url\":\"https://example.com/"
    "firmware.bin\",\"refresh_rate\":123456,\"reset_firmware\":true,\"special_function\":\"identify\",\"action\":"
    "\"special_action\"}";

  ApiDisplayResponse expected = {
      .outcome = ApiDisplayOutcome::Ok,
      .image_url = "http://example.com/foo.bmp",
      .filename = "empty_state",
      .update_firmware = true,
      .firmware_url = "https://example.com/firmware.bin",
      .refresh_rate = 123456,
      .reset_firmware = true,
      .special_function = SPECIAL_FUNCTION::SF_IDENTIFY,
      .action = "special_action"};

  assert_response_equal(expected, parseResponse_apiDisplay(input));
}

void test_parseResponse_apiDisplay_deserializationError(void) {
  String input = "invalid json";

  ApiDisplayResponse expected = {.outcome = ApiDisplayOutcome::DeserializationError};

  assert_response_equal(expected, parseResponse_apiDisplay(input));
}

void test_parseResponse_apiDisplay_missing_fields(void) {
  String input = "{}";

  ApiDisplayResponse expected = {
      .outcome = ApiDisplayOutcome::Ok,
      .image_url = "",
      .update_firmware = false,
      .firmware_url = "",
      .refresh_rate = 0,
      .reset_firmware = false,
      .special_function = SPECIAL_FUNCTION::SF_NONE,
      .action = ""};

  assert_response_equal(expected, parseResponse_apiDisplay(input));
}

void test_parseResponse_apiDisplay_playlist_image_names(void) {
  String input = "{\"status\":0,\"playlist_image_names\":[\"plugin-1a2b3c\",\"mashup-4d5e6f\"]}";

  ApiDisplayResponse actual = parseResponse_apiDisplay(input);

  TEST_ASSERT_TRUE(actual.has_playlist_image_names);
  TEST_ASSERT_EQUAL_STRING("plugin-1a2b3c|mashup-4d5e6f", actual.playlist_image_names.c_str());
}

void test_parseResponse_apiDisplay_empty_playlist_image_names(void) {
  String input = "{\"status\":0,\"playlist_image_names\":[]}";

  ApiDisplayResponse actual = parseResponse_apiDisplay(input);

  TEST_ASSERT_TRUE(actual.has_playlist_image_names);
  TEST_ASSERT_EQUAL_STRING("", actual.playlist_image_names.c_str());
}

void test_parseResponse_apiDisplay_no_playlist_image_names(void) {
  String input = "{\"status\":0}";

  ApiDisplayResponse actual = parseResponse_apiDisplay(input);

  TEST_ASSERT_FALSE(actual.has_playlist_image_names);
}

void test_parseResponse_apiDisplay_treats_unknown_sf_as_none(void) {
  String input = "{\"status\":200,\"image_url\":\"http://example.com/"
                 "foo.bmp\",\"update_firmware\":true,\"firmware_url\":\"https://example.com/"
                 "firmware.bin\",\"refresh_rate\":123456,\"reset_firmware\":true,\"special_function\":\"unknown-"
                 "string\",\"action\":\"special_action\"}";

  auto parsed = parseResponse_apiDisplay(input);
  TEST_ASSERT_EQUAL(parsed.special_function, SPECIAL_FUNCTION::SF_NONE);
}

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_parseResponse_apiDisplay_success);
  RUN_TEST(test_parseResponse_apiDisplay_deserializationError);
  RUN_TEST(test_parseResponse_apiDisplay_treats_unknown_sf_as_none);
  RUN_TEST(test_parseResponse_apiDisplay_missing_fields);
  RUN_TEST(test_parseResponse_apiDisplay_playlist_image_names);
  RUN_TEST(test_parseResponse_apiDisplay_empty_playlist_image_names);
  RUN_TEST(test_parseResponse_apiDisplay_no_playlist_image_names);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}