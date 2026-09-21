// BYOS protocol v1: action none|partial|full, frame_id, full_url, regions_url, full_mode, sleep_mode.
#include <api_response_parsing.h>
#include <unity.h>

void test_v1_partial_fields_are_parsed(void) {
  String input =
    "{\"status\":0,\"action\":\"partial\",\"frame_id\":\"frame-1a2b3c4d5e\","
    "\"full_url\":\"http://rpi:8080/frames/frame-1a2b3c4d5e.png\","
    "\"regions_url\":\"http://rpi:8080/frames/frame-1a2b3c4d5e.regions?from=frame-0f9e8d7c6b\","
    "\"full_mode\":\"fast\",\"sleep_mode\":\"light\",\"refresh_rate\":60,"
    "\"image_url\":\"http://rpi:8080/frames/frame-1a2b3c4d5e.png\",\"filename\":\"frame-1a2b3c4d5e\","
    "\"update_firmware\":false,\"firmware_url\":null,\"reset_firmware\":false,\"special_function\":\"none\"}";

  auto r = parseResponse_apiDisplay(input);
  TEST_ASSERT_EQUAL(ApiDisplayOutcome::Ok, r.outcome);
  TEST_ASSERT_EQUAL(V1_ACTION_PARTIAL, r.v1_action);
  TEST_ASSERT_EQUAL_STRING("frame-1a2b3c4d5e", r.frame_id.c_str());
  TEST_ASSERT_EQUAL_STRING("http://rpi:8080/frames/frame-1a2b3c4d5e.png", r.full_url.c_str());
  TEST_ASSERT_EQUAL_STRING("http://rpi:8080/frames/frame-1a2b3c4d5e.regions?from=frame-0f9e8d7c6b",
                           r.regions_url.c_str());
  TEST_ASSERT_EQUAL(V1_FULL_FAST, r.full_mode);
  TEST_ASSERT_EQUAL(V1_SLEEP_LIGHT, r.sleep_mode);
  TEST_ASSERT_EQUAL_UINT64(60, r.refresh_rate);
}

void test_v1_none_action(void) {
  String input = "{\"status\":0,\"action\":\"none\",\"frame_id\":\"frame-1a2b3c4d5e\",\"sleep_mode\":\"deep\","
                 "\"refresh_rate\":300}";
  auto r = parseResponse_apiDisplay(input);
  TEST_ASSERT_EQUAL(V1_ACTION_NONE, r.v1_action);
  TEST_ASSERT_EQUAL(V1_SLEEP_DEEP, r.sleep_mode);
  TEST_ASSERT_EQUAL(V1_FULL_FULL, r.full_mode);
}

void test_stock_response_without_v1_fields_means_full_via_image_url(void) {
  String input = "{\"status\":0,\"image_url\":\"http://example.com/foo.png\",\"filename\":\"foo\","
                 "\"update_firmware\":false,\"refresh_rate\":900,\"reset_firmware\":false}";
  auto r = parseResponse_apiDisplay(input);
  TEST_ASSERT_EQUAL(V1_ACTION_FULL, r.v1_action);
  TEST_ASSERT_EQUAL_STRING("http://example.com/foo.png", r.full_url.c_str());
  TEST_ASSERT_EQUAL_STRING("foo", r.frame_id.c_str());
  TEST_ASSERT_EQUAL(V1_SLEEP_DEEP, r.sleep_mode);
  TEST_ASSERT_EQUAL(V1_FULL_FULL, r.full_mode);
}

void test_invalid_action_and_modes_fall_back_safely(void) {
  String input = "{\"status\":0,\"action\":\"teleport\",\"image_url\":\"http://x/a.png\",\"filename\":\"a\","
                 "\"sleep_mode\":\"hibernate\",\"full_mode\":\"turbo\",\"refresh_rate\":5}";
  auto r = parseResponse_apiDisplay(input);
  TEST_ASSERT_EQUAL(V1_ACTION_FULL, r.v1_action);
  TEST_ASSERT_EQUAL(V1_SLEEP_DEEP, r.sleep_mode);
  TEST_ASSERT_EQUAL(V1_FULL_FULL, r.full_mode);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_v1_partial_fields_are_parsed);
  RUN_TEST(test_v1_none_action);
  RUN_TEST(test_stock_response_without_v1_fields_means_full_via_image_url);
  RUN_TEST(test_invalid_action_and_modes_fall_back_safely);
  return UNITY_END();
}
