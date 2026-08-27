#include <api_response_parsing.h>
#include <unity.h>

// What /api/display answers a device that finished setup but has not been claimed to an
// account yet. Captured from production; the 202 arm of handleApiDisplayResponse() never
// reads image_url, so the global filename buffer is still empty when downloadAndShow()
// reaches the image fetch.
static const char *UNCLAIMED_DEVICE_BODY =
    "{\"status\":202,\"image_url\":null,\"filename\":null,\"refresh_rate\":908,"
    "\"reset_firmware\":false,\"update_firmware\":false,\"firmware_url\":\"\","
    "\"special_function\":\"identify\",\"maximum_compatibility\":false,"
    "\"temperature_profile\":\"default\"}";

// The URL check HTTPClient::begin(WiFiClient&, String) runs before it will connect
// (framework-arduinoespressif32/libraries/HTTPClient/src/HTTPClient.cpp).
static bool http_client_accepts_url(const String &url) {
  int index = url.indexOf(':');
  if (index < 0) return false;
  String protocol = url.substring(0, index);
  return protocol == "http" || protocol == "https";
}

void test_unclaimed_device_response_carries_no_image_url(void) {
  String body(UNCLAIMED_DEVICE_BODY);
  ApiDisplayResponse response = parseResponse_apiDisplay(body);

  TEST_ASSERT_EQUAL(ApiDisplayOutcome::Ok, response.outcome);
  TEST_ASSERT_EQUAL_UINT64(202, response.status);
  TEST_ASSERT_EQUAL_STRING("", response.image_url.c_str());
}

void test_http_client_rejects_an_empty_image_url(void) {
  TEST_ASSERT_FALSE(http_client_accepts_url(String("")));
  TEST_ASSERT_TRUE(http_client_accepts_url(String("https://usetrmnl.com/images/screen.png")));
}

void setUp(void) {}

void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_unclaimed_device_response_carries_no_image_url);
  RUN_TEST(test_http_client_rejects_an_empty_image_url);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
