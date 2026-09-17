// Protocol v1: the device tells the server which frame is on the panel (X-Frame-Id).
#include <api-client/request_headers.h>
#include <unity.h>

static String headerValue(const HttpHeaderList &headers, const char *name) {
  for (const auto &h : headers) {
    if (h.first == name) return h.second;
  }
  return String("<missing>");
}

void test_frame_id_header_is_sent_when_known(void) {
  ApiDisplayInputs inputs = {};
  inputs.macAddress = "AA:BB:CC:DD:EE:FF";
  inputs.frameId = "frame-1a2b3c4d5e";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_EQUAL_STRING("frame-1a2b3c4d5e", headerValue(headers, "X-Frame-Id").c_str());
}

void test_frame_id_header_is_omitted_when_empty(void) {
  ApiDisplayInputs inputs = {};
  inputs.macAddress = "AA:BB:CC:DD:EE:FF";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_EQUAL_STRING("<missing>", headerValue(headers, "X-Frame-Id").c_str());
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_frame_id_header_is_sent_when_known);
  RUN_TEST(test_frame_id_header_is_omitted_when_empty);
  return UNITY_END();
}
