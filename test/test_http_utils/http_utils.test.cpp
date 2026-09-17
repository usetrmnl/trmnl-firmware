#include <http_utils.h>
#include <unity.h>

// --- httpOriginOf ----------------------------------------------------------

void test_origin_strips_path(void) {
  TEST_ASSERT_EQUAL_STRING("https://trmnl.app", httpOriginOf("https://trmnl.app/api/display").c_str());
}

void test_origin_keeps_port(void) {
  TEST_ASSERT_EQUAL_STRING("http://192.168.1.24:8080",
                           httpOriginOf("http://192.168.1.24:8080/img/mock-1.png?x=1").c_str());
}

void test_origin_without_path_is_unchanged(void) {
  TEST_ASSERT_EQUAL_STRING("https://trmnl.app", httpOriginOf("https://trmnl.app").c_str());
}

void test_origin_without_scheme_is_unchanged(void) {
  TEST_ASSERT_EQUAL_STRING("trmnl.app/api", httpOriginOf("trmnl.app/api").c_str());
}

// --- resolveRedirectLocation -----------------------------------------------

void test_redirect_absolute_http_location_wins(void) {
  TEST_ASSERT_EQUAL_STRING("http://cdn.example/img.png",
                           resolveRedirectLocation("http://cdn.example/img.png", "https://trmnl.app").c_str());
}

void test_redirect_absolute_https_location_wins(void) {
  TEST_ASSERT_EQUAL_STRING("https://cdn.example/img.png",
                           resolveRedirectLocation("https://cdn.example/img.png", "https://trmnl.app").c_str());
}

void test_redirect_relative_location_is_appended_to_origin(void) {
  TEST_ASSERT_EQUAL_STRING("https://trmnl.app/api/display?v=2",
                           resolveRedirectLocation("/api/display?v=2", "https://trmnl.app").c_str());
}

// --- isSameOrigin ----------------------------------------------------------

void test_same_origin_true_for_prefix(void) {
  TEST_ASSERT_TRUE(isSameOrigin("https://trmnl.app/images/a.bmp", "https://trmnl.app"));
}

void test_same_origin_false_for_other_host(void) {
  TEST_ASSERT_FALSE(isSameOrigin("https://s3.amazonaws.com/trmnl/a.bmp", "https://trmnl.app"));
}

void test_same_origin_false_for_empty_base(void) {
  TEST_ASSERT_FALSE(isSameOrigin("https://trmnl.app/images/a.bmp", ""));
}

// --- sniffImageContentType -------------------------------------------------

void test_sniff_bmp(void) {
  const uint8_t bmp[] = {'B', 'M', 0x3E, 0xBB};
  TEST_ASSERT_EQUAL_STRING("image/bmp", sniffImageContentType(bmp, sizeof(bmp)));
}

void test_sniff_png(void) {
  const uint8_t png[] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 0x00};
  TEST_ASSERT_EQUAL_STRING("image/png", sniffImageContentType(png, sizeof(png)));
}

void test_sniff_jpeg(void) {
  const uint8_t jpg[] = {0xFF, 0xD8, 0xFF, 0xE0};
  TEST_ASSERT_EQUAL_STRING("image/jpeg", sniffImageContentType(jpg, sizeof(jpg)));
}

void test_sniff_garbage_is_null(void) {
  const uint8_t junk[] = {'{', '"', 's', 't'};
  TEST_ASSERT_NULL(sniffImageContentType(junk, sizeof(junk)));
}

void test_sniff_too_short_is_null(void) {
  const uint8_t one[] = {'B'};
  TEST_ASSERT_NULL(sniffImageContentType(one, sizeof(one)));
  TEST_ASSERT_NULL(sniffImageContentType(nullptr, 0));
}

// --- shouldFastRetryCode ---------------------------------------------------

void test_retry_transport_errors(void) {
  TEST_ASSERT_TRUE(shouldFastRetryCode(-1));  // HTTPC_ERROR_CONNECTION_REFUSED / modem timeout
  TEST_ASSERT_TRUE(shouldFastRetryCode(-11)); // HTTPC_ERROR_READ_TIMEOUT
  TEST_ASSERT_TRUE(shouldFastRetryCode(0));   // no client / modem ERROR
}

void test_retry_server_errors(void) {
  TEST_ASSERT_TRUE(shouldFastRetryCode(500));
  TEST_ASSERT_TRUE(shouldFastRetryCode(502));
  TEST_ASSERT_TRUE(shouldFastRetryCode(503));
  TEST_ASSERT_TRUE(shouldFastRetryCode(599));
}

void test_retry_request_timeout(void) { TEST_ASSERT_TRUE(shouldFastRetryCode(408)); }

void test_no_retry_success_and_redirects(void) {
  TEST_ASSERT_FALSE(shouldFastRetryCode(200));
  TEST_ASSERT_FALSE(shouldFastRetryCode(301));
  TEST_ASSERT_FALSE(shouldFastRetryCode(307));
}

void test_no_retry_deterministic_client_errors(void) {
  TEST_ASSERT_FALSE(shouldFastRetryCode(400));
  TEST_ASSERT_FALSE(shouldFastRetryCode(401));
  TEST_ASSERT_FALSE(shouldFastRetryCode(404));
  TEST_ASSERT_FALSE(shouldFastRetryCode(429));
  TEST_ASSERT_FALSE(shouldFastRetryCode(499));
}

void test_no_retry_above_5xx(void) { TEST_ASSERT_FALSE(shouldFastRetryCode(600)); }

// --- runner ----------------------------------------------------------------

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_origin_strips_path);
  RUN_TEST(test_origin_keeps_port);
  RUN_TEST(test_origin_without_path_is_unchanged);
  RUN_TEST(test_origin_without_scheme_is_unchanged);
  RUN_TEST(test_redirect_absolute_http_location_wins);
  RUN_TEST(test_redirect_absolute_https_location_wins);
  RUN_TEST(test_redirect_relative_location_is_appended_to_origin);
  RUN_TEST(test_same_origin_true_for_prefix);
  RUN_TEST(test_same_origin_false_for_other_host);
  RUN_TEST(test_same_origin_false_for_empty_base);
  RUN_TEST(test_sniff_bmp);
  RUN_TEST(test_sniff_png);
  RUN_TEST(test_sniff_jpeg);
  RUN_TEST(test_sniff_garbage_is_null);
  RUN_TEST(test_sniff_too_short_is_null);
  RUN_TEST(test_retry_transport_errors);
  RUN_TEST(test_retry_server_errors);
  RUN_TEST(test_retry_request_timeout);
  RUN_TEST(test_no_retry_success_and_redirects);
  RUN_TEST(test_no_retry_deterministic_client_errors);
  RUN_TEST(test_no_retry_above_5xx);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
