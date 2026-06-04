#include <unity.h>
#include <api-client/request_headers.h>
#include <special_function.h>

// --- helpers ---------------------------------------------------------------

// Index of the first header with the given name, or -1 if not present.
static int indexOf(const HttpHeaderList &headers, const char *name)
{
  for (size_t i = 0; i < headers.size(); ++i)
    if (headers[i].first == name)
      return (int)i;
  return -1;
}

static bool has(const HttpHeaderList &headers, const char *name)
{
  return indexOf(headers, name) >= 0;
}

// Value of the first header with the given name (empty String if absent).
static String valueOf(const HttpHeaderList &headers, const char *name)
{
  int i = indexOf(headers, name);
  return i >= 0 ? headers[i].second : String();
}

static ApiDisplayInputs makeDisplayInputs()
{
  ApiDisplayInputs inputs;
  inputs.baseUrl = "https://example.com";
  inputs.apiKey = "test-token";
  inputs.friendlyId = "ABC123";
  inputs.updateSource = "scheduled";
  inputs.refreshRate = 900;
  inputs.macAddress = "AA:BB:CC:DD:EE:FF";
  inputs.batteryVoltage = 4.10f;
  inputs.firmwareVersion = "1.8.5";
  inputs.model = "og";
  inputs.rssi = -55;
  inputs.displayWidth = 800;
  inputs.displayHeight = 480;
  inputs.specialFunction = SF_NONE;
  inputs.usbConnected = false;
  inputs.imageCached = false;
  inputs.prevWakeTime = 42;
  return inputs;
}

// --- buildSetupHeaders -----------------------------------------------------

void test_setup_headers_names_and_order(void)
{
  ApiSetupInputs inputs;
  inputs.baseUrl = "https://example.com";
  inputs.macAddress = "AA:BB:CC:DD:EE:FF";
  inputs.firmwareVersion = "1.8.5";
  inputs.model = "og";

  auto headers = buildSetupHeaders(inputs);

  TEST_ASSERT_EQUAL_UINT32(4, headers.size());
  TEST_ASSERT_EQUAL_STRING("ID", headers[0].first.c_str());
  TEST_ASSERT_EQUAL_STRING("Content-Type", headers[1].first.c_str());
  TEST_ASSERT_EQUAL_STRING("FW-Version", headers[2].first.c_str());
  TEST_ASSERT_EQUAL_STRING("Model", headers[3].first.c_str());

  TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", valueOf(headers, "ID").c_str());
  TEST_ASSERT_EQUAL_STRING("application/json", valueOf(headers, "Content-Type").c_str());
  TEST_ASSERT_EQUAL_STRING("1.8.5", valueOf(headers, "FW-Version").c_str());
  TEST_ASSERT_EQUAL_STRING("og", valueOf(headers, "Model").c_str());
}

// --- buildLogHeaders -------------------------------------------------------

void test_log_headers_names_and_values(void)
{
  ApiLogInputs inputs;
  inputs.macAddress = "AA:BB:CC:DD:EE:FF";
  inputs.apiKey = "secret-key";

  auto headers = buildLogHeaders(inputs);

  TEST_ASSERT_EQUAL_UINT32(4, headers.size());
  TEST_ASSERT_EQUAL_STRING("ID", headers[0].first.c_str());
  TEST_ASSERT_EQUAL_STRING("Accept", headers[1].first.c_str());
  TEST_ASSERT_EQUAL_STRING("Access-Token", headers[2].first.c_str());
  TEST_ASSERT_EQUAL_STRING("Content-Type", headers[3].first.c_str());

  TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", valueOf(headers, "ID").c_str());
  TEST_ASSERT_EQUAL_STRING("application/json, */*", valueOf(headers, "Accept").c_str());
  TEST_ASSERT_EQUAL_STRING("secret-key", valueOf(headers, "Access-Token").c_str());
  TEST_ASSERT_EQUAL_STRING("application/json", valueOf(headers, "Content-Type").c_str());
}

// --- buildDisplayHeaders ---------------------------------------------------

void test_display_headers_core_values(void)
{
  auto inputs = makeDisplayInputs();
  auto headers = buildDisplayHeaders(inputs);

  TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", valueOf(headers, "ID").c_str());
  TEST_ASSERT_EQUAL_STRING("application/json", valueOf(headers, "Content-Type").c_str());
  TEST_ASSERT_EQUAL_STRING("scheduled", valueOf(headers, "Update-Source").c_str());
  TEST_ASSERT_EQUAL_STRING("test-token", valueOf(headers, "Access-Token").c_str());
  TEST_ASSERT_EQUAL_STRING("900", valueOf(headers, "Refresh-Rate").c_str());
  TEST_ASSERT_TRUE(has(headers, "Battery-Voltage")); // exact float formatting is platform-dependent
  TEST_ASSERT_EQUAL_STRING("1.8.5", valueOf(headers, "FW-Version").c_str());
  TEST_ASSERT_EQUAL_STRING("og", valueOf(headers, "Model").c_str());
  TEST_ASSERT_EQUAL_STRING("false", valueOf(headers, "Image-Cached").c_str());
  TEST_ASSERT_EQUAL_STRING("42", valueOf(headers, "Wake-Time").c_str());
  TEST_ASSERT_EQUAL_STRING("-55", valueOf(headers, "RSSI").c_str());
  TEST_ASSERT_EQUAL_STRING("true", valueOf(headers, "Temperature-Profile").c_str());
  TEST_ASSERT_EQUAL_STRING("800", valueOf(headers, "Width").c_str());
  TEST_ASSERT_EQUAL_STRING("480", valueOf(headers, "Height").c_str());
}

// Update-Source and Temperature-Profile must always be present — these were the
// two headers historically missing from the modem (5 GHz) display path.
void test_display_headers_include_update_source_and_temperature_profile(void)
{
  auto headers = buildDisplayHeaders(makeDisplayInputs());
  TEST_ASSERT_TRUE(has(headers, "Update-Source"));
  TEST_ASSERT_TRUE(has(headers, "Temperature-Profile"));
}

void test_display_headers_image_cached_reflects_input(void)
{
  auto inputs = makeDisplayInputs();
  inputs.imageCached = true;
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_EQUAL_STRING("true", valueOf(headers, "Image-Cached").c_str());
}

void test_display_headers_special_function_omitted_when_none(void)
{
  auto inputs = makeDisplayInputs();
  inputs.specialFunction = SF_NONE;
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_FALSE(has(headers, "special_function"));
}

void test_display_headers_special_function_present_when_set(void)
{
  auto inputs = makeDisplayInputs();
  inputs.specialFunction = SF_IDENTIFY;
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_TRUE(has(headers, "special_function"));
  TEST_ASSERT_EQUAL_STRING("true", valueOf(headers, "special_function").c_str());
}

void test_display_headers_wifi_band_2_4(void)
{
  auto inputs = makeDisplayInputs();
  inputs.wifiBand = "2.4";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_EQUAL_STRING("2.4", valueOf(headers, "WiFi-Band").c_str());
}

void test_display_headers_wifi_band_5(void)
{
  auto inputs = makeDisplayInputs();
  inputs.wifiBand = "5";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_EQUAL_STRING("5", valueOf(headers, "WiFi-Band").c_str());
}

void test_display_headers_wifi_band_omitted_when_empty(void)
{
  auto inputs = makeDisplayInputs();
  inputs.wifiBand = "";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_FALSE(has(headers, "WiFi-Band"));
}

void test_display_headers_wifi_ssid_present(void)
{
  auto inputs = makeDisplayInputs();
  inputs.wifiSSID = "MyNetwork";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_EQUAL_STRING("MyNetwork", valueOf(headers, "WiFi-SSID").c_str());
}

void test_display_headers_wifi_ssid_percent_encoded(void)
{
  auto inputs = makeDisplayInputs();
  inputs.wifiSSID = "My Wi-Fi: café";
  auto headers = buildDisplayHeaders(inputs);
  // space, ':' and the UTF-8 bytes of 'é' (0xC3 0xA9) are escaped
  TEST_ASSERT_EQUAL_STRING("My%20Wi-Fi%3A%20caf%C3%A9", valueOf(headers, "WiFi-SSID").c_str());
}

void test_display_headers_wifi_ssid_omitted_when_empty(void)
{
  auto inputs = makeDisplayInputs();
  inputs.wifiSSID = "";
  auto headers = buildDisplayHeaders(inputs);
  TEST_ASSERT_FALSE(has(headers, "WiFi-SSID"));
}

// --- formatHeaders ---------------------------------------------------------

void test_format_headers_newline_separated_no_trailing_newline(void)
{
  HttpHeaderList headers;
  headers.push_back({"ID", "abc"});
  headers.push_back({"Content-Type", "application/json"});

  String formatted = formatHeaders(headers);

  TEST_ASSERT_EQUAL_STRING("ID: abc\nContent-Type: application/json", formatted.c_str());
}

void test_format_headers_empty_list_is_empty_string(void)
{
  HttpHeaderList headers;
  TEST_ASSERT_EQUAL_STRING("", formatHeaders(headers).c_str());
}

// --- percentEncode ---------------------------------------------------------

void test_percent_encode_passes_through_unreserved(void)
{
  TEST_ASSERT_EQUAL_STRING("abcXYZ09-._~", percentEncode("abcXYZ09-._~").c_str());
}

void test_percent_encode_escapes_reserved_and_control(void)
{
  // space, ':', '\n', '%' must all be escaped (uppercase hex)
  TEST_ASSERT_EQUAL_STRING("a%20b%3A%0A%25", percentEncode("a b:\n%").c_str());
}

void test_percent_encode_empty_is_empty(void)
{
  TEST_ASSERT_EQUAL_STRING("", percentEncode("").c_str());
}

void test_format_setup_headers_round_trip(void)
{
  ApiSetupInputs inputs;
  inputs.macAddress = "MAC";
  inputs.firmwareVersion = "9.9.9";
  inputs.model = "x";
  // baseUrl intentionally unused by the header builder

  String formatted = formatHeaders(buildSetupHeaders(inputs));
  TEST_ASSERT_EQUAL_STRING(
      "ID: MAC\nContent-Type: application/json\nFW-Version: 9.9.9\nModel: x",
      formatted.c_str());
}

// --- runner ----------------------------------------------------------------

void setUp(void) {}
void tearDown(void) {}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_setup_headers_names_and_order);
  RUN_TEST(test_log_headers_names_and_values);
  RUN_TEST(test_display_headers_core_values);
  RUN_TEST(test_display_headers_include_update_source_and_temperature_profile);
  RUN_TEST(test_display_headers_image_cached_reflects_input);
  RUN_TEST(test_display_headers_special_function_omitted_when_none);
  RUN_TEST(test_display_headers_special_function_present_when_set);
  RUN_TEST(test_display_headers_wifi_band_2_4);
  RUN_TEST(test_display_headers_wifi_band_5);
  RUN_TEST(test_display_headers_wifi_band_omitted_when_empty);
  RUN_TEST(test_display_headers_wifi_ssid_present);
  RUN_TEST(test_display_headers_wifi_ssid_percent_encoded);
  RUN_TEST(test_display_headers_wifi_ssid_omitted_when_empty);
  RUN_TEST(test_format_headers_newline_separated_no_trailing_newline);
  RUN_TEST(test_format_headers_empty_list_is_empty_string);
  RUN_TEST(test_percent_encode_passes_through_unreserved);
  RUN_TEST(test_percent_encode_escapes_reserved_and_control);
  RUN_TEST(test_percent_encode_empty_is_empty);
  RUN_TEST(test_format_setup_headers_round_trip);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}
