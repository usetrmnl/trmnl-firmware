#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string_utils.h>
#include <unity.h>

// Helper to test format_message_truncated with variadic args
void test_format_helper(char *buffer, int max_size, const char *format, ...) {
  va_list args;
  va_start(args, format);
  format_message_truncated(buffer, max_size, format, args);
  va_end(args);
}

void test_format_message_no_truncation(void) {
  char buffer[100];

  test_format_helper(buffer, sizeof(buffer), "Hello %s", "World");

  TEST_ASSERT_EQUAL_STRING("Hello World", buffer);
}

void test_format_message_with_truncation(void) {
  char buffer[10];

  test_format_helper(buffer, sizeof(buffer), "This is a very long message");

  TEST_ASSERT_EQUAL_STRING("This i...", buffer);
}

void test_format_message_exact_fit(void) {
  char buffer[12];

  test_format_helper(buffer, sizeof(buffer), "Hello World");

  TEST_ASSERT_EQUAL_STRING("Hello World", buffer);
}

void test_format_message_truncation_boundary(void) {
  char buffer[8];

  test_format_helper(buffer, sizeof(buffer), "1234567890");

  TEST_ASSERT_EQUAL_STRING("1234...", buffer);
}

void test_escape_plain_string_unchanged(void) {
  TEST_ASSERT_EQUAL_STRING("MyNetwork", escape_modem_param("MyNetwork").c_str());
}

void test_escape_empty_string(void) { TEST_ASSERT_EQUAL_STRING("", escape_modem_param("").c_str()); }

void test_escape_comma(void) { TEST_ASSERT_EQUAL_STRING("pass\\,word", escape_modem_param("pass,word").c_str()); }

void test_escape_quote(void) { TEST_ASSERT_EQUAL_STRING("say \\\"hi\\\"", escape_modem_param("say \"hi\"").c_str()); }

void test_escape_backslash(void) { TEST_ASSERT_EQUAL_STRING("a\\\\b", escape_modem_param("a\\b").c_str()); }

void test_escape_backslash_before_comma(void) {
    // Input contains a literal backslash followed by a comma: a\,b
    // Backslash must be doubled first, then the comma escaped: a\\\,b
  TEST_ASSERT_EQUAL_STRING("a\\\\\\,b", escape_modem_param("a\\,b").c_str());
}

void test_escape_all_special_chars_combined(void) {
    // Input: c:\dir,"x" -> c:\\dir\,\"x\"
  TEST_ASSERT_EQUAL_STRING("c:\\\\dir\\,\\\"x\\\"", escape_modem_param("c:\\dir,\"x\"").c_str());
}

void test_escape_multiple_commas(void) { TEST_ASSERT_EQUAL_STRING("a\\,b\\,c", escape_modem_param("a,b,c").c_str()); }

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_format_message_no_truncation);
  RUN_TEST(test_format_message_with_truncation);
  RUN_TEST(test_format_message_exact_fit);
  RUN_TEST(test_format_message_truncation_boundary);
  RUN_TEST(test_escape_plain_string_unchanged);
  RUN_TEST(test_escape_empty_string);
  RUN_TEST(test_escape_comma);
  RUN_TEST(test_escape_quote);
  RUN_TEST(test_escape_backslash);
  RUN_TEST(test_escape_backslash_before_comma);
  RUN_TEST(test_escape_all_special_chars_combined);
  RUN_TEST(test_escape_multiple_commas);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}