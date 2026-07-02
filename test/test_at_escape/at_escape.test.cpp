#include <unity.h>
#include <at_escape.h>

void test_plain_string_unchanged(void) {
    TEST_ASSERT_EQUAL_STRING("MyNetwork", escapeAtParam("MyNetwork").c_str());
}

void test_empty_string(void) {
    TEST_ASSERT_EQUAL_STRING("", escapeAtParam("").c_str());
}

void test_comma_escaped(void) {
    TEST_ASSERT_EQUAL_STRING("pass\\,word", escapeAtParam("pass,word").c_str());
}

void test_quote_escaped(void) {
    TEST_ASSERT_EQUAL_STRING("say \\\"hi\\\"", escapeAtParam("say \"hi\"").c_str());
}

void test_backslash_escaped(void) {
    TEST_ASSERT_EQUAL_STRING("a\\\\b", escapeAtParam("a\\b").c_str());
}

void test_backslash_escaped_before_comma(void) {
    // Input contains a literal backslash followed by a comma: a\,b
    // Backslash must be doubled first, then the comma escaped: a\\\,b
    TEST_ASSERT_EQUAL_STRING("a\\\\\\,b", escapeAtParam("a\\,b").c_str());
}

void test_all_special_chars_combined(void) {
    // Input: c:\dir,"x" -> c:\\dir\,\"x\"
    TEST_ASSERT_EQUAL_STRING("c:\\\\dir\\,\\\"x\\\"", escapeAtParam("c:\\dir,\"x\"").c_str());
}

void test_multiple_commas(void) {
    TEST_ASSERT_EQUAL_STRING("a\\,b\\,c", escapeAtParam("a,b,c").c_str());
}

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void process() {
    UNITY_BEGIN();
    RUN_TEST(test_plain_string_unchanged);
    RUN_TEST(test_empty_string);
    RUN_TEST(test_comma_escaped);
    RUN_TEST(test_quote_escaped);
    RUN_TEST(test_backslash_escaped);
    RUN_TEST(test_backslash_escaped_before_comma);
    RUN_TEST(test_all_special_chars_combined);
    RUN_TEST(test_multiple_commas);
    UNITY_END();
}

int main(int argc, char **argv) {
    process();
    return 0;
}
