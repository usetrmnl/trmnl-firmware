#include <unity.h>
#include <string_utils.h>
#include <cstring>
#include <cstdarg>
#include <cstdio>

// Helper to test format_message_truncated with variadic args
void test_format_helper(char* buffer, int max_size, const char* format, ...) {
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
    UNITY_END();
}

int main(int argc, char **argv) {
    process();
    return 0;
}