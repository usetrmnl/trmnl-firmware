#include <button_wakeup.h>
#include <unity.h>

// Pin assignment of the reTerminal E1001. The helper takes the pins as
// arguments, so these are just representative values.
static const int PIN_MAIN = 3;
static const int BUTTON_A = 4;
static const int BUTTON_B = 5;

static const char *sourceFor(uint64_t bits) { return updateSourceForWakeBits(bits, PIN_MAIN, BUTTON_A, BUTTON_B); }

// --- one button at a time --------------------------------------------------

void test_main_pin_reports_button(void) { TEST_ASSERT_EQUAL_STRING("button", sourceFor(1ULL << PIN_MAIN)); }

void test_gpio4_reports_button_a(void) { TEST_ASSERT_EQUAL_STRING("button_a", sourceFor(1ULL << BUTTON_A)); }

void test_gpio5_reports_button_b(void) { TEST_ASSERT_EQUAL_STRING("button_b", sourceFor(1ULL << BUTTON_B)); }

// --- masks that name no button, or more than one ---------------------------

void test_empty_mask_is_unmapped(void) { TEST_ASSERT_NULL(sourceFor(0)); }

void test_unrelated_pin_is_unmapped(void) { TEST_ASSERT_NULL(sourceFor(1ULL << 21)); }

void test_main_pin_wins_when_several_are_held(void) {
  // PIN_MAIN takes precedence over the other two.
  TEST_ASSERT_EQUAL_STRING("button", sourceFor((1ULL << PIN_MAIN) | (1ULL << BUTTON_A) | (1ULL << BUTTON_B)));
}

void test_extra_buttons_do_not_mask_each_other(void) {
  TEST_ASSERT_EQUAL_STRING("button_a", sourceFor((1ULL << BUTTON_A) | (1ULL << BUTTON_B)));
}

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_main_pin_reports_button);
  RUN_TEST(test_gpio4_reports_button_a);
  RUN_TEST(test_gpio5_reports_button_b);
  RUN_TEST(test_empty_mask_is_unmapped);
  RUN_TEST(test_unrelated_pin_is_unmapped);
  RUN_TEST(test_main_pin_wins_when_several_are_held);
  RUN_TEST(test_extra_buttons_do_not_mask_each_other);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
