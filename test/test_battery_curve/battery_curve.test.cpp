#include <battery_curve.h>
#include <unity.h>

void test_curve_points(void) {
  TEST_ASSERT_EQUAL_INT(1, estimate_soc_from_voltage(3.20f));
  TEST_ASSERT_EQUAL_INT(5, estimate_soc_from_voltage(3.60f));
  TEST_ASSERT_EQUAL_INT(14, estimate_soc_from_voltage(3.70f));
  TEST_ASSERT_EQUAL_INT(38, estimate_soc_from_voltage(3.80f));
  TEST_ASSERT_EQUAL_INT(61, estimate_soc_from_voltage(3.90f));
  TEST_ASSERT_EQUAL_INT(74, estimate_soc_from_voltage(4.00f));
  TEST_ASSERT_EQUAL_INT(95, estimate_soc_from_voltage(4.20f));
}

void test_interpolates_between_points(void) {
  TEST_ASSERT_EQUAL_INT(6, estimate_soc_from_voltage(3.625f));
  TEST_ASSERT_EQUAL_INT(81, estimate_soc_from_voltage(4.06f));
  TEST_ASSERT_EQUAL_INT(90, estimate_soc_from_voltage(4.15f));
}

void test_clamps_below_empty(void) {
  TEST_ASSERT_EQUAL_INT(0, estimate_soc_from_voltage(3.00f));
  TEST_ASSERT_EQUAL_INT(0, estimate_soc_from_voltage(2.50f));
  TEST_ASSERT_EQUAL_INT(0, estimate_soc_from_voltage(-1.0f));
}

void test_clamps_above_full(void) {
  TEST_ASSERT_EQUAL_INT(100, estimate_soc_from_voltage(4.25f));
  TEST_ASSERT_EQUAL_INT(100, estimate_soc_from_voltage(4.26f));
  TEST_ASSERT_EQUAL_INT(100, estimate_soc_from_voltage(5.00f));
}

void test_never_rises_as_voltage_falls(void) {
  int previous = 100;
  for (int millivolts = 4300; millivolts >= 2500; millivolts -= 5) {
    int percent = estimate_soc_from_voltage(millivolts / 1000.0f);
    TEST_ASSERT_LESS_OR_EQUAL_INT(previous, percent);
    previous = percent;
  }
}

void setUp(void) {}

void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_curve_points);
  RUN_TEST(test_interpolates_between_points);
  RUN_TEST(test_clamps_below_empty);
  RUN_TEST(test_clamps_above_full);
  RUN_TEST(test_never_rises_as_voltage_falls);
  return UNITY_END();
}
