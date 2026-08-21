#include <battery_soc.h>
#include <unity.h>

// Chain the two calls the way gaugeInit() does.
static BypassCapacity capacityAt(int cellCount, float voltage) {
  return bypassCapacity(cellCount, batteryVoltageToPercent(voltage));
}

void test_percent_follows_the_server_curve(void) {
  TEST_ASSERT_EQUAL_INT(25, batteryVoltageToPercent(3.30f));
  TEST_ASSERT_EQUAL_INT(42, batteryVoltageToPercent(3.50f));
  TEST_ASSERT_EQUAL_INT(67, batteryVoltageToPercent(3.80f));
  TEST_ASSERT_EQUAL_INT(78, batteryVoltageToPercent(3.94f));
}

// Check both sides of every branch boundary.
void test_plateau_boundaries(void) {
  // Float rounding puts this edge one millivolt high.
  TEST_ASSERT_EQUAL_INT(1, batteryVoltageToPercent(3.120f));
  TEST_ASSERT_EQUAL_INT(10, batteryVoltageToPercent(3.121f));

  TEST_ASSERT_EQUAL_INT(83, batteryVoltageToPercent(3.995f));
  TEST_ASSERT_EQUAL_INT(90, batteryVoltageToPercent(3.996f));

  TEST_ASSERT_EQUAL_INT(90, batteryVoltageToPercent(4.019f));
  TEST_ASSERT_EQUAL_INT(95, batteryVoltageToPercent(4.020f));

  TEST_ASSERT_EQUAL_INT(95, batteryVoltageToPercent(4.055f));
  TEST_ASSERT_EQUAL_INT(100, batteryVoltageToPercent(4.056f));
}

// A percentage outside 0-100 would be rejected by the server.
void test_percent_stays_in_range(void) {
  TEST_ASSERT_EQUAL_INT(1, batteryVoltageToPercent(0.0f));
  TEST_ASSERT_EQUAL_INT(1, batteryVoltageToPercent(2.50f));
  TEST_ASSERT_EQUAL_INT(100, batteryVoltageToPercent(4.20f));
  TEST_ASSERT_EQUAL_INT(100, batteryVoltageToPercent(5.00f));

  // A negative voltage must not give a negative percentage.
  TEST_ASSERT_EQUAL_INT(1, batteryVoltageToPercent(-1.0f));
}

// The curve must never report a rising charge for a falling voltage.
void test_percent_never_rises_as_voltage_falls(void) {
  int previous = 101;
  for (int millivolts = 4300; millivolts >= 2500; millivolts -= 1) {
    int percent = batteryVoltageToPercent(millivolts / 1000.0f);
    TEST_ASSERT_TRUE(percent <= previous);
    previous = percent;
  }
}

void test_bypass_capacity_scales_with_cell_count(void) {
  TEST_ASSERT_EQUAL_INT(6000, bypassCapacity(1, 100).full);
  TEST_ASSERT_EQUAL_INT(12000, bypassCapacity(2, 100).full);

  // The reading that started this.
  TEST_ASSERT_EQUAL_INT(12000, capacityAt(2, 3.94f).full);
  TEST_ASSERT_EQUAL_INT(9360, capacityAt(2, 3.94f).remain);
  TEST_ASSERT_EQUAL_INT(4680, capacityAt(1, 3.94f).remain);

  // A full pack reports its whole capacity, never more.
  TEST_ASSERT_EQUAL_INT(12000, capacityAt(2, 4.20f).remain);
}

void setUp(void) {}

void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_percent_follows_the_server_curve);
  RUN_TEST(test_plateau_boundaries);
  RUN_TEST(test_percent_stays_in_range);
  RUN_TEST(test_percent_never_rises_as_voltage_falls);
  RUN_TEST(test_bypass_capacity_scales_with_cell_count);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
