// Single integration test suite. Each <group>_tests.cpp file owns its tests
// internally and exposes one grouping function (test_smoke, test_wifi,
// test_setup). This entry point just orders the groups.
//
// Add a new group: create <group>_tests.cpp with `void test_<group>(void)`,
// declare it in tests.h, and call it from setup() between UNITY_BEGIN/END.

#include <Arduino.h>
#include <unity.h>
#include <Preferences.h>
#include <device_id.h>
#include "test_helpers.h"
#include "tests.h"

extern Preferences preferences; // from bl.cpp

void setUp(void) {}
void tearDown(void) {}

void setup_bl()
{
  preferences.begin("data", false); // copied from bl_init()
}

void setup()
{
  Serial.begin(115200);

  setup_bl();

  UNITY_BEGIN();

  RUN_TEST(connect_to_wifi);

  test_api_setup();
  test_tls_resume();
  // test_api_display();
  // test_wifi(); // must be last in the suite, until production bug is fixed

  UNITY_END();
}

void loop() {}
