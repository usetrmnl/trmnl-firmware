#include <pairing.h>
#include <unity.h>

// Registered devices (status != 202) never show the pairing screen.
void test_pairing_hidden_when_registered(void) {
  TEST_ASSERT_FALSE(shouldShowPairingCode(false, true, "598364", ""));
  TEST_ASSERT_FALSE(shouldShowPairingCode(false, false, "598364", ""));
}

// Power-on / forced refresh always repaints the code.
void test_pairing_shown_on_force_refresh(void) {
  TEST_ASSERT_TRUE(shouldShowPairingCode(true, true, "598364", "598364"));
}

// A device that was showing content (tracker cleared) or whose code changed
// must repaint - this is the "never shows after server-side deletion" fix.
void test_pairing_shown_when_code_changed(void) {
  TEST_ASSERT_TRUE(shouldShowPairingCode(true, false, "598364", ""));
  TEST_ASSERT_TRUE(shouldShowPairingCode(true, false, "598364", "111111"));
}

// Fast-poll of the same code with the code already on screen: no repaint (no thrash).
void test_pairing_deduped_when_same_code(void) {
  TEST_ASSERT_FALSE(shouldShowPairingCode(true, false, "598364", "598364"));
}

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_pairing_hidden_when_registered);
  RUN_TEST(test_pairing_shown_on_force_refresh);
  RUN_TEST(test_pairing_shown_when_code_changed);
  RUN_TEST(test_pairing_deduped_when_same_code);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
