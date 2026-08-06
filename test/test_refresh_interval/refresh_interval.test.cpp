#include <memory_persistence.h>
#include <refresh_interval.h>
#include <unity.h>

// Devices in the field already store these keys; renaming them would orphan
// the persisted values on upgrade.
void test_nvs_keys_are_stable(void) {
  TEST_ASSERT_EQUAL_STRING("refresh_rate", RefreshInterval::SLEEP_KEY);
  TEST_ASSERT_EQUAL_STRING("fast_polls", RefreshInterval::STREAK_KEY);
}

// Seed the persisted streak so the next poll is the streak-th one, then return
// the sleep applyFastPoll() picks for it.
uint32_t sleepAtStreak(uint32_t streak) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  persistence.writeUint(RefreshInterval::STREAK_KEY, streak - 1);
  return refreshInterval.applyFastPoll();
}

void test_ladder_keeps_setup_window_at_five_seconds(void) {
  TEST_ASSERT_EQUAL_UINT32(5, sleepAtStreak(1));
  TEST_ASSERT_EQUAL_UINT32(5, sleepAtStreak(50));
}

void test_ladder_backs_off_past_the_setup_window(void) {
  TEST_ASSERT_EQUAL_UINT32(60, sleepAtStreak(51));
  TEST_ASSERT_EQUAL_UINT32(60, sleepAtStreak(60));
  TEST_ASSERT_EQUAL_UINT32(900, sleepAtStreak(61));
  TEST_ASSERT_EQUAL_UINT32(900, sleepAtStreak(70));
}

void test_ladder_caps_at_one_hour(void) {
  TEST_ASSERT_EQUAL_UINT32(3600, sleepAtStreak(71));
  TEST_ASSERT_EQUAL_UINT32(3600, sleepAtStreak(1000000));
}

void test_fast_poll_advances_the_streak_and_stores_the_interval(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  TEST_ASSERT_EQUAL_UINT32(5, refreshInterval.applyFastPoll());
  TEST_ASSERT_EQUAL_UINT32(1, persistence.readUint(RefreshInterval::STREAK_KEY, 0));
  TEST_ASSERT_EQUAL_UINT32(5, persistence.readUint(RefreshInterval::SLEEP_KEY, 0));
  for (int i = 0; i < 50; i++)
    refreshInterval.applyFastPoll();
  TEST_ASSERT_EQUAL_UINT32(60, refreshInterval.applyFastPoll());
  TEST_ASSERT_EQUAL_UINT32(60, refreshInterval.seconds());
}

void test_reset_returns_the_ladder_to_five_seconds(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  for (int i = 0; i < 100; i++)
    refreshInterval.applyFastPoll();
  refreshInterval.resetFastPollStreak();
  TEST_ASSERT_EQUAL_UINT32(0, persistence.readUint(RefreshInterval::STREAK_KEY, 0));
  TEST_ASSERT_EQUAL_UINT32(5, refreshInterval.applyFastPoll());
}

// A device stuck on a never-resolving "keep polling" response (abandoned setup,
// empty playlist) wakes ~6,600x/day on a flat 5s sleep; ~8s awake overhead per
// wake (boot + WiFi + TLS) matches the ~13s cadence observed on OG hardware.
void test_daily_wake_budget_is_bounded(void) {
  const uint32_t awake_overhead_s = 8;
  const uint32_t day_s = 24 * 3600;

  uint32_t wakes_flat = 0;
  for (uint32_t t = 0; t < day_s; t += awake_overhead_s + 5)
    wakes_flat++;

  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  uint32_t wakes_ladder = 0;
  for (uint32_t t = 0; t < day_s; t += awake_overhead_s + refreshInterval.applyFastPoll())
    wakes_ladder++;

  TEST_ASSERT_GREATER_THAN_UINT32(6000, wakes_flat);
  TEST_ASSERT_LESS_THAN_UINT32(120, wakes_ladder);
}

void test_server_rate_is_stored(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  TEST_ASSERT_EQUAL_UINT32(300, refreshInterval.applyServerRate(300));
  TEST_ASSERT_EQUAL_UINT32(300, refreshInterval.seconds());
}

void test_server_rate_skips_the_write_when_unchanged(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  refreshInterval.applyServerRate(300);
  uint32_t writes = persistence.writeCount;
  TEST_ASSERT_EQUAL_UINT32(300, refreshInterval.applyServerRate(300));
  TEST_ASSERT_EQUAL_UINT32(writes, persistence.writeCount);
}

void test_server_rate_writes_when_key_is_absent_even_at_the_default(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  refreshInterval.applyServerRate(RefreshInterval::DEFAULT_SECONDS);
  TEST_ASSERT_TRUE(persistence.recordExists(RefreshInterval::SLEEP_KEY));
}

void test_api_retry_ladder(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  TEST_ASSERT_EQUAL_UINT32(15, refreshInterval.applyApiRetry(1));
  TEST_ASSERT_EQUAL_UINT32(30, refreshInterval.applyApiRetry(2));
  TEST_ASSERT_EQUAL_UINT32(60, refreshInterval.applyApiRetry(3));
  TEST_ASSERT_EQUAL_UINT32(900, refreshInterval.applyApiRetry(4));
  TEST_ASSERT_EQUAL_UINT32(900, refreshInterval.seconds());
}

void test_wifi_retry_ladder(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  TEST_ASSERT_EQUAL_UINT32(60, refreshInterval.applyWifiRetry(1));
  TEST_ASSERT_EQUAL_UINT32(180, refreshInterval.applyWifiRetry(2));
  TEST_ASSERT_EQUAL_UINT32(300, refreshInterval.applyWifiRetry(3));
  TEST_ASSERT_EQUAL_UINT32(300, refreshInterval.seconds());
}

void test_default_fallback_is_fifteen_minutes(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  TEST_ASSERT_EQUAL_UINT32(900, refreshInterval.applyDefault());
  TEST_ASSERT_EQUAL_UINT32(900, refreshInterval.seconds());
}

void test_seconds_defaults(void) {
  MemoryPersistence persistence;
  RefreshInterval refreshInterval(persistence);
  TEST_ASSERT_EQUAL_UINT32(900, refreshInterval.seconds());
  TEST_ASSERT_EQUAL_UINT32(0, refreshInterval.seconds(0)); // telemetry contract
  refreshInterval.applyServerRate(120);
  TEST_ASSERT_EQUAL_UINT32(120, refreshInterval.seconds());
  TEST_ASSERT_EQUAL_UINT32(120, refreshInterval.seconds(0));
}

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_nvs_keys_are_stable);
  RUN_TEST(test_ladder_keeps_setup_window_at_five_seconds);
  RUN_TEST(test_ladder_backs_off_past_the_setup_window);
  RUN_TEST(test_ladder_caps_at_one_hour);
  RUN_TEST(test_fast_poll_advances_the_streak_and_stores_the_interval);
  RUN_TEST(test_reset_returns_the_ladder_to_five_seconds);
  RUN_TEST(test_daily_wake_budget_is_bounded);
  RUN_TEST(test_server_rate_is_stored);
  RUN_TEST(test_server_rate_skips_the_write_when_unchanged);
  RUN_TEST(test_server_rate_writes_when_key_is_absent_even_at_the_default);
  RUN_TEST(test_api_retry_ladder);
  RUN_TEST(test_wifi_retry_ladder);
  RUN_TEST(test_default_fallback_is_fifteen_minutes);
  RUN_TEST(test_seconds_defaults);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
