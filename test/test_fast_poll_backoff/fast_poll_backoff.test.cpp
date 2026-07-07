#include <unity.h>
#include <fast_poll_backoff.h>
#include <unordered_map>
#include <string>

class MemoryPersistence : public Persistence
{
public:
  bool recordExists(const char *key) override { return storage.count(key) > 0; }
  String readString(const char *key, const String defaultValue) override { return defaultValue; }
  uint32_t readUint(const char *key, const uint32_t defaultValue) override
  {
    return recordExists(key) ? (uint32_t)std::stoul(storage[key]) : defaultValue;
  }
  size_t writeUint(const char *key, const uint32_t value) override
  {
    storage[key] = std::to_string(value);
    return sizeof(value);
  }
  size_t writeString(const char *key, const char *value) override { return 0; }
  uint8_t readUChar(const char *key, const uint8_t defaultValue) override { return defaultValue; }
  size_t writeUChar(const char *key, const uint8_t value) override { return 0; }
  bool readBool(const char *key, const bool defaultValue) override { return defaultValue; }
  size_t writeBool(const char *key, const bool value) override { return 0; }
  bool clear() override { storage.clear(); return true; }
  bool remove(const char *key) override { storage.erase(key); return true; }

private:
  std::unordered_map<std::string, std::string> storage;
};

void test_ladder_keeps_setup_window_at_five_seconds(void)
{
  TEST_ASSERT_EQUAL_UINT32(5, fastPollSleepSeconds(1));
  TEST_ASSERT_EQUAL_UINT32(5, fastPollSleepSeconds(FAST_POLL_SNAPPY_POLLS));
}

void test_ladder_backs_off_past_the_setup_window(void)
{
  TEST_ASSERT_EQUAL_UINT32(60, fastPollSleepSeconds(FAST_POLL_SNAPPY_POLLS + 1));
  TEST_ASSERT_EQUAL_UINT32(60, fastPollSleepSeconds(60));
  TEST_ASSERT_EQUAL_UINT32(900, fastPollSleepSeconds(61));
  TEST_ASSERT_EQUAL_UINT32(900, fastPollSleepSeconds(70));
}

void test_ladder_caps_at_one_hour(void)
{
  TEST_ASSERT_EQUAL_UINT32(3600, fastPollSleepSeconds(71));
  TEST_ASSERT_EQUAL_UINT32(3600, fastPollSleepSeconds(1000000));
}

void test_next_sleep_advances_the_persisted_streak(void)
{
  MemoryPersistence persistence;
  TEST_ASSERT_EQUAL_UINT32(5, fastPollNextSleep(persistence));
  TEST_ASSERT_EQUAL_UINT32(1, persistence.readUint(FAST_POLL_STREAK_KEY, 0));
  for (int i = 0; i < FAST_POLL_SNAPPY_POLLS; i++)
    fastPollNextSleep(persistence);
  TEST_ASSERT_EQUAL_UINT32(60, fastPollNextSleep(persistence));
}

void test_reset_returns_the_ladder_to_five_seconds(void)
{
  MemoryPersistence persistence;
  for (int i = 0; i < 100; i++)
    fastPollNextSleep(persistence);
  fastPollStreakReset(persistence);
  TEST_ASSERT_EQUAL_UINT32(0, persistence.readUint(FAST_POLL_STREAK_KEY, 0));
  TEST_ASSERT_EQUAL_UINT32(5, fastPollNextSleep(persistence));
}

// A device stuck on a never-resolving "keep polling" response (abandoned setup,
// empty playlist) wakes ~6,600x/day on a flat 5s sleep; ~8s awake overhead per
// wake (boot + WiFi + TLS) matches the ~13s cadence observed on OG hardware.
void test_daily_wake_budget_is_bounded(void)
{
  const uint32_t awake_overhead_s = 8;
  const uint32_t day_s = 24 * 3600;

  uint32_t wakes_flat = 0;
  for (uint32_t t = 0; t < day_s; t += awake_overhead_s + FAST_POLL_SLEEP_SECONDS)
    wakes_flat++;

  MemoryPersistence persistence;
  uint32_t wakes_ladder = 0;
  for (uint32_t t = 0; t < day_s; t += awake_overhead_s + fastPollNextSleep(persistence))
    wakes_ladder++;

  TEST_ASSERT_GREATER_THAN_UINT32(6000, wakes_flat);
  TEST_ASSERT_LESS_THAN_UINT32(120, wakes_ladder);
}

void setUp(void) {}
void tearDown(void) {}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_ladder_keeps_setup_window_at_five_seconds);
  RUN_TEST(test_ladder_backs_off_past_the_setup_window);
  RUN_TEST(test_ladder_caps_at_one_hour);
  RUN_TEST(test_next_sleep_advances_the_persisted_streak);
  RUN_TEST(test_reset_returns_the_ladder_to_five_seconds);
  RUN_TEST(test_daily_wake_budget_is_bounded);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}
