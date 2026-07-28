#include <fast_poll.h>
#include <string>
#include <unity.h>
#include <unordered_map>

class MemoryPersistence : public Persistence {
public:
  bool recordExists(const char *key) override { return storage.count(key) > 0; }
  String readString(const char *key, const String defaultValue) override { return defaultValue; }
  uint32_t readUint(const char *key, const uint32_t defaultValue) override {
    return recordExists(key) ? (uint32_t)std::stoul(storage[key]) : defaultValue;
  }
  size_t writeUint(const char *key, const uint32_t value) override {
    storage[key] = std::to_string(value);
    return sizeof(value);
  }
  size_t writeString(const char *key, const char *value) override { return 0; }
  uint8_t readUChar(const char *key, const uint8_t defaultValue) override { return defaultValue; }
  size_t writeUChar(const char *key, const uint8_t value) override { return 0; }
  bool readBool(const char *key, const bool defaultValue) override { return defaultValue; }
  size_t writeBool(const char *key, const bool value) override { return 0; }
  bool clear() override {
    storage.clear();
    return true;
  }
  bool remove(const char *key) override {
    storage.erase(key);
    return true;
  }

private:
  std::unordered_map<std::string, std::string> storage;
};

// Seed the persisted streak so the next poll is the streak-th one, then return
// the sleep nextSleep() picks for it.
uint32_t sleepAtStreak(uint32_t streak) {
  MemoryPersistence persistence;
  FastPoll fastPoll(persistence);
  persistence.writeUint(FastPoll::STREAK_KEY, streak - 1);
  return fastPoll.nextSleep();
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

void test_next_sleep_advances_the_persisted_streak(void) {
  MemoryPersistence persistence;
  FastPoll fastPoll(persistence);
  TEST_ASSERT_EQUAL_UINT32(5, fastPoll.nextSleep());
  TEST_ASSERT_EQUAL_UINT32(1, persistence.readUint(FastPoll::STREAK_KEY, 0));
  for (int i = 0; i < 50; i++)
    fastPoll.nextSleep();
  TEST_ASSERT_EQUAL_UINT32(60, fastPoll.nextSleep());
}

void test_reset_returns_the_ladder_to_five_seconds(void) {
  MemoryPersistence persistence;
  FastPoll fastPoll(persistence);
  for (int i = 0; i < 100; i++)
    fastPoll.nextSleep();
  fastPoll.reset();
  TEST_ASSERT_EQUAL_UINT32(0, persistence.readUint(FastPoll::STREAK_KEY, 0));
  TEST_ASSERT_EQUAL_UINT32(5, fastPoll.nextSleep());
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
  FastPoll fastPoll(persistence);
  uint32_t wakes_ladder = 0;
  for (uint32_t t = 0; t < day_s; t += awake_overhead_s + fastPoll.nextSleep())
    wakes_ladder++;

  TEST_ASSERT_GREATER_THAN_UINT32(6000, wakes_flat);
  TEST_ASSERT_LESS_THAN_UINT32(120, wakes_ladder);
}

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_ladder_keeps_setup_window_at_five_seconds);
  RUN_TEST(test_ladder_backs_off_past_the_setup_window);
  RUN_TEST(test_ladder_caps_at_one_hour);
  RUN_TEST(test_next_sleep_advances_the_persisted_streak);
  RUN_TEST(test_reset_returns_the_ladder_to_five_seconds);
  RUN_TEST(test_daily_wake_budget_is_bounded);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
