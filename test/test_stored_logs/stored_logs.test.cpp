#include <unity.h>
#include <stored_logs.h>
#include <unordered_map>
#include <string>
#include "memory_persistence.h"

MemoryPersistence persistence;
StoredLogs subject(3, "log_", "log_head", persistence);

void test_stores_string(void)
{
  TEST_ASSERT_EQUAL(0, persistence.size());
  subject.store_log("asdf");
  TEST_ASSERT_EQUAL(1, persistence.size());
  String s = subject.gather_stored_logs();
  TEST_ASSERT_EQUAL_STRING("asdf", s.c_str());
  subject.clear_stored_logs();
  TEST_ASSERT_EQUAL(0, persistence.size());
}

void test_stores_several_strings()
{
  TEST_ASSERT_EQUAL(0, persistence.size());
  subject.store_log("asdf");
  subject.store_log("qwer");
  subject.store_log("zxcv");
  TEST_ASSERT_EQUAL(3, persistence.size());
  String s = subject.gather_stored_logs();
  TEST_ASSERT_EQUAL_STRING("asdf,qwer,zxcv", s.c_str());
  subject.clear_stored_logs();
  TEST_ASSERT_EQUAL(0, persistence.size());
}

void test_circular_buffer_overwrites_oldest()
{
  TEST_ASSERT_EQUAL(0, persistence.size());

  subject.store_log("log1");
  subject.store_log("log2");
  subject.store_log("log3");

  TEST_ASSERT_EQUAL(3, persistence.size());

  subject.store_log("log4");

  // Should now be 4 total (3 logs + head pointer)
  TEST_ASSERT_EQUAL(4, persistence.size());

  String s = subject.gather_stored_logs();

  TEST_ASSERT(strstr(s.c_str(), "log4,") != NULL);
  TEST_ASSERT(strstr(s.c_str(), "log1,") == NULL); // log1 should be gone

  subject.clear_stored_logs();

  TEST_ASSERT_EQUAL(1, persistence.size());
}

void setUp(void) {}

void tearDown(void) {}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_stores_string);
  RUN_TEST(test_stores_several_strings);
  RUN_TEST(test_circular_buffer_overwrites_oldest);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}