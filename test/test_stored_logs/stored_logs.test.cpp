#include <unity.h>
#include "stored_logs.h"
#include <unordered_map>
#include <string>
#include "memory_persistence.h"

void test_stores_several_strings()
{
  MemoryPersistence persistence;
  StoredLogs subject(0, 3, "log_", "log_head", persistence);
  
  subject.store_log("asdf");
  subject.store_log("qwer");
  subject.store_log("zxcv");
  TEST_ASSERT_EQUAL_STRING("asdf,qwer,zxcv", subject.gather_stored_logs().c_str());
}

void test_circular_buffer_overwrites_oldest()
{
  MemoryPersistence persistence;
  StoredLogs subject(0, 3, "log_", "log_head", persistence);
  
  subject.store_log("log1");
  subject.store_log("log2");
  subject.store_log("log3");
  TEST_ASSERT_EQUAL_STRING("log1,log2,log3", subject.gather_stored_logs().c_str());

  subject.store_log("log4");
  TEST_ASSERT_EQUAL_STRING("log2,log3,log4", subject.gather_stored_logs().c_str());
}

void test_overwrite_counter()
{
  MemoryPersistence persistence;
  StoredLogs subject(0, 3, "log_", "log_head", persistence);
  
  TEST_ASSERT_EQUAL(0, subject.get_overwrite_count());

  // Fill all slots - no overwrites yet
  subject.store_log("log1");
  subject.store_log("log2");
  subject.store_log("log3");
  TEST_ASSERT_EQUAL(0, subject.get_overwrite_count());

  // Start overwriting
  subject.store_log("log4");
  TEST_ASSERT_EQUAL(1, subject.get_overwrite_count());

  subject.store_log("log5");
  TEST_ASSERT_EQUAL(2, subject.get_overwrite_count());

  // Clear should reset counter
  subject.clear_stored_logs();
  TEST_ASSERT_EQUAL(0, subject.get_overwrite_count());
}

void test_keeps_oldest_only()
{
  MemoryPersistence persistence;
  StoredLogs subject(3, 0, "log_", "log_head", persistence);

  subject.store_log("first");
  subject.store_log("second");
  subject.store_log("third");
  TEST_ASSERT_EQUAL_STRING("first,second,third", subject.gather_stored_logs().c_str());

  subject.store_log("fourth");
  TEST_ASSERT_EQUAL_STRING("first,second,third", subject.gather_stored_logs().c_str());

  subject.store_log("fifth");
  TEST_ASSERT_EQUAL_STRING("first,second,third", subject.gather_stored_logs().c_str());
}

void test_mixed_mode_1_oldest_2_newest()
{
  MemoryPersistence persistence;
  StoredLogs subject(1, 2, "log_", "log_head", persistence);

  subject.store_log("first");
  subject.store_log("second");
  subject.store_log("third");
  TEST_ASSERT_EQUAL_STRING("first,second,third", subject.gather_stored_logs().c_str());

  subject.store_log("fourth");
  TEST_ASSERT_EQUAL_STRING("first,third,fourth", subject.gather_stored_logs().c_str());

  subject.store_log("fifth");
  TEST_ASSERT_EQUAL_STRING("first,fourth,fifth", subject.gather_stored_logs().c_str());

  subject.clear_stored_logs();
  TEST_ASSERT_EQUAL_STRING("", subject.gather_stored_logs().c_str());
  TEST_ASSERT_EQUAL(0, subject.get_overwrite_count());
}

void test_mixed_mode_2_oldest_1_newest()
{
  MemoryPersistence persistence;
  StoredLogs subject(2, 1, "log_", "log_head", persistence);

  subject.store_log("first");
  subject.store_log("second");
  subject.store_log("third");
  TEST_ASSERT_EQUAL_STRING("first,second,third", subject.gather_stored_logs().c_str());

  subject.store_log("fourth");
  TEST_ASSERT_EQUAL_STRING("first,second,fourth", subject.gather_stored_logs().c_str());

  subject.store_log("fifth");
  TEST_ASSERT_EQUAL_STRING("first,second,fifth", subject.gather_stored_logs().c_str());
}

void test_several_overwrites()
{
  MemoryPersistence persistence;
  StoredLogs subject(3, 5, "log_", "log_head", persistence);

  for (int i = 0; i < 15; i++)
  {
    subject.store_log(String(i));
  }

  TEST_ASSERT_EQUAL(7, subject.get_overwrite_count());
  TEST_ASSERT_EQUAL_STRING("0,1,2,10,11,12,13,14", subject.gather_stored_logs().c_str());
}

void setUp(void) {}

void tearDown(void) {}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_stores_several_strings);
  RUN_TEST(test_circular_buffer_overwrites_oldest);
  RUN_TEST(test_overwrite_counter);
  RUN_TEST(test_keeps_oldest_only);
  RUN_TEST(test_mixed_mode_1_oldest_2_newest);
  RUN_TEST(test_mixed_mode_2_oldest_1_newest);
  RUN_TEST(test_several_overwrites);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}