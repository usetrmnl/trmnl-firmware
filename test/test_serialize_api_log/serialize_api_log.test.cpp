#include <unity.h>
#include <ArduinoJson.h>
#include <api_types.h>
#include <serialize_log.h>
#include <api_request_serialization.h>


String compact(String input)
{
  JsonDocument doc;
  deserializeJson(doc, input);
  String output;
  serializeJson(doc, output);
  return output;
}

void test_serialize_one_log(void)
{
  auto expected = String("{\"logs\":[{\"foo\":\"bar\"}]}");
  auto result = serializeApiLogRequest("{\"foo\":\"bar\"}");

  TEST_ASSERT_EQUAL_STRING(expected.c_str(), result.c_str());
}

void test_serialize_multiple_logs(void)
{
  auto expected = String("{\"logs\":[{\"foo\":\"bar\"},{\"foo\":\"baz\"}]}");
  auto result = serializeApiLogRequest("{\"foo\":\"bar\"},{\"foo\":\"baz\"}");

  TEST_ASSERT_EQUAL_STRING(expected.c_str(), result.c_str());
}

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_serialize_one_log);
  RUN_TEST(test_serialize_multiple_logs);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}