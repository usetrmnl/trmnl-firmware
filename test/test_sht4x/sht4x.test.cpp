#include <unity.h>
#include <sht4x.h>

// Canonical Sensirion CRC-8 test vector: CRC of {0xBE, 0xEF} is 0x92.
void test_sht4x_crc8_canonical_vector(void)
{
  const uint8_t data[2] = {0xBE, 0xEF};
  TEST_ASSERT_EQUAL_HEX8(0x92, sht4x_crc8(data, 2));
}

// Build a valid 6-byte frame from two data words, filling CRCs.
static void make_frame(uint8_t raw[6], uint16_t t, uint16_t h)
{
  raw[0] = (uint8_t)(t >> 8); raw[1] = (uint8_t)(t & 0xFF);
  raw[2] = sht4x_crc8(&raw[0], 2);
  raw[3] = (uint8_t)(h >> 8); raw[4] = (uint8_t)(h & 0xFF);
  raw[5] = sht4x_crc8(&raw[3], 2);
}

void test_sht4x_convert_valid(void)
{
  uint8_t raw[6];
  make_frame(raw, 0x6667, 0x8000); // ~25.0 C, ~56.5 %
  float t = 0.0f, h = 0.0f;
  TEST_ASSERT_TRUE(sht4x_convert(raw, t, h));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 25.0f, t);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 56.5f, h);
}

void test_sht4x_convert_humidity_clamps_low_and_high(void)
{
  uint8_t raw[6];
  float t = 0.0f, h = 0.0f;
  make_frame(raw, 0x6667, 0x0000); // RH formula = -6 -> clamp 0
  TEST_ASSERT_TRUE(sht4x_convert(raw, t, h));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, h);
  make_frame(raw, 0x6667, 0xFFFF); // RH formula = 119 -> clamp 100
  TEST_ASSERT_TRUE(sht4x_convert(raw, t, h));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, h);
}

void test_sht4x_convert_rejects_bad_crc(void)
{
  uint8_t raw[6];
  make_frame(raw, 0x6667, 0x8000);
  raw[2] ^= 0xFF; // corrupt temperature CRC
  float t = 0.0f, h = 0.0f;
  TEST_ASSERT_FALSE(sht4x_convert(raw, t, h));
}

void test_sht4x_convert_rejects_bad_humidity_crc(void)
{
  uint8_t raw[6];
  make_frame(raw, 0x6667, 0x8000);
  raw[5] ^= 0xFF; // corrupt humidity CRC
  float t = 0.0f, h = 0.0f;
  TEST_ASSERT_FALSE(sht4x_convert(raw, t, h));
}

void setUp(void) {}
void tearDown(void) {}

void process(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_sht4x_crc8_canonical_vector);
  RUN_TEST(test_sht4x_convert_valid);
  RUN_TEST(test_sht4x_convert_humidity_clamps_low_and_high);
  RUN_TEST(test_sht4x_convert_rejects_bad_crc);
  RUN_TEST(test_sht4x_convert_rejects_bad_humidity_crc);
  UNITY_END();
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  process();
  return 0;
}
