#include <unity.h>
#include <sht4x.h>

// Canonical Sensirion CRC-8 test vector: CRC of {0xBE, 0xEF} is 0x92.
void test_sht4x_crc8_canonical_vector(void)
{
  const uint8_t data[2] = {0xBE, 0xEF};
  TEST_ASSERT_EQUAL_HEX8(0x92, sht4x_crc8(data, 2));
}

void setUp(void) {}
void tearDown(void) {}

void process(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_sht4x_crc8_canonical_vector);
  UNITY_END();
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  process();
  return 0;
}
