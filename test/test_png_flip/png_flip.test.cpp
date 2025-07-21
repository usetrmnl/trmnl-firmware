#include <unity.h>
#include <png_flip.h>
#include <string.h>

void test_reverse_bits(void)
{
  // Test cases:
  // 0000 0000 -> 0000 0000
  TEST_ASSERT_EQUAL_HEX8(0x00, reverse_bits(0x00));

  // 1111 1111 -> 1111 1111
  TEST_ASSERT_EQUAL_HEX8(0xFF, reverse_bits(0xFF));

  // 1010 1010 -> 0101 0101
  TEST_ASSERT_EQUAL_HEX8(0x55, reverse_bits(0xAA));

  // 0101 0101 -> 1010 1010
  TEST_ASSERT_EQUAL_HEX8(0xAA, reverse_bits(0x55));

  // 1000 0001 -> 1000 0001
  TEST_ASSERT_EQUAL_HEX8(0x81, reverse_bits(0x81));

  // 1100 0011 -> 1100 0011
  TEST_ASSERT_EQUAL_HEX8(0xC3, reverse_bits(0xC3));

  // 0011 1100 -> 0011 1100
  TEST_ASSERT_EQUAL_HEX8(0x3C, reverse_bits(0x3C));

  // 0001 0010 -> 0100 1000
  TEST_ASSERT_EQUAL_HEX8(0x48, reverse_bits(0x12));
}

void test_flip_image(void)
{
  // Create a test 16x8 image (2 bytes per row, 8 rows)
  const int width = 16;
  const int height = 8;
  const int row_bytes = width / 8;
  const int buffer_size = row_bytes * height;

  unsigned char original[buffer_size] = {
      0xAA, 0x55, // Row 0: 10101010 01010101
      0xA5, 0x5A, // Row 1: 10100101 01011010
      0x33, 0xCC, // Row 2: 00110011 11001100
      0x0F, 0xF0, // Row 3: 00001111 11110000
      0xFF, 0x00, // Row 4: 11111111 00000000
      0x00, 0xFF, // Row 5: 00000000 11111111
      0x3C, 0xC3, // Row 6: 00111100 11000011
      0x99, 0x66  // Row 7: 10011001 01100110
  };

  unsigned char expected[buffer_size] = {
      0x66, 0x99,
      0xC3, 0x3C,
      0xFF, 0x00,
      0x00, 0xFF,
      0x0F, 0xF0,
      0x33, 0xCC,
      0x5A, 0xA5,
      0xAA, 0x55};

  // Create a copy of the original to modify
  unsigned char buffer[buffer_size];
  memcpy(buffer, original, buffer_size);

  // Apply the flip operation
  flip_image(buffer, width, height, true);

  // Verify the result
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buffer, buffer_size);
}

void test_horizontal_mirror(void)
{
  // Create a test 16x4 image (2 bytes per row, 4 rows)
  const int width = 16;
  const int height = 4;
  const int row_bytes = width / 8;
  const int buffer_size = row_bytes * height;

  unsigned char original[buffer_size] = {
      0x55, 0xAA, // Row 0: 01010101 10101010
      0x33, 0xCC, // Row 1: 00110011 11001100
      0xFF, 0x00, // Row 2: 11111111 00000000
      0x3C, 0xC3  // Row 3: 00111100 11000011
  };

  unsigned char expected[buffer_size] = {
      0x55, 0xAA,
      0x33, 0xCC,
      0x00, 0xFF,
      0xC3, 0x3C};

  // Create a copy of the original to modify
  unsigned char buffer[buffer_size];
  memcpy(buffer, original, buffer_size);

  // Apply the horizontal mirror operation
  horizontal_mirror(buffer, width, height);

  // Verify the result
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buffer, buffer_size);
}

void setUp(void)
{
  // set stuff up here
}

void tearDown(void)
{
  // clean stuff up here
}

void process()
{
  UNITY_BEGIN();
  RUN_TEST(test_reverse_bits);
  RUN_TEST(test_flip_image);
  RUN_TEST(test_horizontal_mirror);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}