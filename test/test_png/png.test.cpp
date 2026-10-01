#include <png.h>
#include <string.h>
#include <unity.h>
#include <vector>

// 8-byte signature, then the IHDR chunk: length, "IHDR", width, height, bit depth, ...
static std::vector<uint8_t> pngHeader(uint32_t width, uint32_t height) {
  std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 0, 0, 0, 13, 'I', 'H', 'D', 'R'};
  for (int shift = 24; shift >= 0; shift -= 8) png.push_back((width >> shift) & 0xFF);
  for (int shift = 24; shift >= 0; shift -= 8) png.push_back((height >> shift) & 0xFF);
  png.insert(png.end(), {1, 0, 0, 0, 0, 0, 0, 0, 0}); // 1-bit depth, color/compression/filter/interlace, crc
  return png;
}

void test_parsePNGHeader_PNG_NO_ERR(void) {
  auto png = pngHeader(800, 480);

  TEST_ASSERT_EQUAL(PNG_NO_ERR, parsePNGHeader(png.data(), png.size()));
}

void test_parsePNGHeader_wrong_signature(void) {
  auto png = pngHeader(800, 480);
  png[1] = 'X';

  TEST_ASSERT_EQUAL(PNG_WRONG_FORMAT, parsePNGHeader(png.data(), png.size()));
}

void test_parsePNGHeader_random_bytes(void) {
  std::vector<uint8_t> junk(48062);
  for (size_t i = 0; i < junk.size(); i++) junk[i] = (uint8_t)(i * 2654435761u >> 24);

  TEST_ASSERT_EQUAL(PNG_WRONG_FORMAT, parsePNGHeader(junk.data(), junk.size()));
}

void test_parsePNGHeader_missing_ihdr(void) {
  auto png = pngHeader(800, 480);
  memcpy(&png[12], "IDAT", 4);

  TEST_ASSERT_EQUAL(PNG_WRONG_FORMAT, parsePNGHeader(png.data(), png.size()));
}

void test_parsePNGHeader_truncated_before_ihdr(void) {
  auto png = pngHeader(800, 480);

  TEST_ASSERT_EQUAL(PNG_WRONG_FORMAT, parsePNGHeader(png.data(), 20));
}

void test_parsePNGHeader_empty_buffer(void) {
  uint8_t none = 0;

  TEST_ASSERT_EQUAL(PNG_WRONG_FORMAT, parsePNGHeader(&none, 0));
}

void test_parsePNGHeader_zero_size(void) {
  auto png = pngHeader(0, 480);

  TEST_ASSERT_EQUAL(PNG_BAD_SIZE, parsePNGHeader(png.data(), png.size()));
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_parsePNGHeader_PNG_NO_ERR);
  RUN_TEST(test_parsePNGHeader_wrong_signature);
  RUN_TEST(test_parsePNGHeader_random_bytes);
  RUN_TEST(test_parsePNGHeader_missing_ihdr);
  RUN_TEST(test_parsePNGHeader_truncated_before_ihdr);
  RUN_TEST(test_parsePNGHeader_empty_buffer);
  RUN_TEST(test_parsePNGHeader_zero_size);
  return UNITY_END();
}
