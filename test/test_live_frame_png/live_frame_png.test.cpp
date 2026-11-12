#include <live_frame_png.h>
#include <string.h>
#include <unity.h>

// The first 33 bytes of a PNG: signature and IHDR
static void pngHeaderBytes(uint8_t *out, uint32_t width, uint32_t height, uint8_t bitDepth, uint8_t colorType,
                           uint8_t interlace = 0) {
  const uint8_t start[16] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 0, 0, 0, 13, 'I', 'H', 'D', 'R'};
  memcpy(out, start, 16);
  for (int i = 0; i < 4; i++) {
    out[16 + i] = width >> (24 - 8 * i);
    out[20 + i] = height >> (24 - 8 * i);
  }
  out[24] = bitDepth;
  out[25] = colorType;
  out[26] = 0; // compression
  out[27] = 0; // filter
  out[28] = interlace;
  memset(&out[29], 0, 4); // CRC, not checked
}

void test_one_bit_gray_or_indexed_screen_sized_pngs_are_drawable(void) {
  uint8_t png[33];
  pngHeaderBytes(png, 800, 480, 1, 0);
  TEST_ASSERT_TRUE(liveFramePngDrawable(png, sizeof(png), 800, 480));
  pngHeaderBytes(png, 800, 480, 1, 3);
  TEST_ASSERT_TRUE(liveFramePngDrawable(png, sizeof(png), 800, 480));
}

void test_gray_truecolor_and_interlaced_pngs_are_not_drawable(void) {
  uint8_t png[33];
  pngHeaderBytes(png, 800, 480, 2, 0);
  TEST_ASSERT_FALSE(liveFramePngDrawable(png, sizeof(png), 800, 480));
  pngHeaderBytes(png, 800, 480, 8, 2);
  TEST_ASSERT_FALSE(liveFramePngDrawable(png, sizeof(png), 800, 480));
  pngHeaderBytes(png, 800, 480, 1, 0, 1);
  TEST_ASSERT_FALSE(liveFramePngDrawable(png, sizeof(png), 800, 480));
}

void test_pngs_of_another_size_are_not_drawable(void) {
  uint8_t png[33];
  pngHeaderBytes(png, 480, 800, 1, 0);
  TEST_ASSERT_FALSE(liveFramePngDrawable(png, sizeof(png), 800, 480));
}

void test_truncated_or_unsigned_data_is_not_drawable(void) {
  uint8_t png[33];
  pngHeaderBytes(png, 800, 480, 1, 0);
  TEST_ASSERT_FALSE(liveFramePngDrawable(png, 28, 800, 480));
  png[1] = 'X';
  TEST_ASSERT_FALSE(liveFramePngDrawable(png, sizeof(png), 800, 480));
}

void test_rows_keep_their_pixels_when_black_comes_first(void) {
  const uint8_t pixels[2] = {0xF0, 0x0F};
  const uint8_t blackFirst[6] = {0, 0, 0, 255, 255, 255};
  uint8_t out[2];
  liveFrameRow(pixels, 16, nullptr, out);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pixels, out, 2);
  liveFrameRow(pixels, 16, blackFirst, out);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pixels, out, 2);
}

void test_rows_flip_when_white_comes_first_in_the_palette(void) {
  const uint8_t pixels[1] = {0xF0};
  const uint8_t whiteFirst[6] = {255, 255, 255, 0, 0, 0};
  uint8_t out[1];
  liveFrameRow(pixels, 8, whiteFirst, out);
  TEST_ASSERT_EQUAL_HEX8(0x0F, out[0]);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_one_bit_gray_or_indexed_screen_sized_pngs_are_drawable);
  RUN_TEST(test_gray_truecolor_and_interlaced_pngs_are_not_drawable);
  RUN_TEST(test_pngs_of_another_size_are_not_drawable);
  RUN_TEST(test_truncated_or_unsigned_data_is_not_drawable);
  RUN_TEST(test_rows_keep_their_pixels_when_black_comes_first);
  RUN_TEST(test_rows_flip_when_white_comes_first_in_the_palette);
  return UNITY_END();
}
