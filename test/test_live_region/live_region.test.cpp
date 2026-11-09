#include <live_region.h>
#include <string.h>
#include <unity.h>

// The first 33 bytes of a PNG: signature and IHDR
void pngHeaderBytes(uint8_t *out, uint32_t width, uint32_t height, uint8_t bitDepth, uint8_t colorType) {
  const uint8_t start[16] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 0, 0, 0, 13, 'I', 'H', 'D', 'R'};
  memcpy(out, start, 16);
  out[16] = width >> 24;
  out[17] = width >> 16;
  out[18] = width >> 8;
  out[19] = width;
  out[20] = height >> 24;
  out[21] = height >> 16;
  out[22] = height >> 8;
  out[23] = height;
  out[24] = bitDepth;
  out[25] = colorType;
  out[26] = 0; // compression
  out[27] = 0; // filter
  out[28] = 0; // interlace
  memset(&out[29], 0, 4); // CRC, not checked
}

void test_png_header_is_read(void) {
  uint8_t bytes[33];
  pngHeaderBytes(bytes, 800, 480, 1, 0);
  PngHeader header;
  TEST_ASSERT_TRUE(readPngHeader(bytes, sizeof(bytes), header));
  TEST_ASSERT_EQUAL_UINT32(800, header.width);
  TEST_ASSERT_EQUAL_UINT32(480, header.height);
  TEST_ASSERT_EQUAL_UINT8(1, header.bitDepth);
  TEST_ASSERT_EQUAL_UINT8(0, header.colorType);
}

void test_png_header_needs_signature_and_length(void) {
  uint8_t bytes[33];
  pngHeaderBytes(bytes, 8, 8, 1, 0);
  PngHeader header;
  TEST_ASSERT_FALSE(readPngHeader(bytes, 20, header));
  bytes[1] = 'X';
  TEST_ASSERT_FALSE(readPngHeader(bytes, sizeof(bytes), header));
}

void test_only_one_and_two_bit_gray_or_indexed_pngs_are_supported(void) {
  PngHeader header = {.width = 64, .height = 32, .bitDepth = 1, .colorType = 0, .interlace = 0};
  TEST_ASSERT_TRUE(liveRegionPngSupported(header));
  header.colorType = 3;
  TEST_ASSERT_TRUE(liveRegionPngSupported(header));
  header.bitDepth = 2;
  TEST_ASSERT_TRUE(liveRegionPngSupported(header));
  header.bitDepth = 8;
  TEST_ASSERT_FALSE(liveRegionPngSupported(header));
  header = {.width = 64, .height = 32, .bitDepth = 1, .colorType = 2, .interlace = 0}; // truecolor
  TEST_ASSERT_FALSE(liveRegionPngSupported(header));
  header = {.width = 64, .height = 32, .bitDepth = 1, .colorType = 0, .interlace = 1};
  TEST_ASSERT_FALSE(liveRegionPngSupported(header));
}

void test_region_must_lie_on_the_screen(void) {
  TEST_ASSERT_TRUE(liveRegionFits(0, 0, 800, 480, 800, 480));
  TEST_ASSERT_TRUE(liveRegionFits(700, 400, 100, 80, 800, 480));
  TEST_ASSERT_FALSE(liveRegionFits(701, 400, 100, 80, 800, 480));
  TEST_ASSERT_FALSE(liveRegionFits(700, 401, 100, 80, 800, 480));
  TEST_ASSERT_FALSE(liveRegionFits(-1, 0, 10, 10, 800, 480));
  TEST_ASSERT_FALSE(liveRegionFits(0, 0, 0, 10, 800, 480));
  TEST_ASSERT_FALSE(liveRegionFits(0, 0, 4000000000u, 10, 800, 480)); // no overflow
}

void test_coordinates_are_plain_numbers(void) {
  int32_t value = -1;
  TEST_ASSERT_TRUE(parseLiveRegionCoordinate("0", value));
  TEST_ASSERT_EQUAL_INT32(0, value);
  TEST_ASSERT_TRUE(parseLiveRegionCoordinate("472", value));
  TEST_ASSERT_EQUAL_INT32(472, value);
  TEST_ASSERT_FALSE(parseLiveRegionCoordinate("", value));
  TEST_ASSERT_FALSE(parseLiveRegionCoordinate(nullptr, value));
  TEST_ASSERT_FALSE(parseLiveRegionCoordinate("-3", value));
  TEST_ASSERT_FALSE(parseLiveRegionCoordinate("12px", value));
  TEST_ASSERT_FALSE(parseLiveRegionCoordinate("99999999999", value));
  TEST_ASSERT_EQUAL_INT32(472, value);
}

void test_one_bit_rows_keep_their_pixels(void) {
  const uint8_t pixels[2] = {0xF0, 0x0F};
  uint8_t out[2];
  TEST_ASSERT_TRUE(liveRegionRowTo1Bit(pixels, 16, 1, nullptr, out));
  TEST_ASSERT_EQUAL_HEX8(0xF0, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x0F, out[1]);

  const uint8_t blackFirst[6] = {0, 0, 0, 255, 255, 255};
  TEST_ASSERT_TRUE(liveRegionRowTo1Bit(pixels, 16, 1, blackFirst, out));
  TEST_ASSERT_EQUAL_HEX8(0xF0, out[0]);
}

void test_one_bit_rows_flip_when_white_comes_first_in_the_palette(void) {
  const uint8_t pixels[1] = {0xF0};
  const uint8_t whiteFirst[6] = {255, 255, 255, 0, 0, 0};
  uint8_t out[1];
  TEST_ASSERT_TRUE(liveRegionRowTo1Bit(pixels, 8, 1, whiteFirst, out));
  TEST_ASSERT_EQUAL_HEX8(0x0F, out[0]);
}

void test_two_bit_rows_make_all_but_white_black(void) {
  // gray samples 3, 2, 1, 0, 3, 3, 0, 3
  const uint8_t pixels[2] = {0b11100100, 0b11110011};
  uint8_t out[1];
  TEST_ASSERT_TRUE(liveRegionRowTo1Bit(pixels, 8, 2, nullptr, out));
  TEST_ASSERT_EQUAL_HEX8(0b10001101, out[0]);

  // indexed: 0 white, 1 black
  const uint8_t palette[6] = {255, 255, 255, 0, 0, 0};
  const uint8_t indexed[1] = {0b00010001};
  TEST_ASSERT_TRUE(liveRegionRowTo1Bit(indexed, 4, 2, palette, out));
  TEST_ASSERT_EQUAL_HEX8(0b10100000, out[0] & 0xF0);
}

void test_other_bit_depths_are_refused(void) {
  const uint8_t pixels[8] = {0};
  uint8_t out[1];
  TEST_ASSERT_FALSE(liveRegionRowTo1Bit(pixels, 8, 8, nullptr, out));
}

void test_aligned_row_is_copied_in_place(void) {
  uint8_t frame[4 * 3]; // 32 × 3
  memset(frame, 0xFF, sizeof(frame));
  const uint8_t row[2] = {0x00, 0x0F};
  liveRegionComposeRow(frame, 32, 8, 1, row, 16);
  const uint8_t expected[12] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, sizeof(frame));
}

void test_unaligned_row_touches_only_its_pixels(void) {
  uint8_t frame[3] = {0xFF, 0xFF, 0xFF}; // 24 × 1
  const uint8_t row[2] = {0x00, 0x00}; // 10 black pixels
  liveRegionComposeRow(frame, 24, 3, 0, row, 10);
  // pixels 3..12 black
  TEST_ASSERT_EQUAL_HEX8(0b11100000, frame[0]);
  TEST_ASSERT_EQUAL_HEX8(0b00000111, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, frame[2]);

  const uint8_t white[1] = {0xFF};
  liveRegionComposeRow(frame, 24, 5, 0, white, 2);
  TEST_ASSERT_EQUAL_HEX8(0b11100110, frame[0]);
}

void test_regions_refresh_partially_until_it_is_time_for_a_full_one(void) {
  TEST_ASSERT_EQUAL(LiveRegionRefresh::Partial, liveRegionRefresh(0, false, false));
  TEST_ASSERT_EQUAL(LiveRegionRefresh::Partial, liveRegionRefresh(LIVE_REGION_MAX_PARTIALS - 1, false, false));
  TEST_ASSERT_EQUAL(LiveRegionRefresh::Full, liveRegionRefresh(LIVE_REGION_MAX_PARTIALS, false, false));
}

void test_full_refresh_when_asked_or_in_maximum_compatibility(void) {
  TEST_ASSERT_EQUAL(LiveRegionRefresh::Full, liveRegionRefresh(0, true, false));
  TEST_ASSERT_EQUAL(LiveRegionRefresh::Full, liveRegionRefresh(0, false, true));
}

void test_dither_mask_covers_every_pixel_once_over_its_phases(void) {
  const uint32_t everies[] = {2, 4, 8, 16};
  for (uint32_t every : everies) {
    for (uint32_t row = 0; row < 4; row++) {
      uint8_t seen = 0;
      int count = 0;
      for (uint32_t phase = 0; phase < every; phase++) {
        uint8_t mask = liveRegionDitherMask(row, every, phase);
        TEST_ASSERT_EQUAL_HEX8(0, seen & mask); // no pixel twice
        seen |= mask;
        count += __builtin_popcount(mask);
      }
      TEST_ASSERT_EQUAL_HEX8(0xFF, seen);
      TEST_ASSERT_EQUAL(8, count);
    }
  }
}

void test_dither_mask_is_sparse(void) {
  // 1 in 16: two pixels of a 4×8 block over its four rows
  int count = 0;
  for (uint32_t row = 0; row < 4; row++)
    count += __builtin_popcount(liveRegionDitherMask(row, 16, 5));
  TEST_ASSERT_EQUAL(2, count);
  TEST_ASSERT_EQUAL_HEX8(0, liveRegionDitherMask(0, 3, 0));
  TEST_ASSERT_EQUAL_HEX8(0, liveRegionDitherMask(0, 0, 0));
}

void setUp(void) {}

void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_png_header_is_read);
  RUN_TEST(test_png_header_needs_signature_and_length);
  RUN_TEST(test_only_one_and_two_bit_gray_or_indexed_pngs_are_supported);
  RUN_TEST(test_region_must_lie_on_the_screen);
  RUN_TEST(test_coordinates_are_plain_numbers);
  RUN_TEST(test_one_bit_rows_keep_their_pixels);
  RUN_TEST(test_one_bit_rows_flip_when_white_comes_first_in_the_palette);
  RUN_TEST(test_two_bit_rows_make_all_but_white_black);
  RUN_TEST(test_other_bit_depths_are_refused);
  RUN_TEST(test_aligned_row_is_copied_in_place);
  RUN_TEST(test_unaligned_row_touches_only_its_pixels);
  RUN_TEST(test_regions_refresh_partially_until_it_is_time_for_a_full_one);
  RUN_TEST(test_full_refresh_when_asked_or_in_maximum_compatibility);
  RUN_TEST(test_dither_mask_covers_every_pixel_once_over_its_phases);
  RUN_TEST(test_dither_mask_is_sparse);
  return UNITY_END();
}
