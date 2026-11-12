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

void test_one_and_two_bit_gray_or_indexed_screen_sized_pngs_are_drawable(void) {
  uint8_t png[33];
  pngHeaderBytes(png, 800, 480, 1, 0);
  TEST_ASSERT_TRUE(liveFramePngDrawable(png, sizeof(png), 800, 480));
  pngHeaderBytes(png, 800, 480, 2, 3);
  TEST_ASSERT_TRUE(liveFramePngDrawable(png, sizeof(png), 800, 480));
}

void test_deeper_truecolor_and_interlaced_pngs_are_not_drawable(void) {
  uint8_t png[33];
  pngHeaderBytes(png, 800, 480, 4, 0);
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

void test_one_bit_rows_become_black_and_white_levels(void) {
  const uint8_t pixels[1] = {0xF0};
  const uint8_t expected[2] = {0xFF, 0x00};
  uint8_t out[2];
  TEST_ASSERT_FALSE(liveFrameRow(pixels, 8, 1, nullptr, out));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, 2);
}

void test_two_bit_gray_rows_keep_their_levels_and_report_gray(void) {
  const uint8_t pixels[1] = {0x1B}; // black, dark gray, light gray, white
  uint8_t out[1];
  TEST_ASSERT_TRUE(liveFrameRow(pixels, 4, 2, nullptr, out));
  TEST_ASSERT_EQUAL_HEX8(0x1B, out[0]);
}

void test_indexed_rows_take_their_level_from_the_palette(void) {
  const uint8_t pixels[1] = {0x1B};
  const uint8_t palette[12] = {255, 255, 255, 170, 170, 170, 85, 85, 85, 0, 0, 0}; // white first
  uint8_t out[1];
  TEST_ASSERT_TRUE(liveFrameRow(pixels, 4, 2, palette, out));
  TEST_ASSERT_EQUAL_HEX8(0xE4, out[0]);
}

// Plane bits per pixel (plane 1, plane 2): 11 WW, 01 BW, 10 WB, 00 BB
static void planes(uint8_t was, uint8_t now, int pass, uint8_t *plane1, uint8_t *plane2) {
  liveFramePlaneRow(&was, &now, 4, pass, 1, plane1);
  liveFramePlaneRow(&was, &now, 4, pass, 2, plane2);
}

void test_pass_one_takes_changed_pixels_to_the_nearer_extreme_and_holds_white(void) {
  uint8_t plane1, plane2;
  planes(0x3F, 0xC7, 1, &plane1, &plane2); // black->white (BW), white->black (WB), white->dark (WB), white (WW)
  TEST_ASSERT_EQUAL_HEX8(0x70, plane1);
  TEST_ASSERT_EQUAL_HEX8(0x90, plane2);
}

void test_pass_one_leaves_unchanged_black_and_gray_to_the_bb_table(void) {
  uint8_t plane1, plane2;
  planes(0x1B, 0x1B, 1, &plane1, &plane2);
  TEST_ASSERT_EQUAL_HEX8(0x10, plane1);
  TEST_ASSERT_EQUAL_HEX8(0x10, plane2);
}

void test_pass_two_pushes_new_grays_and_holds_unchanged_black(void) {
  uint8_t plane1, plane2;
  planes(0x0F, 0x1B, 2, &plane1, &plane2); // black stays (BB), black->dark (BW), white->light (WB), white (WW)
  TEST_ASSERT_EQUAL_HEX8(0x30, plane1);
  TEST_ASSERT_EQUAL_HEX8(0x50, plane2);
}

void test_packed_rows_unpack_to_the_same_levels(void) {
  uint8_t levels[200], packed[800], unpacked[200];
  memset(levels, 0xFF, sizeof(levels)); // white, 68 black (two runs), then grays
  memset(levels + 10, 0x00, 17);
  levels[27] = 0x1B;
  const size_t written = liveFrameRowPack(levels, 800, packed, sizeof(packed));
  TEST_ASSERT_EQUAL(written, liveFrameRowUnpack(packed, 800, unpacked));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(levels, unpacked, sizeof(levels));
}

void test_rows_that_do_not_fit_pack_to_nothing(void) {
  uint8_t levels[200], packed[4];
  memset(levels, 0x1B, sizeof(levels)); // every pixel its own run
  TEST_ASSERT_EQUAL(0, liveFrameRowPack(levels, 800, packed, sizeof(packed)));
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_one_and_two_bit_gray_or_indexed_screen_sized_pngs_are_drawable);
  RUN_TEST(test_deeper_truecolor_and_interlaced_pngs_are_not_drawable);
  RUN_TEST(test_pngs_of_another_size_are_not_drawable);
  RUN_TEST(test_truncated_or_unsigned_data_is_not_drawable);
  RUN_TEST(test_one_bit_rows_become_black_and_white_levels);
  RUN_TEST(test_two_bit_gray_rows_keep_their_levels_and_report_gray);
  RUN_TEST(test_indexed_rows_take_their_level_from_the_palette);
  RUN_TEST(test_pass_one_takes_changed_pixels_to_the_nearer_extreme_and_holds_white);
  RUN_TEST(test_pass_one_leaves_unchanged_black_and_gray_to_the_bb_table);
  RUN_TEST(test_pass_two_pushes_new_grays_and_holds_unchanged_black);
  RUN_TEST(test_packed_rows_unpack_to_the_same_levels);
  RUN_TEST(test_rows_that_do_not_fit_pack_to_nothing);
  return UNITY_END();
}
