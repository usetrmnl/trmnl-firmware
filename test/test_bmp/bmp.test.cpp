#include <bmp.h>
#include <fstream>
#include <string.h>
#include <unity.h>
#include <vector>

std::vector<uint8_t> readBMPFile(const char *filename) {
  try {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open BMP file.");
    }

    // Get the size of the file
    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // Create a vector to hold file data
    std::vector<uint8_t> buffer(fileSize);

    // Read the file data into the vector
    file.read(reinterpret_cast<char *>(buffer.data()), fileSize);
    file.close();

    return buffer;
  } catch (const std::exception &e) {
    TEST_FAIL_MESSAGE(e.what());
  }
}

void test_parseBMPHeader_BMP_NO_ERR(void) {
  auto bmp_data = readBMPFile("./test.bmp");
  bool image_reverse = false;

  bmp_err_e result = parseBMPHeader(bmp_data.data(), image_reverse);

  TEST_ASSERT_EQUAL(BMP_NO_ERR, result);
  TEST_ASSERT_EQUAL(false, image_reverse);
}

void test_parseBMPHeader_BMP_NO_ERR_reversed(void) {
  auto bmp_data = readBMPFile("./test.bmp");
  bool image_reverse = false;

  bmp_data[54] = 255;
  bmp_data[55] = 255;
  bmp_data[56] = 255;
  bmp_data[57] = 0;
  bmp_data[58] = 0;
  bmp_data[59] = 0;
  bmp_data[60] = 0;
  bmp_data[61] = 0;

  TEST_ASSERT_EQUAL(BMP_NO_ERR, parseBMPHeader(bmp_data.data(), image_reverse));
  TEST_ASSERT_EQUAL(true, image_reverse);
}

void test_parseBMPHeader_NOT_BMP(void) {
  auto bmp_data = readBMPFile("./test.bmp");
  bool image_reverse = false;

  bmp_data[0] = 'A';

  TEST_ASSERT_EQUAL(BMP_NOT_BMP, parseBMPHeader(bmp_data.data(), image_reverse));
}

void test_parseBMPHeader_BMP_BAD_SIZE(void) {
  auto bmp_data = readBMPFile("./test.bmp");
  bool image_reverse = false;

  bmp_data[18] = 123;

  TEST_ASSERT_EQUAL(BMP_BAD_SIZE, parseBMPHeader(bmp_data.data(), image_reverse));
}

void test_parseBMPHeader_BMP_COLOR_SCHEME_FAILED(void) {
  auto bmp_data = readBMPFile("./test.bmp");
  bool image_reverse = false;

  bmp_data[54] = 123;

  TEST_ASSERT_EQUAL(BMP_COLOR_SCHEME_FAILED, parseBMPHeader(bmp_data.data(), image_reverse));
}

void test_parseBMPHeader_BMP_INVALID_OFFSET(void) {
  auto bmp_data = readBMPFile("./test.bmp");
  bool image_reverse = false;

  bmp_data[10] = 5;

  TEST_ASSERT_EQUAL(BMP_INVALID_OFFSET, parseBMPHeader(bmp_data.data(), image_reverse));
}

void test_bmpIsTruncated_whole_file(void) {
  auto bmp_data = readBMPFile("./test.bmp");

  TEST_ASSERT_FALSE(bmpIsTruncated(bmp_data.data(), bmp_data.size()));
}

void test_bmpIsTruncated_short_of_declared_length(void) {
  auto bmp_data = readBMPFile("./test.bmp");

  TEST_ASSERT_TRUE(bmpIsTruncated(bmp_data.data(), bmp_data.size() - 1));
}

// A partial write that stopped on a 4096-byte chunk boundary, which is how the filesystem cuts them.
void test_bmpIsTruncated_first_chunk_only(void) {
  auto bmp_data = readBMPFile("./test.bmp");

  TEST_ASSERT_TRUE(bmpIsTruncated(bmp_data.data(), 4096));
}

void test_bmpIsTruncated_too_short_to_carry_its_length(void) {
  uint8_t stub[4] = {'B', 'M', 0x00, 0x00};

  TEST_ASSERT_TRUE(bmpIsTruncated(stub, sizeof(stub)));
}

// PNG and JPEG caches go through the same guard, and nothing in them declares a length.
void test_bmpIsTruncated_ignores_other_formats(void) {
  uint8_t png[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};

  TEST_ASSERT_FALSE(bmpIsTruncated(png, sizeof(png)));
}

void test_bmpIsTruncated_accepts_a_zero_declared_length(void) {
  uint8_t headerless[6] = {'B', 'M', 0x00, 0x00, 0x00, 0x00};

  TEST_ASSERT_FALSE(bmpIsTruncated(headerless, sizeof(headerless)));
}

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_parseBMPHeader_BMP_NO_ERR);
  RUN_TEST(test_parseBMPHeader_BMP_NO_ERR_reversed);
  RUN_TEST(test_parseBMPHeader_NOT_BMP);
  RUN_TEST(test_parseBMPHeader_BMP_BAD_SIZE);
  RUN_TEST(test_parseBMPHeader_BMP_COLOR_SCHEME_FAILED);
  RUN_TEST(test_parseBMPHeader_BMP_INVALID_OFFSET);
  RUN_TEST(test_bmpIsTruncated_whole_file);
  RUN_TEST(test_bmpIsTruncated_short_of_declared_length);
  RUN_TEST(test_bmpIsTruncated_first_chunk_only);
  RUN_TEST(test_bmpIsTruncated_too_short_to_carry_its_length);
  RUN_TEST(test_bmpIsTruncated_ignores_other_formats);
  RUN_TEST(test_bmpIsTruncated_accepts_a_zero_declared_length);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}