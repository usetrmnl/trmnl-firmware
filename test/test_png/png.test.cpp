#include <unity.h>
#include <png.h>
#include <PNGdec.h>
#include <string.h>
#include <fstream>
#include <vector>

std::vector<uint8_t> readPNGFile(const char *filename)
{
  try
  {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
      throw std::runtime_error("Failed to open PNG file.");
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
  }
  catch (const std::exception &e)
  {
    TEST_FAIL_MESSAGE(e.what());
    return std::vector<uint8_t>();
  }
}

// Create invalid PNG data (just some random bytes that don't form a valid PNG)
std::vector<uint8_t> createInvalidPNGData()
{
  std::vector<uint8_t> invalid_data = {0x42, 0x41, 0x44, 0x50, 0x4E, 0x47, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
  return invalid_data;
}

void test_decodePNG_Success()
{
  // Read the valid PNG file (800x480 1-bit)
  auto png_data = readPNGFile("./test/test_png/valid_size.png");

  TEST_ASSERT_TRUE(png_data.size() > 0);

  uint8_t *decoded_buffer = nullptr;

  image_err_e result = decodePNG(png_data.data(), decoded_buffer);

  TEST_ASSERT_EQUAL(PNG_NO_ERR, result);
  TEST_ASSERT_NOT_NULL(decoded_buffer);

  if (decoded_buffer != nullptr)
  {
    free(decoded_buffer);
  }
}

void test_decodePNG_WrongSize()
{
  // Read the wrong size PNG file (640x480)
  auto png_data = readPNGFile("./test/test_png/wrong_size.png");

  TEST_ASSERT_TRUE(png_data.size() > 0);

  uint8_t *decoded_buffer = nullptr;

  image_err_e result = decodePNG(png_data.data(), decoded_buffer);

  TEST_ASSERT_EQUAL(PNG_BAD_SIZE, result);

  if (decoded_buffer != nullptr)
  {
    free(decoded_buffer);
  }
}

void test_decodePNG_WrongDepth()
{
  // Read the wrong depth PNG file (8-bit instead of 1-bit)
  auto png_data = readPNGFile("./test/test_png/wrong_depth.png");

  TEST_ASSERT_TRUE(png_data.size() > 0);

  uint8_t *decoded_buffer = nullptr;

  image_err_e result = decodePNG(png_data.data(), decoded_buffer);

  // Should fail with bad size error (which includes bpp check)
  TEST_ASSERT_EQUAL(PNG_BAD_SIZE, result);

  if (decoded_buffer != nullptr)
  {
    free(decoded_buffer);
  }
}

void test_decodePNG_InvalidFormat()
{
  auto invalid_data = createInvalidPNGData();
  uint8_t *decoded_buffer = nullptr;

  image_err_e result = decodePNG(invalid_data.data(), decoded_buffer);

  TEST_ASSERT_EQUAL(PNG_WRONG_FORMAT, result);

  TEST_ASSERT_NULL(decoded_buffer);
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
  RUN_TEST(test_decodePNG_Success);
  RUN_TEST(test_decodePNG_WrongSize);
  RUN_TEST(test_decodePNG_WrongDepth);
  RUN_TEST(test_decodePNG_InvalidFormat);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}