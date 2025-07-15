#include <unity.h>
#include <cstdint>
#include <cstring>
#include <trmnl_image.h>

void test_ImageBasic(void) {
  uint16_t width = 800;
  uint16_t height = 480;
  uint8_t bpp = 1;
  uint16_t bmpstride = StrideFourByte(width, bpp);
  uint16_t pngstride = StrideOneByte(width, bpp);
  TEST_ASSERT_EQUAL(bmpstride, pngstride);
  TEST_ASSERT_EQUAL(bmpstride, 100);
  TrmnlImage *image1 = TrmnlNewImage(800, 480, bpp, pngstride);
  TEST_ASSERT_EQUAL(image1->width, width);
  TEST_ASSERT_EQUAL(image1->height, height);
  TEST_ASSERT_EQUAL(image1->bpp, bpp);
  TEST_ASSERT_EQUAL(image1->stride, pngstride);
  TEST_ASSERT_EQUAL(image1->owndata, true);
  TrmnlImage *image2 = TrmnlUseImage(image1);
  TEST_ASSERT_EQUAL(image2->width, image1->width);
  TEST_ASSERT_EQUAL(image2->height, image1->height);
  TEST_ASSERT_EQUAL(image2->bpp, image1->bpp);
  TEST_ASSERT_EQUAL(image2->stride, image1->stride);
  TEST_ASSERT_EQUAL(image2->owndata, false);
  TrmnlDeleteImage(image2);
  // access image 1
  *TrmnlGetRowData(image1, 0) = 42;
  TrmnlDeleteImage(image1);
}

void test_ImageDifferentStrides(void) {
  uint16_t width = 804;
  uint16_t height = 640;
  uint8_t bpp = 1;
  uint16_t bmpstride = StrideFourByte(width, bpp);
  uint16_t pngstride = StrideOneByte(width, bpp);
  TEST_ASSERT_EQUAL(bmpstride, 104);
  TEST_ASSERT_EQUAL(pngstride, 101);
  TrmnlImage *image1 = TrmnlNewImage(width, height, bpp, bmpstride);
  // fill with pattern
  for (uint16_t h = 0; h < image1->height; ++h) {
    memset(TrmnlGetRowData(image1, h), h % 256, image1->stride);
  }
  // copy with different stride, not flipped
  TrmnlImage *image2 = TrmnlFromImage(image1);
  TEST_ASSERT_EQUAL(image2->stride, pngstride);
  // check that first byte for top and bottom row is the  same
  TEST_ASSERT_EQUAL(*TrmnlGetRowData(image1, 0), *TrmnlGetRowData(image2, 0));
  TEST_ASSERT_EQUAL(*TrmnlGetRowData(image1, height - 1),
                    *TrmnlGetRowData(image2, height - 1));
  TrmnlDeleteImage(image2);

// copy with different stride, flipped (what happens for normal BMPs)
  TrmnlImage *image3 = TrmnlFromImage(image1, true);
  // check that first byte for top and bottom row are flipped
  TEST_ASSERT_EQUAL(*TrmnlGetRowData(image1, 0), *TrmnlGetRowData(image3, height - 1));
  TEST_ASSERT_EQUAL(*TrmnlGetRowData(image1, height - 1),
                    *TrmnlGetRowData(image3, 0));
  TrmnlDeleteImage(image3);
  TrmnlDeleteImage(image1);
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
  RUN_TEST(test_ImageBasic);
  RUN_TEST(test_ImageDifferentStrides);
  UNITY_END();
}

int main(int argc, char **argv)
{
  process();
  return 0;
}