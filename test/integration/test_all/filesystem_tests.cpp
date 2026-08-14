// Cache eviction against real SPIFFS. The native suite covers the eviction policy; only
// hardware can show how much slack SPIFFS needs before it refuses to allocate, and whether
// evicting actually spares the filesystem from being formatted.

#include <Arduino.h>
#include <SPIFFS.h>
#include <filesystem.h>
#include <unity.h>

#include "tests.h"

static const size_t IMAGE_BYTES = 12000; // what a dithered OG screen compresses to
static const int MAX_FILL_IMAGES = 60;
static uint8_t *image_buffer = nullptr;

static void image_name(char *out, size_t out_size, int index) {
  snprintf(out, out_size, "/plugin-%06d-%010u", index, 1786700000u + (unsigned)index);
}

/// Writes straight to SPIFFS, chunked the way filesystem_write_to_file does, so that filling
/// the filesystem does not go through the eviction we are trying to test.
static bool write_image_directly(const char *name) {
  File file = SPIFFS.open(name, FILE_WRITE, true);
  if (!file) {
    return false;
  }

  size_t written = 0;
  while (written < IMAGE_BYTES) {
    size_t chunk = _min(4096, IMAGE_BYTES - written);
    if (file.write(image_buffer + written, chunk) != chunk) {
      file.close();
      return false;
    }
    written += chunk;
  }
  file.close();
  return true;
}

static uint32_t free_bytes() { return SPIFFS.totalBytes() - SPIFFS.usedBytes(); }

static int count_files() {
  int files = 0;
  File rootDir = SPIFFS.open("/");
  while (File file = rootDir.openNextFile()) {
    if (!file.isDirectory()) {
      files++;
    }
    file.close();
  }
  rootDir.close();
  return files;
}

// The number this test prints is what FS_WRITE_HEADROOM_BYTES has to cover.
static void test_spiffs_refuses_to_allocate_while_it_still_reports_free_space(void) {
  TEST_ASSERT_TRUE(SPIFFS.format());

  char name[36];
  uint32_t free_at_failure = 0;
  int images_written = 0;

  for (int i = 0; i < MAX_FILL_IMAGES; i++) {
    image_name(name, sizeof(name), i);
    free_at_failure = free_bytes();
    if (!write_image_directly(name)) {
      break;
    }
    images_written++;
    free_at_failure = 0;
  }

  char message[192];
  snprintf(message, sizeof(message),
           "SPIFFS total %u bytes, fit %d images of %u, then refused a write with %u bytes still free",
           (unsigned)SPIFFS.totalBytes(), images_written, (unsigned)IMAGE_BYTES, (unsigned)free_at_failure);
  TEST_MESSAGE(message);

  TEST_ASSERT_NOT_EQUAL_MESSAGE(0, free_at_failure,
                                "filesystem never refused a write - raise MAX_FILL_IMAGES or IMAGE_BYTES");
}

static void test_a_full_filesystem_evicts_the_oldest_image_instead_of_formatting(void) {
  TEST_ASSERT_TRUE(SPIFFS.format());

  // No timestamp, so this stands in for the setup logo and the last displayed image: the only
  // picture a device has when it cannot reach the API. Eviction must leave it alone.
  File fallback = SPIFFS.open("/current.bmp", FILE_WRITE, true);
  TEST_ASSERT_TRUE(fallback);
  fallback.write(image_buffer, 1024);
  fallback.close();

  char oldest[36];
  image_name(oldest, sizeof(oldest), 0);

  char name[36];
  int filled = 0;
  while (free_bytes() > IMAGE_BYTES && filled < MAX_FILL_IMAGES) {
    image_name(name, sizeof(name), filled);
    if (!write_image_directly(name)) {
      break;
    }
    filled++;
  }
  TEST_ASSERT_GREATER_THAN_MESSAGE(2, filled, "did not fit enough images to have anything to evict");

  uint32_t free_before = free_bytes();
  int files_before = count_files();

  char arrival[36];
  image_name(arrival, sizeof(arrival), 999);
  size_t written = filesystem_write_to_file(arrival, image_buffer, IMAGE_BYTES);

  char report[224];
  snprintf(report, sizeof(report), "filled %d; before: %d files %u free; wrote %u of %u; after: %d files %u free",
           filled, files_before, (unsigned)free_before, (unsigned)written, (unsigned)IMAGE_BYTES, count_files(),
           (unsigned)free_bytes());
  TEST_MESSAGE(report);

  TEST_ASSERT_EQUAL_MESSAGE(IMAGE_BYTES, written, report);
  TEST_ASSERT_TRUE_MESSAGE(SPIFFS.exists("/current.bmp"),
                           "offline fallback is gone - the filesystem was formatted after all");
  TEST_ASSERT_FALSE_MESSAGE(SPIFFS.exists(oldest), "oldest image survived - eviction chose the wrong file");
  TEST_ASSERT_TRUE_MESSAGE(SPIFFS.exists(arrival), "the image we just wrote is not on the filesystem");
}

// Keeps the pre-existing recovery honest: with nothing evictable the write must still fail
// rather than silently report success.
static void test_a_filesystem_with_nothing_evictable_still_falls_back(void) {
  TEST_ASSERT_TRUE(SPIFFS.format());

  char name[36];
  for (int i = 0; i < MAX_FILL_IMAGES; i++) {
    snprintf(name, sizeof(name), "/logo-%02d.bmp", i); // no trailing epoch, so never evictable
    if (!write_image_directly(name)) {
      break;
    }
  }

  TEST_ASSERT_NOT_EQUAL_MESSAGE(IMAGE_BYTES, filesystem_write_to_file("/plugin-abcdef-1786799999", image_buffer,
                                                                     IMAGE_BYTES),
                                "write reported success on a filesystem with no room and nothing to evict");
}

// A partial write used to survive on flash, and the cached-image path read it back and decoded
// half a picture. Reading has to refuse it so the caller deletes it and downloads again.
static void test_a_truncated_image_is_refused_rather_than_returned(void) {
  TEST_ASSERT_TRUE(SPIFFS.format());

  const uint32_t declared = IMAGE_BYTES;
  image_buffer[0] = 'B';
  image_buffer[1] = 'M';
  image_buffer[2] = (uint8_t)declared;
  image_buffer[3] = (uint8_t)(declared >> 8);
  image_buffer[4] = (uint8_t)(declared >> 16);
  image_buffer[5] = (uint8_t)(declared >> 24);

  const char *cut_short = "/plugin-000001-1786700001";
  File file = SPIFFS.open(cut_short, FILE_WRITE, true);
  TEST_ASSERT_TRUE(file);
  file.write(image_buffer, IMAGE_BYTES / 2); // the filesystem ran out of room half way through
  file.close();

  uint8_t *buffer = nullptr;
  TEST_ASSERT_EQUAL_MESSAGE(0, filesystem_read_and_allocate(cut_short, &buffer),
                            "a half-written image was handed back for display");

  const char *whole = "/plugin-000002-1786700002";
  TEST_ASSERT_TRUE(write_image_directly(whole));

  buffer = nullptr;
  TEST_ASSERT_EQUAL_MESSAGE(IMAGE_BYTES, filesystem_read_and_allocate(whole, &buffer),
                            "a complete image was refused");
  free(buffer);
}

void test_filesystem(void) {
  image_buffer = (uint8_t *)malloc(IMAGE_BYTES);
  TEST_ASSERT_NOT_NULL_MESSAGE(image_buffer, "could not allocate the test image buffer");
  memset(image_buffer, 0xA5, IMAGE_BYTES);

  TEST_ASSERT_TRUE(filesystem_init());

  RUN_TEST(test_spiffs_refuses_to_allocate_while_it_still_reports_free_space);
  RUN_TEST(test_a_full_filesystem_evicts_the_oldest_image_instead_of_formatting);
  RUN_TEST(test_a_filesystem_with_nothing_evictable_still_falls_back);
  RUN_TEST(test_a_truncated_image_is_refused_rather_than_returned);

  SPIFFS.format(); // leave the bench board with a clean filesystem
  free(image_buffer);
  image_buffer = nullptr;
}
