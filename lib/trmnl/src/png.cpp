#include <inttypes.h>
#include <png.h>
#include <string.h>
#include <trmnl_log.h>

/**
 * @brief Function to check that a buffer starts with a PNG signature and IHDR chunk
 *        The decoder draws straight to the panel, so this is the only check before
 *        the image is shown and cached
 * @param data pointer to the buffer
 * @param size number of bytes in the buffer
 * @return image_err_e error code
 */
image_err_e parsePNGHeader(const uint8_t *data, uint32_t size) {
  static const uint8_t szSignature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  uint32_t width, height;

  if (size < 33 || memcmp(data, szSignature, 8) != 0 || memcmp(&data[12], "IHDR", 4) != 0) { // 8 signature + 25 IHDR
    Log_error("It is not a PNG file");
    return PNG_WRONG_FORMAT;
  }
  width = ((uint32_t)data[16] << 24) | ((uint32_t)data[17] << 16) | ((uint32_t)data[18] << 8) | data[19];
  height = ((uint32_t)data[20] << 24) | ((uint32_t)data[21] << 16) | ((uint32_t)data[22] << 8) | data[23];
  if (width == 0 || height == 0) {
    Log_error("PNG has an empty size (%" PRIu32 "x%" PRIu32 ")", width, height);
    return PNG_BAD_SIZE;
  }
  Log_info("PNG %" PRIu32 "x%" PRIu32 ", %d-bit", width, height, data[24]);
  return PNG_NO_ERR;
} /* parsePNGHeader() */
