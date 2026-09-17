#pragma once

#include <cstdint>

enum image_err_e {
  PNG_BAD_SIZE,
  PNG_WRONG_FORMAT,
  PNG_NO_ERR,
  PNG_DECODE_ERR,
  PNG_MALLOC_FAILED,
  PNG_FS_ERROR,
  PNG_FILE_NOT_FOUND
};

image_err_e parsePNGHeader(const uint8_t *data, uint32_t size);
