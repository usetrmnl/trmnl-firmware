#pragma once

#include <PNGdec.h>

enum image_err_e
{
  PNG_BAD_SIZE,
  PNG_WRONG_FORMAT,
  PNG_NO_ERR,
  PNG_DECODE_ERR,
  PNG_MALLOC_FAILED,
  PNG_FS_ERROR,
  PNG_FILE_NOT_FOUND
};

image_err_e processPNG(PNG *png, uint8_t * png_buffer, uint8_t *decoded_buffer);
image_err_e decodePNG(uint8_t* pngbuffer, uint8_t *&decoded_buffer);