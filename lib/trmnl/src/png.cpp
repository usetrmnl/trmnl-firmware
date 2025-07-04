#include <PNGdec.h>
#include "png.h"
#include <trmnl_log.h>

image_err_e processPNG(PNG *png, uint8_t *&decoded_buffer)
{
  if (!(decoded_buffer = (uint8_t *)malloc(48000)))
  {
    Log_error("PNG MALLOC FAILED");
    return PNG_MALLOC_FAILED;
  }
  png->setBuffer(decoded_buffer);

  uint32_t width = png->getWidth();
  uint32_t height = png->getHeight();
  uint32_t bpp = png->getBpp();

  if (width != 800 || height != 480 || bpp != 1)
  {
    Log_error("PNG_BAD_SIZE");
    return PNG_BAD_SIZE;
  }

  if (!(png->decode(nullptr, 0)))
  {
    Log_error("PNG_SUCCESS");
    return PNG_NO_ERR;
  }
  Log_error("PNG_DECODE_ERR");
  return PNG_DECODE_ERR;
}

/**
 * @brief Function to decode png from buffer
 * @param buffer pointer to the buffer
 * @param decodded_buffer Buffer where decoded PNG bitmap save
 * @return image_err_e error code
 */
image_err_e decodePNG(uint8_t *buffer, uint8_t *&decoded_buffer)
{
  PNG *png = new PNG();

  if (!png)
    return PNG_MALLOC_FAILED;

  int rc = png->openRAM(buffer, 48000, nullptr);

  if (rc == PNG_INVALID_FILE)
  {
    delete png;
    return PNG_WRONG_FORMAT;
  }

  image_err_e result = processPNG(png, decoded_buffer);
  delete png;
  return result;
}