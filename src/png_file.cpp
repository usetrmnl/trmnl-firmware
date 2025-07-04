#include <png_file.h>
#include <trmnl_log.h>
#include <PNGdec.h>
#include <esp_mac.h>
#include <SPIFFS.h>
#include "png.h"

File pngfile; // Global file handle

void *pngOpen(const char *filename, int32_t *size)
{
  Serial.printf("Attempting to open %s from SPIFFS\n", filename);
  pngfile = SPIFFS.open(filename, "r");

  if (!pngfile)
  {
    Serial.println("Failed to open file!");
    *size = 0;
    return nullptr;
  }

  *size = pngfile.size();
  return &pngfile;
}

void pngClose(void *handle)
{
  if (pngfile)
  {
    pngfile.close();
  }
}

int32_t pngRead(PNGFILE *page, uint8_t *buffer, int32_t length)
{
  if (!pngfile)
    return 0;
  (void)page; // suppress unused warning
  return pngfile.read(buffer, length);
}

int32_t pngSeek(PNGFILE *page, int32_t position)
{
  if (!pngfile)
    return 0;
  (void)page;
  return pngfile.seek(position);
}

/**
 * @brief Function to decode png file
 * @param szFilename PNG file location
 * @param decodded_buffer Buffer where decoded PNG bitmap save
 * @return image_err_e error code
 */
image_err_e decodePNG(const char *szFilename, uint8_t *&decoded_buffer)
{
  PNG *png = new PNG();

  if (!png)
    return PNG_MALLOC_FAILED;

  int rc = png->open(szFilename, pngOpen, pngClose, pngRead, pngSeek, nullptr);

  if (rc == PNG_INVALID_FILE)
  {
    Log_error("WRONG_FILE");
    delete png;
    return PNG_WRONG_FORMAT;
  }

  image_err_e result = processPNG(png, decoded_buffer);
  delete png;
  return result;
}
