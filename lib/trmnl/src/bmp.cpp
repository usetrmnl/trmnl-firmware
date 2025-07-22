#include <bmp.h>
#include <trmnl_log.h>
#include <math.h>
#include<stdio.h>

/**
 * @brief Function to parse .bmp file header
 * @param data pointer to the buffer
 * @return bmp_err_e error code
 */
bmp_err_e parseBMPHeader(uint8_t *data)
{

  // Check if the file is a BMP image
  if (*(uint16_t*)data != BMP_SIGNATURE)
  {
    Log_fatal("It is not a BMP file");
    return BMP_NOT_BMP;
  }
  // Get width and height from the header
  int32_t width = *(int32_t *)&data[18];
  int32_t height = *(int32_t *)&data[22];
  uint16_t bitsPerPixel = *(uint16_t *)&data[28];
  uint32_t compressionMethod = *(uint32_t *)&data[30];
  uint32_t imageDataSize = *(uint32_t *)&data[34];
  uint32_t colorTableEntries = *(uint32_t *)&data[46];
 // Get the offset of the pixel data
  uint32_t dataOffset = *(uint32_t *)&data[10];
// Display BMP information
  printf("BMP Header Information:\r\nWidth: %d\r\nHeight: %d\r\nBits per Pixel: %d\r\nCompression Method: %d\r\nImage Data Size: %d\r\nColor Table Entries: %d\r\nData offset: %d", width, height, bitsPerPixel, compressionMethod, imageDataSize, colorTableEntries, dataOffset);

  if (width != 800 || abs(height) != 480 || bitsPerPixel != 1 || imageDataSize != 48000 || colorTableEntries != 2)
    return BMP_BAD_SIZE;
 
  
  // Check if there's a color table
  if (dataOffset > 54)
  {
    // Read color table entries
    uint32_t colorTableSize = colorTableEntries * 4; // Each color entry is 4 bytes

    // Display color table
    Log_info("Color table");
    for (uint32_t i = 0; i < colorTableSize; i += 4)
    {
      Log_info("Color %d: B-%d, R-%d, G-%d, A-%d", i / 4 + 1, data[54 + i], data[55 + i], data[56 + i], data[57 + i]);
    }

    return BMP_NO_ERR;
  }
  else
  {
    return BMP_INVALID_OFFSET;
  }
}