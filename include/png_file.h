#include <cstdint>
#include <png.h>

image_err_e decodePNG(const char *szFilename, uint8_t *&decoded_buffer);
