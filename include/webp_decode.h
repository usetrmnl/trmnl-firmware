#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** True if buffer looks like a RIFF/WEBP container. */
int is_webp_buffer(const uint8_t *data, int size);

/**
 * Decode WebP into the FastEPD framebuffer (4-bpp) when `BOARD_X_CLASS` is defined.
 * On other boards returns WEBP_ERR_DECODE (server should not send WebP there).
 * @param decode_to_previous 0 = current buffer; non-zero = previous buffer (partial updates).
 * @return 0 on success, negative WEBP_ERR_* codes on failure.
 */
int webp_to_epd(const uint8_t *data, int size, int decode_to_previous);

#ifdef __cplusplus
}
#endif
