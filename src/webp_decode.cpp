#include "webp_decode.h"

#include <Arduino.h>
#include <stdlib.h>

enum
{
    WEBP_ERR_BAD_SIZE = -3,
    WEBP_ERR_DECODE = -2,
    WEBP_ERR_MALLOC = -1,
    WEBP_OK = 0,
};

#if defined(BOARD_X_CLASS)

#include "FastEPD.h"
#include "config.h"
#include "trmnl_log.h"
#include "webp/decode.h"
#include "webp/types.h"

extern FASTEPD bbep;

// Pack Y (luma) plane directly into 4-bpp framebuffer.
// Much cheaper than RGBA: no alpha blend, no RGB→gray conversion needed.
static void webp_y_to_4bpp_plane(const uint8_t *y_plane, int y_stride,
                                 int w, int h, uint8_t *pBuffer, int iPitch)
{
    for (int y = 0; y < h; y++)
    {
        const uint8_t *s = y_plane + (size_t)y * y_stride;
        uint8_t *d = &pBuffer[y * iPitch];
        int x = 0;
        for (; x + 1 < w; x += 2)
        {
            uint8_t n0 = s[x] >> 4;
            uint8_t n1 = s[x + 1] >> 4;
            *d++ = (uint8_t)((n0 << 4) | n1);
        }
        if (x < w)
            *d = (uint8_t)((s[x] >> 4) << 4);
    }
}

#endif /* BOARD_X_CLASS */

extern "C" int is_webp_buffer(const uint8_t *data, int size)
{
    if (!data || size < 12)
        return 0;
    if (data[0] != 'R' || data[1] != 'I' || data[2] != 'F' || data[3] != 'F')
        return 0;
    if (data[8] != 'W' || data[9] != 'E' || data[10] != 'B' || data[11] != 'P')
        return 0;
    return 1;
}

extern "C" int webp_to_epd(const uint8_t *data, int size, int decode_to_previous)
{
#if !defined(BOARD_X_CLASS)
    (void)data;
    (void)size;
    (void)decode_to_previous;
    return WEBP_ERR_DECODE;
#else
#ifndef BB_EPAPER
    if (decode_to_previous && bbep.getPreviousMode() != BB_MODE_NONE)
        return WEBP_OK;
#endif
    int w = 0, h = 0;
    if (!WebPGetInfo(data, (size_t)size, &w, &h))
    {
        Log_error("%s [%d]: WebPGetInfo failed\r\n", __FILE__, __LINE__);
        return WEBP_ERR_DECODE;
    }
    if (w != (int)bbep.width() || h != (int)bbep.height())
    {
        Log_error("%s [%d]: WebP size %dx%d, need %dx%d\r\n", __FILE__, __LINE__, w, h,
                  bbep.width(), bbep.height());
        return WEBP_ERR_BAD_SIZE;
    }

    WebPBitstreamFeatures feat;
    VP8StatusCode st = WebPGetFeatures(data, (size_t)size, &feat);
    if (st != VP8_STATUS_OK)
    {
        Log_error("%s [%d]: WebPGetFeatures status %d\r\n", __FILE__, __LINE__, (int)st);
        return WEBP_ERR_DECODE;
    }
    if (feat.has_animation)
    {
        Log_error("%s [%d]: Animated WebP not supported\r\n", __FILE__, __LINE__);
        return WEBP_ERR_DECODE;
    }

    // Decode to YUV and use only the Y (luma) plane.
    // Memory: Y = w*h, U = V = (w/2)*(h/2) ≈ 1.5x w*h total.
    // For 1872x1404 that's ~3.9 MB vs ~10.5 MB for RGBA.
    const int uv_w = (w + 1) / 2;
    const int uv_h = (h + 1) / 2;
    const size_t y_size  = (size_t)w * h;
    const size_t uv_size = (size_t)uv_w * uv_h;
    const size_t total   = y_size + 2 * uv_size;

#ifdef CONFIG_SPIRAM
    uint8_t *yuv_buf = (uint8_t *)ps_malloc(total);
#else
    uint8_t *yuv_buf = (uint8_t *)malloc(total);
#endif
    if (!yuv_buf)
    {
        Log_error("%s [%d]: WebP YUV buffer alloc failed (%u bytes)\r\n", __FILE__, __LINE__,
                  (unsigned)total);
        return WEBP_ERR_MALLOC;
    }

    uint8_t *y_plane = yuv_buf;
    uint8_t *u_plane = yuv_buf + y_size;
    uint8_t *v_plane = u_plane + uv_size;

    if (!WebPDecodeYUVInto(data, (size_t)size,
                           y_plane, y_size, w,
                           u_plane, uv_size, uv_w,
                           v_plane, uv_size, uv_w))
    {
        Log_error("%s [%d]: WebPDecodeYUVInto failed\r\n", __FILE__, __LINE__);
        free(yuv_buf);
        return WEBP_ERR_DECODE;
    }

    const int iPitch = bbep.width() / 2;
    if (decode_to_previous)
    {
        bbep.setPreviousMode(BB_MODE_4BPP);
        webp_y_to_4bpp_plane(y_plane, w, w, h, bbep.previousBuffer(), iPitch);
        Log_info("%s [%d]: WebP decoded to previous 4-bpp buffer\r\n", __FILE__, __LINE__);
    }
    else
    {
        bbep.setMode(BB_MODE_4BPP);
        webp_y_to_4bpp_plane(y_plane, w, w, h, bbep.currentBuffer(), iPitch);
        Log_info("%s [%d]: WebP decoded to 4-bpp framebuffer\r\n", __FILE__, __LINE__);
    }

    free(yuv_buf);
    return WEBP_OK;
#endif
}
