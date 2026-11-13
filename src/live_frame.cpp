#include <epd_driver.h>
#include <live_frame.h>

#ifdef BB_EPAPER
#include <Arduino.h>
#include <PNGdec.h>
#include <new>
#include <display.h>
#include <live_frame_png.h>
#include <trmnl_log.h>

namespace {
  uint8_t *onScreen = nullptr; // the 1-bit frame on screen, rows (width + 7) / 8 bytes, 1 = white
  uint32_t drawsWhenOnScreen = UINT32_MAX; // the panel's draw count while onScreen is what it shows

  bool onScreenKnown() { return display_panel_draws() == drawsWhenOnScreen; }

  int decodeRow(PNGDRAW *draw) {
    const uint8_t *palette = draw->iPixelType == PNG_PIXEL_INDEXED ? draw->pPalette : nullptr;
    liveFrameRow(draw->pPixels, draw->iWidth, palette, onScreen + draw->y * ((display_width() + 7) / 8));
    return 1;
  }

  bool decodeInto(uint8_t *png, size_t size) {
    PNG *decoder = new (std::nothrow) PNG();
    const bool decoded = decoder && decoder->openRAM(png, size, decodeRow) == PNG_SUCCESS &&
                         decoder->decode(nullptr, 0) == PNG_SUCCESS;
    if (decoder) decoder->close();
    delete decoder;
    return decoded;
  }
} // namespace

bool live_frame_draw(uint8_t *png, size_t size) {
  if (!display_live_supported() || !liveFramePngDrawable(png, size, display_width(), display_height())) {
    free(onScreen); // the usual draw makes the copy stale and needs the room for its decoder
    onScreen = nullptr;
    drawsWhenOnScreen = UINT32_MAX;
    return false;
  }
  if (!onScreen) onScreen = (uint8_t *)malloc((size_t)((display_width() + 7) / 8) * display_height());
  if (!onScreen) return false;
  if (!onScreenKnown() || display_full_refresh_due() || !display_live_begin(onScreen)) {
    // the usual draw that follows shows it; the next frame starts from this copy
    drawsWhenOnScreen = decodeInto(png, size) ? display_panel_draws() + 1 : UINT32_MAX;
    return false;
  }
  const uint32_t started = millis();
  if (!decodeInto(png, size)) {
    display_live_cancel();
    drawsWhenOnScreen = UINT32_MAX;
    return false;
  }
  display_live_finish(onScreen);
  drawsWhenOnScreen = display_panel_draws();
  Log_info("Live frame drawn in %u ms", millis() - started);
  return true;
}
#else
bool live_frame_draw(uint8_t *, size_t) { return false; }
#endif
