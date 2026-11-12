#include <epd_driver.h>
#include <live_frame.h>

#ifdef BB_EPAPER
#include <Arduino.h>
#include <PNGdec.h>
#include <display.h>
#include <live_frame_png.h>
#include <string.h>
#include <trmnl_log.h>

namespace {
  uint8_t *onScreen = nullptr; // the 1-bit frame on screen, rows (width + 7) / 8 bytes, 1 = white
  uint32_t drawsWhenOnScreen = UINT32_MAX; // the panel's draw count while onScreen is what it shows
  char onScreenName[64] = "";
  uint32_t decodeX = 0, decodeY = 0; // where decoded rows go in onScreen

  bool onScreenKnown() { return display_panel_draws() == drawsWhenOnScreen; }

  void remember(uint32_t draws, const char *name) {
    drawsWhenOnScreen = draws;
    strlcpy(onScreenName, name, sizeof(onScreenName));
  }

  int decodeRow(PNGDRAW *draw) {
    const uint8_t *palette = draw->iPixelType == PNG_PIXEL_INDEXED ? draw->pPalette : nullptr;
    const uint32_t pitch = (display_width() + 7) / 8;
    liveFrameRow(draw->pPixels, draw->iWidth, palette, onScreen + (decodeY + draw->y) * pitch + decodeX / 8);
    return 1;
  }

  bool decodeInto(uint8_t *png, size_t size, uint32_t x = 0, uint32_t y = 0) {
    decodeX = x, decodeY = y;
    PNG *decoder = new PNG();
    const bool decoded = decoder && decoder->openRAM(png, size, decodeRow) == PNG_SUCCESS &&
                         decoder->decode(nullptr, 0) == PNG_SUCCESS;
    if (decoder) decoder->close();
    delete decoder;
    return decoded;
  }
} // namespace

bool live_frame_draw(uint8_t *png, size_t size, const char *name) {
  if (!display_live_supported() || !liveFramePngDrawable(png, size, display_width(), display_height())) return false;
  if (!onScreen) onScreen = (uint8_t *)malloc((size_t)((display_width() + 7) / 8) * display_height());
  if (!onScreen) return false;
  const uint32_t started = millis();
  if (!onScreenKnown() || display_full_refresh_due() ||
      !display_live_begin(onScreen, 0, 0, display_width(), display_height())) {
    // the usual draw that follows shows it; the next frame starts from this copy
    remember(decodeInto(png, size) ? display_panel_draws() + 1 : UINT32_MAX, name);
    return false;
  }
  if (!decodeInto(png, size)) {
    display_live_cancel();
    drawsWhenOnScreen = UINT32_MAX;
    return false;
  }
  display_live_finish(onScreen);
  remember(display_panel_draws(), name);
  Log_info("Live frame drawn in %u ms", millis() - started);
  return true;
}

bool live_frame_shows(const char *name) { return onScreen && onScreenKnown() && strcmp(name, onScreenName) == 0; }

bool live_frame_draw_patch(uint8_t *png, size_t size, uint32_t x, uint32_t y, const char *name) {
  uint32_t width, height;
  const uint32_t started = millis();
  if (!onScreen || !onScreenKnown() || display_full_refresh_due() ||
      !liveFramePatchFits(png, size, x, y, display_width(), display_height(), &width, &height) ||
      !display_live_begin(onScreen, x, y, width, height))
    return false;
  if (!decodeInto(png, size, x, y)) { // rows decoded before the error are already in onScreen
    display_live_cancel();
    drawsWhenOnScreen = UINT32_MAX;
    return false;
  }
  display_live_finish(onScreen);
  remember(display_panel_draws(), name);
  Log_info("Live patch %ux%u at %u,%u drawn in %u ms", width, height, x, y, millis() - started);
  return true;
}
#else
bool live_frame_draw(uint8_t *, size_t, const char *) { return false; }
bool live_frame_shows(const char *) { return false; }
bool live_frame_draw_patch(uint8_t *, size_t, uint32_t, uint32_t, const char *) { return false; }
#endif
