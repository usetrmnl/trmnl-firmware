#ifdef LIVE_MODE
#include <PNGdec.h>
#include <display.h>
#include <live_frame.h>
#include <live_region.h>
#include <globals.h>
#include <trmnl_log.h>

#include <Arduino.h>

extern uint32_t live_draw_ms;
extern int live_refresh_mode;

namespace {
uint8_t *screenFrame = nullptr; // a copy of the 1-bit frame on screen, rows (width + 7) / 8 bytes, 1 = white
bool screenFrameValid = false;
uint32_t partialsSinceFull = 0;

struct RowTarget {
  uint8_t *row;
  bool ok;
};

int composeRow(PNGDRAW *draw) {
  RowTarget *target = (RowTarget *)draw->pUser;
  const uint8_t *palette = draw->iPixelType == PNG_PIXEL_INDEXED ? draw->pPalette : nullptr;
  if (!liveRegionRowTo1Bit(draw->pPixels, draw->iWidth, draw->iBpp, palette, target->row)) {
    target->ok = false;
    return 0;
  }
  liveRegionComposeRow(screenFrame, display_width(), 0, draw->y, target->row, draw->iWidth);
  return 1;
}

bool composePng(uint8_t *png, size_t size) {
  PNG *decoder = new PNG();
  uint8_t *row = (uint8_t *)malloc((display_width() + 7) / 8);
  RowTarget target = {.row = row, .ok = decoder && row};
  if (target.ok && decoder->openRAM(png, size, composeRow) == PNG_SUCCESS) {
    if (decoder->decode(&target, 0) != PNG_SUCCESS) target.ok = false;
    decoder->close();
  } else {
    target.ok = false;
  }
  free(row);
  delete decoder;
  return target.ok;
}

PartialWaveform liveWaveform() {
  const ApiDisplayResponse &r = apiDisplayResult.response;
  return PartialWaveform{.drive = LIVE_REGION_DRIVE_FRAMES,
                         .keep = (uint8_t)(r.live_keep >= 0 ? r.live_keep : LIVE_REGION_KEEP_FRAMES),
                         .vcom = (uint8_t)(r.live_vcom >= 0 ? r.live_vcom : LIVE_REGION_VCOM),
                         .otp = false,
                         .pll = LIVE_REGION_PLL,
                         .tcon = LIVE_REGION_TCON,
                         .border = true,
                         .balanced = r.live_balanced};
}
} // namespace

bool live_frame_draw(uint8_t *png, size_t size) {
  const uint32_t started = millis();
  PngHeader header;
  if (!readPngHeader(png, size, header) || !liveRegionPngSupported(header) || header.width != display_width() ||
      header.height != display_height() || !display_window_ready())
    return false;
  const size_t frameSize = (size_t)((display_width() + 7) / 8) * display_height();
  if (!screenFrame) screenFrame = (uint8_t *)malloc(frameSize);
  if (!screenFrame) return false;

  const int rows = display_height();
  const bool full = liveRegionRefresh(partialsSinceFull, false, false) == LiveRegionRefresh::Full;
  if (apiDisplayResult.response.live_stock) screenFrameValid = false;
  if (!screenFrameValid || full) {
    // The usual (balanced) refresh draws it; keep a copy so the next frames can drive only what changes
    screenFrameValid = composePng(png, size);
    partialsSinceFull = 0;
    return false;
  }
  DitherHold none = {};
  if (!display_window_begin(screenFrame, 0, rows, none)) return false;
  if (!composePng(png, size)) {
    screenFrameValid = false; // the copy is half new; the usual path redraws the whole screen
    return false;
  }
  display_window_refresh(screenFrame, 0, rows, liveWaveform());
  partialsSinceFull++;
  const bool known = true;
  live_draw_ms = millis() - started;
  live_refresh_mode = known ? 3 : 4; // 3 = changed pixels only, 4 = every pixel driven
  Log_info("Live frame drawn in %u ms (%s), %u partials since a full one", live_draw_ms, known ? "changes" : "all",
           partialsSinceFull);
  return true;
}
#endif
