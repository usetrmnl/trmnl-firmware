#include <epd_driver.h>
#include <live_frame.h>

#ifdef BB_EPAPER
#include <Arduino.h>
#include <PNGdec.h>
#include <display.h>
#include <esp_heap_caps.h>
#include <live_frame_png.h>
#include <trmnl_log.h>

namespace {
  // A 2-bit copy of the screen is 96 KB, which leaves no block for PNGdec's 49 KB beside live mode's connection, so
  // the copy is packed as runs; a frame too busy to pack in the 1-bit copy's 48 KB draws the usual way.
  const size_t COPY_CAPACITY = (800 / 8) * 480;
  uint8_t *onScreen = nullptr; // the frame on screen, packed rows (liveFrameRowPack)
  uint32_t drawsWhenOnScreen = UINT32_MAX; // the panel's draw count while onScreen is what it shows

  // What a decode of the next frame does with each row: sends one plane of a pass, or packs it into a new copy
  struct Decode {
    int pass, plane;
    uint8_t *packed;
    size_t read, written;
    bool gray, full;
  } decode;

  bool onScreenKnown() { return onScreen && display_panel_draws() == drawsWhenOnScreen; }

  int decodeRow(PNGDRAW *draw) {
    const uint8_t *palette = draw->iPixelType == PNG_PIXEL_INDEXED ? draw->pPalette : nullptr;
    uint8_t levels[800 / 4], was[800 / 4], plane[800 / 8];
    decode.gray |= liveFrameRow(draw->pPixels, draw->iWidth, draw->iBpp, palette, levels);
    if (decode.plane) {
      decode.read += liveFrameRowUnpack(onScreen + decode.read, draw->iWidth, was);
      liveFramePlaneRow(was, levels, draw->iWidth, decode.pass, decode.plane, plane);
      display_live_row(plane);
    } else if (!decode.full) {
      const size_t n = liveFrameRowPack(levels, draw->iWidth, decode.packed + decode.written,
                                        COPY_CAPACITY - decode.written);
      decode.full = n == 0;
      decode.written += n;
    }
    return 1;
  }

  // PNGdec decodes a row at a time, so each plane decodes the PNG again rather than hold a second frame in memory
  bool decodeAs(uint8_t *png, size_t size, int pass, int plane) {
    decode.pass = pass, decode.plane = plane, decode.read = 0;
    if (plane) display_live_plane(plane);
    PNG *decoder = new PNG();
    const bool decoded = decoder && decoder->openRAM(png, size, decodeRow) == PNG_SUCCESS &&
                         decoder->decode(nullptr, 0) == PNG_SUCCESS;
    if (decoder) decoder->close();
    delete decoder;
    return decoded;
  }

  // Replaces the copy with this frame's; false when it can't, which leaves none
  bool keepCopy(uint8_t *png, size_t size) {
    free(onScreen);
    onScreen = nullptr;
    decode.packed = (uint8_t *)malloc(COPY_CAPACITY), decode.written = 0, decode.full = false;
    if (!decode.packed) return false;
    if (!decodeAs(png, size, 0, 0) || decode.full) {
      free(decode.packed);
      return false;
    }
    onScreen = (uint8_t *)realloc(decode.packed, decode.written);
    return true;
  }

  bool drawPasses(uint8_t *png, size_t size) {
    decode.gray = false;
    if (!decodeAs(png, size, 1, 1) || !decodeAs(png, size, 1, 2)) return false;
    display_live_refresh(1, decode.gray);
    if (!decode.gray) return true;
    if (!decodeAs(png, size, 2, 1) || !decodeAs(png, size, 2, 2)) return false;
    display_live_refresh(2, true);
    return true;
  }
} // namespace

bool live_frame_draw(uint8_t *png, size_t size) {
  if (!display_live_supported() || !liveFramePngDrawable(png, size, display_width(), display_height())) return false;
  if (!onScreenKnown() || display_full_refresh_due() || !display_live_begin()) {
    // the usual draw that follows shows it; the next frame starts from this copy
    drawsWhenOnScreen = keepCopy(png, size) ? display_panel_draws() + 1 : UINT32_MAX;
    return false;
  }
  Log_info("Live heap: free %u, largest block %u", heap_caps_get_free_size(MALLOC_CAP_8BIT),
           heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  const uint32_t started = millis();
  const bool drawn = drawPasses(png, size);
  display_live_finish(drawn);
  const uint32_t drawnAt = millis();
  drawsWhenOnScreen = drawn && keepCopy(png, size) ? display_panel_draws() : UINT32_MAX;
  if (drawn)
    Log_info("Live frame drawn in %u ms (%s), copy packed in %u bytes, %u ms", drawnAt - started,
             decode.gray ? "gray" : "black and white", decode.written, millis() - drawnAt);
  return drawn;
}
#else
bool live_frame_draw(uint8_t *, size_t) { return false; }
#endif
