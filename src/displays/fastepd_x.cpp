#ifdef BOARD_TRMNL_X

#include "fastepd_x.h"

#include <Arduino.h>
#include <string.h>
#include <trmnl_log.h>

#include "esp_heap_caps.h"

FASTEPDX bbep;

// Hybrid GC16-derived 4bpp waveform: 16 levels x 38 passes, level-major.
// clang-format off
static const uint8_t u8_graytable[] = {
/*  0 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
/*  1 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
/*  2 */ 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 0,
/*  3 */ 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 2, 0,
/*  4 */ 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0,
/*  5 */ 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 2, 2, 0,
/*  6 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 2, 2, 0, 0,
/*  7 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 0, 2, 2, 0,
/*  8 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 0, 0, 2, 2, 0,
/*  9 */ 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 0,
/* 10 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 0, 0, 0, 2, 2, 2, 0,
/* 11 */ 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 0,
/* 12 */ 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 0, 0, 0, 0, 0, 2, 2, 2, 2, 0,
/* 13 */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 2, 2, 2, 2, 2, 2, 0, 0,
/* 14 */ 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0,
/* 15 */ 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0
};

// Per-pass frame hold (us).
static const uint16_t u8_timing_official_38[] = {
	50,	50,	50, 50, 50, 50, 50, 50, 50, 50,
	50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
	50, 100, 180, 260, 340, 420, 500, 580, 660, 740,
  820, 1000, 1500, 2000, 2500, 3000, 1500, 500
};
// clang-format on

// FastEPD internals, linked from the library.
int bbepEinkPower(FASTEPDSTATE *pState, int bOn);
void bbepClear(FASTEPDSTATE *pState, uint8_t val, uint8_t count, BB_RECT *pRect);
void bbepRowControl(FASTEPDSTATE *pState, int iType);
void bbepWriteRow(FASTEPDSTATE *pState, uint8_t *pData, int iLen, int bRowStep);
extern volatile bool dma_is_done;

static uint8_t *u8FastLUT;
#define X_RING_SLOTS         8
// De-ghost oscillations before each draw (each = 8 darken + 8 lighten passes).
#define X_DEGHOST_CYCLES     2
// Extra net-lighten passes so this dark-biased panel reaches true white before
// the from-white draw; a balanced clear settles on a gray floor that ghosts.
#define X_DEGHOST_WHITE_PUSH 12
static uint8_t *u8RingBuf;
static volatile uint32_t g_produced;
static volatile uint32_t g_consumed;
static TaskHandle_t s_feederTask;
static TaskHandle_t s_mainTask;
static FASTEPDSTATE *g_ps;
static int g_iRowBytes, g_iSrcWords, g_rows, g_iRowPSRAM;
static bool g_bMirror;

static void x_build_pass_lut(const uint8_t *pMatrix, int iPasses, int pass, bool bMirrorX, uint8_t *lut) {
  uint8_t u8U[256], u8L[256];
  for (int i = 0; i < 256; i++) {
    uint8_t lo;
    if (bMirrorX) {
      lo = (pMatrix[(i & 0xf) * iPasses + pass] << 2) | pMatrix[(i >> 4) * iPasses + pass];
    } else {
      lo = (pMatrix[(i >> 4) * iPasses + pass] << 2) | pMatrix[(i & 0xf) * iPasses + pass];
    }
    u8L[i] = lo;
    u8U[i] = lo << 4;
  }
  for (int hi = 0; hi < 256; hi++) {
    uint8_t *row = &lut[hi << 8];
    if (bMirrorX) {
      uint8_t u = u8U[hi];
      for (int lo = 0; lo < 256; lo++)
        row[lo] = u | u8L[lo];
    } else {
      uint8_t l = u8L[hi];
      for (int lo = 0; lo < 256; lo++)
        row[lo] = u8U[lo] | l;
    }
  }
}

static inline void x_convert_row(int y, uint8_t *d) {
  uint16_t row_buf[512];
  memcpy(row_buf, &g_ps->pCurrent[y * g_iRowPSRAM], g_iRowPSRAM);
  const uint32_t *w = (const uint32_t *)row_buf;
  const int nWords = g_iSrcWords >> 1;
  if (g_bMirror) {
    int di = 0;
    for (int j = nWords - 1; j >= 0; j--) {
      uint32_t v = w[j];
      d[di++] = u8FastLUT[v >> 16];
      d[di++] = u8FastLUT[v & 0xFFFF];
    }
  } else {
    for (int j = 0; j < nWords; j++) {
      uint32_t v = w[j];
      d[(j << 1)] = u8FastLUT[v & 0xFFFF];
      d[(j << 1) + 1] = u8FastLUT[v >> 16];
    }
  }
}

static void x_feeder_task(void *arg) {
  (void)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (uint32_t y = 0; y < (uint32_t)g_rows; y++) {
      while (g_produced - g_consumed >= X_RING_SLOTS - 1) {
      }
      x_convert_row(y, &u8RingBuf[(y % X_RING_SLOTS) * g_iRowBytes]);
      __sync_synchronize();
      g_produced = y + 1;
    }
    xTaskNotifyGive(s_mainTask);
  }
}

// Yielding 4bpp render for the 38-pass from-white waveform. Stock fullUpdate()
// runs every pass in one CPU hold (~seconds), tripping the interrupt watchdog,
// and its clear cadence under-clears the from-white waveform. This does a full
// de-ghost, then feeds rows from core 0 and blocks once per pass so the
// watchdog stays fed.
static void display_fast_gray_update(void) {
  FASTEPDSTATE *ps = bbep.state();
  const int iPasses = (int)(sizeof(u8_graytable) / 16);
  const int iRowBytes = ps->native_width / 4;
  const int iSrcWords = ps->native_width / 4;
  const bool bMirror = (ps->iFlags & BB_PANEL_FLAG_MIRROR_X) != 0;

  if (!u8FastLUT) {
    u8FastLUT = (uint8_t *)heap_caps_malloc(65536, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!u8FastLUT) {
      Log_info("fast4bpp: 64KB LUT alloc failed, falling back to fullUpdate");
      bbep.fullUpdate(CLEAR_SLOW, false);
      return;
    }
  }

  unsigned long t0 = micros();
  bbepEinkPower(ps, 1);
  for (int c = 0; c < X_DEGHOST_CYCLES; c++) {
    bbepClear(ps, BB_CLEAR_DARKEN, 8, NULL);
    bbepClear(ps, BB_CLEAR_LIGHTEN, 8, NULL);
  }
  bbepClear(ps, BB_CLEAR_LIGHTEN, X_DEGHOST_WHITE_PUSH, NULL);
  bbepClear(ps, BB_CLEAR_NEUTRAL, 1, NULL);
  unsigned long tClear = micros() - t0;

  const int iRowPSRAM = ps->native_width / 2;
  g_ps = ps;
  g_iRowBytes = iRowBytes;
  g_iSrcWords = iSrcWords;
  g_rows = ps->native_height;
  g_iRowPSRAM = iRowPSRAM;
  g_bMirror = bMirror;
  if (!u8RingBuf) {
    u8RingBuf =
      (uint8_t *)heap_caps_aligned_alloc(4, (size_t)X_RING_SLOTS * iRowBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  }
  if (!u8RingBuf) {
    Log_info("fast4bpp: ring alloc failed, falling back to fullUpdate");
    bbepEinkPower(ps, 0);
    bbep.fullUpdate(CLEAR_SLOW, false);
    return;
  }
  if (!s_feederTask) {
    s_mainTask = xTaskGetCurrentTaskHandle();
    // Feeder on core 0; Arduino main (consumer) runs on core 1.
    xTaskCreatePinnedToCore(x_feeder_task, "epd_feed", 4096, NULL, configMAX_PRIORITIES - 2, &s_feederTask, 0);
  }

  unsigned long tGray0 = micros();
  for (int pass = 0; pass < iPasses; pass++) {
    x_build_pass_lut(u8_graytable, iPasses, pass, bMirror, u8FastLUT);
    g_produced = 0;
    g_consumed = 0;
    bbepRowControl(ps, ROW_START);
    xTaskNotifyGive(s_feederTask);
    for (int y = 0; y < ps->native_height; y++) {
      while (g_produced <= (uint32_t)y) {
      }
      __sync_synchronize();
      while (!dma_is_done) {
      }
      uint8_t *d = &u8RingBuf[(y % X_RING_SLOTS) * iRowBytes];
      bbepWriteRow(ps, d, iRowBytes, (y != 0));
      g_consumed = y + 1;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    delayMicroseconds(u8_timing_official_38[pass]);
  }
  unsigned long tGray = micros() - tGray0;
  bbepClear(ps, BB_CLEAR_NEUTRAL, 1, NULL);
  bbepEinkPower(ps, 0);
  Log_info("fast4bpp: clear %lu us, %d gray frames in %lu us (%lu us/frame; design 11765)", tClear, iPasses, tGray,
           tGray / (unsigned long)iPasses);
}

namespace fastepd_x {
  void init() {
    // 40MHz shortens the frame so the 38-pass waveform doesn't overshoot highlights to white.
    bbep.initPanel(BB_PANEL_TRMNL_X, 40000000);
    bbep.setPasses(3, 3);
  }

  void update(bool bWait, bool bSkipClear, int iUpdateCount, uint32_t iTempProfile) {
    int rc = bbep.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    Log_info("%s [%d]: setCustomMatrix returned %d\r\n", __FILE__, __LINE__, rc);

    if (bbep.getMode() == BB_MODE_4BPP && bWait) {
      display_fast_gray_update();
      return;
    }

    // bWait=false is the loading screen: skip clears for speed; brief ghosting
    // is fine since the real refresh (bWait=true) follows immediately.
    int iClearMode;
    if (bSkipClear) {
      iClearMode = CLEAR_NONE;
    } else if ((iUpdateCount & 7) == 0 || iTempProfile > 0) {
      iClearMode = CLEAR_SLOW;
    } else {
      iClearMode = CLEAR_FAST;
    }
    Log_info("fullUpdate clear mode = %d\n", iClearMode);
    bbep.fullUpdate(iClearMode, false);
  }

} // namespace fastepd_x

#endif // BOARD_TRMNL_X
