#ifdef BOARD_X_CLASS

#include "fastepd.h"

#include <Arduino.h>
#include <trmnl_log.h>

#include "DEV_Config.h"
#include "config.h"

FASTEPD bbep;

#ifdef BOARD_TRMNL_X
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
static const uint16_t u8_phase_timing[] = {
	50,	50,	50, 50, 50, 50, 50, 50, 50, 50,
	50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
	50, 100, 180, 260, 340, 420, 500, 580, 660, 740,
  820, 1000, 1500, 2000, 2500, 3000, 1500, 500
};
// clang-format on
#else
// Stock 9-pass 4bpp waveform used by non-TRMNL_X FastEPD boards.
// clang-format off
static const uint8_t u8_graytable[] = {
/* 0 */  0, 0, 0, 0, 0, 0, 1, 1, 1,
/* 1 */  0, 0, 1, 1, 1, 2, 2, 1, 1,
/* 2 */  0, 0, 0, 0, 1, 2, 2, 1, 1,
/* 3 */  1, 1, 2, 2, 1, 1, 1, 1, 2,
/* 4 */  0, 0, 0, 1, 2, 1, 1, 1, 2,
/* 5 */  1, 2, 2, 2, 2, 1, 1, 1, 2,
/* 6 */  0, 0, 1, 1, 2, 2, 1, 1, 2,
/* 7 */  0, 1, 1, 2, 1, 1, 2, 1, 2,
/* 8 */  0, 1, 1, 1, 2, 1, 2, 1, 2,
/* 9 */  0, 1, 1, 1, 1, 2, 2, 1, 2,
/* 10 */ 1, 1, 1, 2, 1, 1, 1, 2, 2,
/* 11 */ 0, 0, 1, 2, 1, 1, 1, 2, 2,
/* 12 */ 0, 0, 0, 1, 2, 1, 1, 2, 2,
/* 13 */ 0, 0, 0, 0, 1, 2, 1, 2, 2,
/* 14 */ 0, 1, 1, 1, 2, 2, 2, 2, 2,
/* 15 */ 0, 0, 0, 0, 0, 0, 0, 0, 2
};
// clang-format on
#endif

namespace fastepd {

  void init() {
#ifdef BOARD_TRMNL_X
    bbep.initPanel(BB_PANEL_TRMNL_X, 40000000);
    bbep.setPasses(3, 3);
#elif defined(BOARD_TRMNL_X_SENSORIAS3)
    bbep.initPanel(BB_PANEL_V7_RAW);
    bbep.setPanelSize(1280, 720, BB_PANEL_FLAG_MIRROR_X, -1600);
#elif defined(BOARD_TRMNL_X_SENSORIAC5)
    bbep.initPanel(BB_PANEL_SENSORIA_C5);
#elif defined(BOARD_TRMNL_X_PAPERS3)
    bbep.initPanel(BB_PANEL_M5PAPERS3);
#elif defined(BOARD_TRMNL_X_LILYGO)
    bbep.initPanel(BB_PANEL_EPDIY_V7);
    bbep.setPanelSize(960, 540);
#elif defined(BOARD_SEEED_RETERMINAL_E1003)
    bbep.initIT8951(EPD_MOSI_PIN, EPD_MISO_PIN, EPD_SCK_PIN, EPD_CS_PIN, EPD_BUSY_PIN, EPD_RST_PIN, EPD_EN_PIN,
                    EPD_VCC_EN);
    bbep.setPanelSize(BBEP_DISPLAY_ED103TC2);
#endif
  }

  void update(bool bWait, bool bSkipClear, int iUpdateCount, uint32_t iTempProfile) {
    (void)bWait;
    int rc = bbep.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    Log_info("%s [%d]: setCustomMatrix returned %d\r\n", __FILE__, __LINE__, rc);
#ifdef BOARD_TRMNL_X
    bbep.setPhaseTiming(u8_phase_timing);
#endif

    // bWait=false means loading screen: skip clearing passes so it appears
    // faster. Ghosting from the previous image is acceptable since the
    // real content refresh (bWait=true) immediately follows.
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

} // namespace fastepd

#endif // BOARD_X_CLASS
