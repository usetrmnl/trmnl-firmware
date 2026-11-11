#include <Arduino.h>
#include <JPEGDEC.h>
#include <PNGdec.h>
#include <Preferences.h>
#include <WiFi.h>
#include <board_config.h>
#include <color_reduce.h>
#include <display.h>
#include <display_internal.h>
#include <display_widgets.h>
#include <epd_panel_rev.h>
#include <jpeg_render.h>
#include <png_render.h>
#include <power.h>
#include <preferences_persistence.h>
#include <refresh_interval.h>

#include "battery.h"
#include "config.h"
#include "messages.h"
#include <live_region.h>
#ifdef PARALLEL_EPD
FASTEPD bbep;
#else
BBEPAPER bbep;
#endif // !PARALLEL_EPD

#ifdef PARALLEL_EPD
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

// clang-format off
const uint8_t u8_graytable[] = {
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
/* 10 */  1, 1, 1, 2, 1, 1, 1, 2, 2, 
/* 11 */  0, 0, 1, 2, 1, 1, 1, 2, 2, 
/* 12 */  0, 0, 0, 1, 2, 1, 1, 2, 2, 
/* 13 */  0, 0, 0, 0, 1, 2, 1, 2, 2, 
/* 14 */  0, 1, 1, 1, 2, 2, 2, 2, 2, 
/* 15 */  0, 0, 0, 0, 0, 0, 0, 0, 2
};
// clang-format on
#endif

#include <api-client/display.h>
#include <config.h>
#include <ctype.h> //iscntrl()
#include <globals.h>
#include <inttypes.h>
#include <trmnl_log.h>

#include "Group5.h"
#include "bmp.h"
#include "png_flip.h"

#ifdef BB_EPAPER
static uint8_t bLightSleepWanted = 0;
static int iLastRefreshMode = REFRESH_FULL;

static void display_apply_light_sleep(uint8_t enabled) {
#ifdef CMD_CS1_CS2
  bbep.setLightSleep(enabled, pDevice->epd_en_pin);
#else
  bbep.setLightSleep(enabled);
#endif
}

bool display_update_epaper(int refreshMode, bool wait, bool writePlane, uint8_t plane) {
#if defined(BOARD_SEEED_RETERMINAL_E1002) || defined(BOARD_SEEED_RETERMINAL_E1004) || defined(BOARD_TRMNL_GEN2)
    // Light sleep with the WiFi radio running is unsupported
  if (bLightSleepWanted && WiFi.getMode() != WIFI_MODE_NULL) {
    display_apply_light_sleep(false);
  } else {
    display_apply_light_sleep(bLightSleepWanted);
  }
#endif
  if (writePlane) {
    bbep.writePlane(plane);
  }
  if (refreshMode == REFRESH_PARTIAL && !bCanDoPartial) {
    refreshMode = REFRESH_FAST;
    Log_info("Can't do partial refresh, no previous image");
  } else if (refreshMode == REFRESH_PARTIAL) {
    Log_info("Have valid previous image in EPD memory, doing partial refresh");
  }
  bbep.refresh(refreshMode, wait);
  iLastRefreshMode = refreshMode;
    // The next update can be a partial update because the current is 1-bpp and stays in the EPD RAM
  bCanDoPartial = (bbep.getPanelType() == dpList[pDevice->panel_set][iTempProfile].OneBit);
  return true;
}

static void display_select_one_bit_panel(bool only_if_changed = false) {
  bool custom_pins = pDevice->epd_mosi_pin != 0 || pDevice->epd_sck_pin != 0;
  int one_bit = dpList[pDevice->panel_set][custom_pins ? iTempProfile : 0].OneBit;
  if (only_if_changed && bbep.getPanelType() == one_bit) {
    return;
  }
  if (custom_pins) {
    bbep.setPanelType(one_bit);
    bbep.initIO(pDevice->epd_dc_pin, pDevice->epd_rst_pin, pDevice->epd_busy_pin, pDevice->epd_cs_pin,
                pDevice->epd_mosi_pin, pDevice->epd_sck_pin, 8000000);
  } else {
    bbep.begin(one_bit);
  }
}

int display_last_refresh_mode(void)
{
    return iLastRefreshMode;
}

// Message screens are 1-bit
void display_prepare_message(void) {
  display_select_one_bit_panel(true);
  bbep.setMemoryMode(BBEP_MODE_1BPP); // use 1-bit mode to save memory
  bbep.allocBuffer(false);
}

#endif // BB_EPAPER

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void) {
  if (pDevice == NULL) {
    hw_config_init(DEVICE_MODEL);
  }
  Log_info("dev module start");
  iTempProfile = preferences.getUInt(PREFERENCES_TEMP_PROFILE, TEMP_PROFILE_DEFAULT);
  Log_info("Saved temperature profile: %" PRIu32, iTempProfile);
#ifdef BB_EPAPER
  Log_info("BB e-Paper init");
#ifdef BOARD_SEEED_STICKY
// Special case for the Sticky - it shares the SPI bus with the uSD card, so the EPD CS line
// must be the only one active at a time
  pinMode(8, OUTPUT); // SD card CS (shared SPI with EPD)
  digitalWrite(8, 1); // SD CS disabled
  pinMode(47, OUTPUT);
  digitalWrite(47, 1); // enable EPD power
  pinMode(10, OUTPUT); // SD card enable (if it's powered down, the SPI bus may be blocked)
  digitalWrite(10, 1);
#endif // BOARD_SEEED_STICKY
    // Read the panel ID after any board-specific power/CS setup, but before bb_epaper takes over the SPI pins.
    // Only on the first call: the bit-banged read detaches SCK/MOSI from the SPI peripheral, and
    // SPI.begin() is a no-op once the bus is running, so a second read would leave the panel unreachable.
  static bool panel_rev_read = false;
  if (!panel_rev_read) {
    panel_rev = get_panel_rev();
    panel_rev_read = true;
    Log_info("Panel ID = 0x%08x\n", panel_rev);
  }
  if (strcmp(DEVICE_MODEL, "xiao_epaper_mini") == 0 && panel_rev == 0x40000000) {
    // New 4.26" panel needs a different ePaper configuration
    Log_info("Detected newer 4.26 inch panel type; switching profiles...");
    hw_config_init("xiao_epaper_mini_b");
  }
  if (pDevice->epd_mosi_pin != 0 || pDevice->epd_sck_pin != 0) {
    bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit); // must be set BEFORE calling initio
    bbep.initIO(pDevice->epd_dc_pin, pDevice->epd_rst_pin, pDevice->epd_busy_pin, pDevice->epd_cs_pin,
                pDevice->epd_mosi_pin, pDevice->epd_sck_pin, 8000000);
  } else { // it's a pre-defined PCB+display in bb_epaper
    bbep.begin(dpList[pDevice->panel_set][0].OneBit);
  }
#else // Parallel eink devices
#if defined(BOARD_SEEED_RETERMINAL_E1003)
  bbep.initIT8951(EPD_MOSI_PIN, EPD_MISO_PIN, EPD_SCK_PIN, EPD_CS_PIN, EPD_BUSY_PIN, EPD_RST_PIN, EPD_EN_PIN,
                  EPD_VCC_EN);
  bbep.setPanelSize(BBEP_DISPLAY_ED103TC2);
#else // normal parallel eink devices
  bbep.initPanel(pDevice->iBoardType);
  if (pDevice->iPanelSize != BB_PANEL_NONE) {
    bbep.setPanelSize(pDevice->iPanelSize);
  }
#endif
#ifdef BOARD_TRMNL_X
//    bbep.initPanel(BB_PANEL_TRMNL_X);
  bbep.setPasses(3, 3); // The 10.3" panel needs fewer pushes
#endif
// #elif defined( BOARD_TRMNL_X_SENSORIAS3 )
//     bbep.initPanel(BB_PANEL_V7_RAW);
//     bbep.setPanelSize(1280, 720, BB_PANEL_FLAG_MIRROR_X, -1600);
// #elif defined( BOARD_TRMNL_X_SENSORIAC5 )
//     bbep.initPanel(BB_PANEL_SENSORIA_C5);
// #elif defined(BOARD_TRMNL_X_PAPERS3)
//     bbep.initPanel(BB_PANEL_M5PAPERS3);
// #elif defined(BOARD_TRMNL_X_LILYGO)
//     bbep.initPanel(BB_PANEL_EPDIY_V7);
//     bbep.setPanelSize(960, 540);
#endif // bb_epaper
  Log_info("dev module end");
}

/**
 * @brief Enable or disable light sleep at runtime
 * @param enabled true to enable light sleep, false to disable
 * @return none
 */
void display_set_light_sleep(uint8_t enabled) {
#ifdef BB_EPAPER
  bLightSleepWanted = enabled;
  display_apply_light_sleep(enabled);
#endif
}

// Clear any ghosting on the X display by wiping it black/white many times
void display_wipe(void) {
#ifdef BB_EPAPER

#ifdef BOARD_TRMNL_4CLR
  int refreshCount = 2;
#else
  int refreshCount = 60;
#endif

  bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit);
  bbep.fillScreen(BBEP_WHITE);
  for (int i = 0; i < refreshCount; i++) {
    bbep.refresh(REFRESH_FULL); // 2 to 3 minutes of Black/White clearing of the display
  }
  bbep.sleep(LIGHT_SLEEP);
#else
  bbep.setMode(BB_MODE_1BPP);
  bbep.fillScreen(BBEP_WHITE);
  for (int i = 0; i < 100; i++) { // 200 black/white cycles should remove any ghosting
    bbep.fullUpdate(CLEAR_SLOW, true);
  }
  bbep.einkPower(0); // power off the display
#endif
} /* display_wipe() */

/**
 * @brief Function to sleep the ESP32 while saving power
 * @param u32Millis represents the sleep time in milliseconds
 * @return none
 */
void display_sleep(uint32_t u32Millis) {
#ifdef DO_NOT_LIGHT_SLEEP
  delay(u32Millis);
#else
    // Runtime control for light sleep (true = enabled, false = disabled)
  static bool light_sleep_enabled = true;
#ifdef BOARD_TRMNL_GEN2
  // Light sleep with the WiFi radio running hangs the ESP32-C5
  bool wifi_on = WiFi.getMode() != WIFI_MODE_NULL;
#else
  bool wifi_on = false;
#endif
  if (!light_sleep_enabled || wifi_on) {
    delay(u32Millis);
  } else {
    esp_sleep_enable_timer_wakeup(u32Millis * 1000L);
    esp_light_sleep_start();
  }
#endif
}

/**
 * @brief Function to reset the display
 * @param none
 * @return none
 */
void display_reset(void) {
  Log_info("e-Paper Clear start");
  bbep.fillScreen(BBEP_WHITE);
#ifdef BB_EPAPER
  display_set_light_sleep(true);
  if (!display_update_epaper(apiDisplayResult.response.maximum_compatibility ? REFRESH_FULL : REFRESH_FAST, true)) {
    Log_error("display_reset: e-paper update failed");
  }
#else
  bbep.fullUpdate();
#endif
  Log_info("e-Paper Clear end");
    // DEV_Delay_ms(500);
}

/**
 * @brief Function to read the display height
 * @return uint16_t - height of display in pixels
 */
uint16_t display_height() { return bbep.height(); }

/**
 * @brief Function to read the display width
 * @return uint16_t - width of display in pixels
 */
uint16_t display_width() { return bbep.width(); }

// Draws a PNG, JPEG, Group5 or BMP image into the panel driver and picks the refresh mode it needs;
// false when there was no memory to draw it
static bool draw_image(uint8_t *image_buffer, int data_size, int *pRefreshMode, bool *pAlloc) {
  bool isPNG = data_size >= 4 && MOTOLONG(image_buffer) == (int32_t)0x89504e47;
  auto width = display_width();
  auto height = display_height();
  bool bAlloc = false;
#ifdef BB_EPAPER
  int iRefreshMode = REFRESH_FULL; // assume full (slow) refresh
#else
  int iRefreshMode = 0;
#endif

#ifdef BB_EPAPER
  display_select_one_bit_panel();
#endif // BB_EPAPER
  if (isPNG == true && data_size < MAX_IMAGE_SIZE) {
    Log_info("Drawing PNG");
    iRefreshMode = png_to_epd(image_buffer, data_size, false);
  } else if (MOTOSHORT(image_buffer) == 0xffd8) {
    Log_info("Drawing JPEG");
    iRefreshMode = jpeg_to_epd(image_buffer, data_size);
  } else // uncompressed BMP or Group5 compressed image
  {
    if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER) {
            // G5 compressed image
      BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
#ifdef BB_EPAPER
      bbep.setMemoryMode(BBEP_MODE_1BPP); // use 1-bit mode to save memory
      if (bbep.allocBuffer(false) != BBEP_SUCCESS) {
        Log_info("Error allocating bb_epaper frame buffer");
        return false;
      }
      bAlloc = true;
#endif
        //    int x = (width - pBBB->width)/2;
        //    int y = (height - pBBB->height)/2; // center it
        // place it in the lower right corner
      int x = (width - pBBB->width);
      int y = (height - pBBB->height);
      if (x > 0 || y > 0) // only clear if the image is smaller than the display
      {
        bbep.fillScreen(BBEP_WHITE);
      }
      bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
#ifdef BOARD_TRMNL_X
      display_draw_battery_icon(bbep.width() - DISPLAY_BATTERY_ICON_W - DISPLAY_BATTERY_ICON_MARGIN,
                                DISPLAY_BATTERY_ICON_MARGIN);
#endif // BOARD_TRMNL_X
    } else {
         // This work-around is due to a lack of RAM; the correct method would be to use loadBMP()
      const int iBmpWidth = image_buffer[18] | (image_buffer[19] << 8);
      const int iBmpHeight = image_buffer[22] | (image_buffer[23] << 8);
      bmpNormalizePolarity(image_buffer, image_buffer + 62,
                           (iBmpWidth / 8) * iBmpHeight); // palette may be [white, black]
      flip_image(image_buffer + 62, iBmpWidth, iBmpHeight, false); // fix bottom-up bitmap images
#ifdef BB_EPAPER
      bbep.setMemoryMode(BBEP_MODE_1BPP);
      bbep.setBuffer(image_buffer + 62); // uncompressed 1-bpp bitmap
#endif // BB_EPAPER
    }
#ifdef BB_EPAPER
    bbep.writePlane(); // send image data to the EPD (converting it if needed to 4/6-clr)
    iRefreshMode = REFRESH_PARTIAL;
#endif // BB_EPAPER
    iUpdateCount = 1; // use partial update
  }
  *pRefreshMode = iRefreshMode;
  *pAlloc = bAlloc;
  return true;
}

// Saves the temperature profile the server asked for, when it changed
static void save_temp_profile_from_api(void) {
  // Cached/menu redraws can happen before this wake has fetched a display response.
  if (!apiDisplayResult.received) return;
  if (iTempProfile != apiDisplayResult.response.temp_profile) {
    iTempProfile = apiDisplayResult.response.temp_profile;
    Log_info("Saving new temperature profile (%" PRIu32 ") to FLASH", iTempProfile);
    preferences.putUInt(PREFERENCES_TEMP_PROFILE, iTempProfile);
  }
}

#ifdef BB_EPAPER
// Adjusts the image's refresh mode for the refresh count, the server's settings and the panel
static int choose_refresh_mode(int iRefreshMode, bool *pWait) {
  bool bWait = *pWait;
  const int fullEvery = apiDisplayResult.response.test_full_every >= 0 ? apiDisplayResult.response.test_full_every : 8;
  if ((fullEvery > 0 && iUpdateCount % fullEvery == 0) || apiDisplayResult.response.maximum_compatibility == true) {
    Log_info("%s [%d]: Forcing full refresh; desired refresh mode was: %d\r\n", __FILE__, __LINE__, iRefreshMode);
    iRefreshMode = REFRESH_FULL; // force full refresh every 8 partials
  }
  int refresh_seconds = refreshInterval.seconds();
  if (refresh_seconds >= 30 * 60 && iRefreshMode == REFRESH_PARTIAL) {
        // For users who set updates 30 minutes or longer, use the "fast" update to prevent ghosting
    Log_info("%s [%d]: Forcing fast refresh (not partial) since the TRMNL refresh_rate is set to > 30 min\n", __FILE__,
             __LINE__);
    iRefreshMode = REFRESH_FAST;
  }
  if (bbep.capabilities() & (BBEP_4GRAY | BBEP_4COLOR | BBEP_3COLOR | BBEP_7COLOR)) bWait = 1;
  if (!bWait) {
    iRefreshMode = REFRESH_FAST; // fast update when showing loading screen
  }
  const String &forced = apiDisplayResult.response.test_refresh; // spike: compare refresh types without waiting 30 min
  if (forced == "fast" && iRefreshMode == REFRESH_PARTIAL) iRefreshMode = REFRESH_FAST;
  if (forced == "partial" && iRefreshMode == REFRESH_FAST && bWait) iRefreshMode = REFRESH_PARTIAL;
  Log_info("%s [%d]: EPD refresh mode: %d\r\n", __FILE__, __LINE__, iRefreshMode);
  *pWait = bWait;
  return iRefreshMode;
}

// Refreshes the panel with the drawn image; false when the update failed
static bool refresh_epaper(int iRefreshMode, bool bWait, bool bAlloc) {
  static int i426Workaround = 0;
#ifdef DO_NOT_LIGHT_SLEEP
  display_set_light_sleep(false);
#else
  display_set_light_sleep(true);
#endif // DO_NOT_LIGHT_SLEEP
  if (bbep.getPanelType() == EP397_800x480 && iRefreshMode == REFRESH_FAST) {
        // Seeed Sticky: fast refresh on this panel isn't working and full refresh = fast
    iRefreshMode = REFRESH_FULL;
  }
  if (!display_update_epaper(iRefreshMode, bWait, bAlloc)) {
    Log_error("display_show_image: e-paper update failed");
    if (bAlloc) {
      bbep.freeBuffer();
    }
    return false;
  }

  if ((bbep.getPanelType() == EP426_800x480 || bbep.getPanelType() == EP426_800x480_4GRAY ||
       bbep.getPanelType() == EP397_800x480 ||
       bbep.getPanelType() == EP397_800x480_4GRAY) /* && iRefreshMode == REFRESH_PARTIAL*/) {
    i426Workaround = 1; // need to re-initialize the controller for another update before sleeping
  }
  if (bAlloc) {
    bbep.freeBuffer();
  }
  return true;
}

bool display_show_1bit_frame(uint8_t *frame, int iRefreshMode)
{
    if (!bCanDoPartial && iRefreshMode == REFRESH_PARTIAL) {
        return false; // the panel isn't showing a 1-bit image to change
    }
    display_select_one_bit_panel(true);
    bbep.setMemoryMode(BBEP_MODE_1BPP);
    bbep.setBuffer(frame);
    bbep.writePlane();
    bbep.setBuffer(NULL); // the frame is the caller's; don't let freeBuffer() take it
    if (!refresh_epaper(iRefreshMode, true, false)) {
        return false;
    }
    iUpdateCount++;
    return true;
}

// The OG's UC8179 partial window. bb_epaper's partial refresh always redraws the whole panel, and the panel drives
// every pixel against its "old" image memory, which bb_epaper leaves to the controller to keep up: when it doesn't match
// the screen (after a full refresh, say), unchanged pixels drift darker. So these write both the old and the new image.
// A window only limits the rows (gates) that are driven, not the columns, so it spans the screen's width.
static bool can_refresh_window(void)
{
    int panel = bbep.getPanelType();
    return bbep.getChip() == BBEP_CHIP_UC81xx && (panel == EP75_800x480 || panel == EP75_800x480_GEN2);
}

static void select_rows(int y, int h)
{
    int x1 = bbep.width();
    uint8_t ptl[9] = {
        0, 0, (uint8_t)((x1 - 1) >> 8), (uint8_t)((x1 - 1) | 7),
        (uint8_t)(y >> 8), (uint8_t)y, (uint8_t)((y + h - 1) >> 8), (uint8_t)(y + h - 1),
        0x00, // PT_SCAN: only inside the window
    };
    bbep.writeCmd(UC8151_PTIN);
    bbep.writeCmd(UC8151_PTL);
    bbep.writeData(ptl, sizeof(ptl));
}

static void write_rows(uint8_t command, const uint8_t *frame, int y, int h)
{
    int pitch = (bbep.width() + 7) / 8;
    bbep.writeCmd(command);
    for (int row = y; row < y + h; row++) {
        bbep.writeData((uint8_t *)frame + row * pitch, pitch);
    }
}

// Sends the rows' current pixels as the old image, with the dithered hold's subset outside the region flipped
static void write_old_rows(const uint8_t *frame, int y, int h, const DitherHold &hold)
{
    int pitch = (bbep.width() + 7) / 8;
    uint8_t *row = bbep.getCache(); // scratch, as png_draw() uses it
    int first = hold.x / 8, last = (hold.x + hold.w + 7) / 8; // the region's bytes
    bbep.writeCmd(UC8151_DTM1);
    for (int r = y; r < y + h; r++) {
        const uint8_t *pixels = frame + r * pitch;
        uint8_t mask = liveRegionDitherMask(r, hold.every, hold.phase);
        if (!mask) {
            bbep.writeData((uint8_t *)pixels, pitch);
            continue;
        }
        bool inRegion = r >= hold.y && r < hold.y + hold.h;
        for (int i = 0; i < pitch; i++) {
            row[i] = inRegion && i >= first && i < last ? pixels[i] : pixels[i] ^ mask;
        }
        bbep.writeData(row, pitch);
    }
}

bool display_window_ready(void)
{
    display_select_one_bit_panel(true); // after a gray image the panel is set up for 2 bits
    if (!can_refresh_window()) {
        return false;
    }
    bCanDoPartial = true; // the window refresh sends both the old and the new image itself
    return true;
}

bool display_window_begin_unknown(const uint8_t *frame, int y, int h)
{
    if (!can_refresh_window() || !bCanDoPartial) {
        return false;
    }
    int pitch = (bbep.width() + 7) / 8;
    uint8_t *row = bbep.getCache();
    select_rows(y, h);
    bbep.writeCmd(UC8151_DTM1);
    for (int r = y; r < y + h; r++) {
        for (int i = 0; i < pitch; i++) {
            row[i] = ~frame[r * pitch + i]; // every pixel "changes", so each is driven to its new color
        }
        bbep.writeData(row, pitch);
    }
    return true;
}

bool display_window_begin(const uint8_t *frame, int y, int h, const DitherHold &hold)
{
    if (!can_refresh_window() || !bCanDoPartial) {
        return false;
    }
    select_rows(y, h);
    write_old_rows(frame, y, h, hold); // what's on screen now
    return true;
}

// A partial waveform that drives pixels that change for `drive` frames twice (with a 5-frame rest after each) and can
// hold pixels that don't for `keep` frames toward their color. bb_epaper's OG partial table drives unchanged pixels as
// long as changing ones, every refresh, which darkens the screen; with VCOM near 0 V a short hold keeps them white
// (it shows as a slight white flash of the panel; without it they darken). Each table: groups of level select (2 bits
// a phase: 00 GND, 01 VDH = black, 10 VDL = white), four phase lengths in frames and a repeat count; all tables last
// as many frames.
static void send_partial_lut(const PartialWaveform &waveform)
{
    const struct {
        uint8_t command;
        uint8_t length;
        uint8_t changeLevels; // during the drive
        uint8_t keepLevels; // during the keep
        uint8_t balancedChangeLevels; // the opposite color first, then the target
        uint8_t balancedKeepLevels;
    } tables[] = {
        {UC8151_LUT_VCOM, 44, 0x00, 0x00, 0x00, 0x00}, // VCOM stays at its DC level
        {UC8151_LUT_WW, 42, 0x00, 0x80, 0x00, 0x60},   // unchanged white
        {UC8151_LUT_BW, 42, 0x88, 0x00, 0x48, 0x00},   // black to white
        {UC8151_LUT_WB, 42, 0x44, 0x00, 0x84, 0x00},   // white to black
        {UC8151_LUT_BB, 42, 0x00, 0x40, 0x00, 0x90},   // unchanged black
        {UC8151_LUT_VCOM2, 42, 0x00, 0x80, 0x00, 0x60}, // the border (with CDI 0x39; 00 is VCOM here, 10 VDL)
    };
    uint8_t lut[44];
    for (const auto &table : tables) {
        memset(lut, 0, sizeof(lut));
        uint8_t *group = lut;
        const uint8_t driveGroup[6] = {waveform.balanced ? table.balancedChangeLevels : table.changeLevels,
                                       waveform.drive, 5, waveform.drive, 5, 0x01};
        memcpy(group, driveGroup, sizeof(driveGroup));
        group += 6;
        if (waveform.keep) {
const uint8_t keepGroup[6] = {waveform.balanced ? table.balancedKeepLevels : table.keepLevels, waveform.keep,
                                         (uint8_t)(waveform.balanced ? waveform.keep : 0), 0, 0, 0x01};
            memcpy(group, keepGroup, sizeof(keepGroup));
        }
        bbep.writeCmd(table.command);
        bbep.writeData(lut, table.length);
    }
}

// The panel's own partial waveform, as Waveshare, GoodDisplay and GxEPD2 use it on the UC8179: the waveform and VCOM
// from the glass's OTP, its differential bank picked by a fixed temperature of 110, the border floating
static void send_otp_partial_settings(void)
{
    const uint8_t psr = 0x1f, tsfix = 0x02, temperature = 0x6e, cdi[2] = {0xa9, 0x07};
    bbep.writeCmd(UC8151_PSR); // waveform from the panel's OTP
    bbep.writeData((uint8_t *)&psr, 1);
    bbep.writeCmd(0xe0); // cascade setting: use the temperature below instead of the sensor
    bbep.writeData((uint8_t *)&tsfix, 1);
    bbep.writeCmd(0xe5); // force temperature
    bbep.writeData((uint8_t *)&temperature, 1);
    bbep.writeCmd(UC8151_CDI);
    bbep.writeData((uint8_t *)cdi, sizeof(cdi));
}

// The device's own partial waveform from the registers
static void send_register_partial_settings(const PartialWaveform &waveform)
{
    // The partial mode's settings (as in bb_epaper's OG partial sequence); a full refresh sets its own
    // CDI 0x29: the border follows the black-to-white table; 0x39: its own
    const uint8_t pwr[5] = {0x07, 0x07, 0x3f, 0x3f, 0x03}, psr = 0x3f;
    const uint8_t cdi[2] = {(uint8_t)(waveform.border ? 0x39 : 0x29), 0x07}, tcon = waveform.tcon, pll = waveform.pll;
    bbep.writeCmd(0x30); // frame rate
    bbep.writeData((uint8_t *)&pll, 1);
    bbep.writeCmd(UC8151_PWR);
    bbep.writeData((uint8_t *)pwr, sizeof(pwr));
    bbep.writeCmd(UC8151_PSR); // waveform from the registers below, not the panel's OTP
    bbep.writeData((uint8_t *)&psr, 1);
    bbep.writeCmd(UC8151_CDI);
    bbep.writeData((uint8_t *)cdi, sizeof(cdi));
    bbep.writeCmd(UC8151_TCON);
    bbep.writeData((uint8_t *)&tcon, 1);
    send_partial_lut(waveform);
    // VCOM's DC level (VDCS 0x00 = -0.1 V, each step -0.05 V more): it should cancel the pixels' TFT kickback, which
    // other drivers for this panel do at about -2 V
    uint8_t vdcs = waveform.vcom;
    bbep.writeCmd(0x82);
    bbep.writeData(&vdcs, 1);
}

bool display_window_refresh(const uint8_t *frame, int y, int h, const PartialWaveform &waveform)
{
    select_rows(y, h);
    write_rows(UC8151_DTM2, frame, y, h); // what it becomes
    if (waveform.otp) {
        send_otp_partial_settings();
    } else {
        send_register_partial_settings(waveform);
    }
    bbep.writeCmd(UC8151_PON);
    bbep.wait();
    bbep.writeCmd(UC8151_DRF);
    bbep.wait();
    bbep.writeCmd(UC8151_PTOU);
    const uint8_t normalPll = 0x06, normalTcon = 0x22; // what full refreshes run at
    bbep.writeCmd(0x30);
    bbep.writeData((uint8_t *)&normalPll, 1);
    bbep.writeCmd(UC8151_TCON);
    bbep.writeData((uint8_t *)&normalTcon, 1);
    if (waveform.otp) {
        const uint8_t sensor = 0x00;
        bbep.writeCmd(0xe0); // back to the temperature sensor, so full refreshes pick their usual waveform
        bbep.writeData((uint8_t *)&sensor, 1);
    }
    iLastRefreshMode = REFRESH_PARTIAL;
    iUpdateCount++;
    return true;
}

void display_power_off(void)
{
    if (!can_refresh_window()) {
        return;
    }
    bbep.writeCmd(UC8151_POFF); // keeps the registers, unlike display_sleep(), which also floats the border
    bbep.wait();
}

void display_frame_begin(const uint8_t *frame)
{
    if (!can_refresh_window()) {
        return;
    }
    bbep.writeCmd(UC8151_PTOU);
    write_rows(UC8151_DTM1, frame, 0, bbep.height()); // what's on screen now
}
#else
// Pushes the drawn image to the panel with the right clearing passes
static void refresh_fastepd(bool bSkipClear) {
  {
    int rc = bbep.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    Log_info("%s [%d]: setCustomMatrix returned %d\r\n", __FILE__, __LINE__, rc);

 //   if (bbep.getPreviousMode() != BB_MODE_NONE && (bbep.getMode() == BB_MODE_1BPP || bbep.getMode() ==
 //   BB_MODE_2BPP)) {
    //       Log_info("%s [%d]: Using partial update since we have a copy of the previous image\n", __FILE__, __LINE__);
    //       bbep.setPasses(6,6);
    //       bbep.partialUpdate(false); // we have a previous image to diff against; use a non-flickering update
    //   } else {
    // bWait=false means loading screen: skip clearing passes so it appears
    // faster. Ghosting from the previous image is acceptable since the
    // real content refresh (bWait=true) immediately follows.
    int iClearMode = bSkipClear                                        ? CLEAR_NONE
                     : ((iUpdateCount & 7) == 0 || (iTempProfile > 0)) ? CLEAR_SLOW
                                                                       : CLEAR_FAST;
    Log_info("fullUpdate clear mode = %d\n", iClearMode);
    bbep.fullUpdate(iClearMode, false);
    //   }
  }
}
#endif // BB_EPAPER

/**
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */
#ifdef LIVE_MODE
uint32_t live_draw_ms = 0;
int live_refresh_mode = -1;
#endif
void display_show_image(uint8_t *image_buffer, int data_size, bool bWait, bool bSkipClear) {
#ifdef LIVE_MODE
  const uint32_t draw_started = millis();
  struct DrawTimer { uint32_t s; ~DrawTimer() { live_draw_ms = millis() - s; } } draw_timer{draw_started};
#endif
  int iRefreshMode;
  bool bAlloc;

  Log_info("display_show_image start");
  Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
  if (!draw_image(image_buffer, data_size, &iRefreshMode, &bAlloc)) {
    return;
  }
  Log_info("Display refresh start");
  save_temp_profile_from_api();
#ifdef BB_EPAPER
  iRefreshMode = choose_refresh_mode(iRefreshMode, &bWait);
#ifdef LIVE_MODE
  live_refresh_mode = iRefreshMode;
#endif
  if (!refresh_epaper(iRefreshMode, bWait, bAlloc)) {
    return;
  }
#else
  refresh_fastepd(bSkipClear);
#endif
  iUpdateCount++;
  Log_info("display_show_image end");
}
/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void) {
  Log_info("Goto Sleep...");
#ifdef BB_EPAPER
  bbep.sleep(LIGHT_SLEEP);
#else
  bbep.einkPower(0);
  bbep.deInit();
#endif
}
