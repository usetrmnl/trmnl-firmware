#include <Arduino.h>
#include <display.h>
#include <power.h>
#include <PNGdec.h>
#include <JPEGDEC.h>
#include <Preferences.h>
#include <WiFi.h>
#include <preferences_persistence.h>
#include <refresh_interval.h>
#include "messages.h"
#include "config.h"
#include "battery.h"
#include <board_config.h>
#include <color_reduce.h>
#include <display_internal.h>
#define MAX_BIT_DEPTH 8
#ifdef PARALLEL_EPD
FASTEPD bbep;
#else
BBEPAPER bbep;
#include <SPIFFS.h>
#define FS SPIFFS
#endif // !PARALLEL_EPD

#ifdef PARALLEL_EPD
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "LittleFS.h"
#define FS LittleFS

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
#include "BQ27427.h"
extern BQ27427 lipo; // Use lipo.[] to interact with the library in an Arduino 
#endif

#include "Group5.h"
#include <config.h>
#include <ctype.h> //iscntrl()
#include <api-client/display.h>
#include <inttypes.h>
#include <trmnl_log.h>
#include "png_flip.h"
#include "bmp.h"
#include <globals.h>

#ifdef BB_EPAPER
static uint8_t bLightSleepWanted = 0;

static void display_apply_light_sleep(uint8_t enabled)
{
#ifdef CMD_CS1_CS2
    bbep.setLightSleep(enabled, pDevice->epd_en_pin);
#else
    bbep.setLightSleep(enabled);
#endif
}

bool display_update_epaper(int refreshMode, bool wait, bool writePlane, uint8_t plane)
{
#if defined(BOARD_SEEED_RETERMINAL_E1002) || defined(BOARD_SEEED_RETERMINAL_E1004)
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
    } else {
        Log_info("Have valid previous image in EPD memory, doing partial refresh");
    }
    bbep.refresh(refreshMode, wait);
    // The next update can be a partial update because the current is 1-bpp and stays in the EPD RAM
    bCanDoPartial = (bbep.getPanelType() == dpList[pDevice->panel_set][iTempProfile].OneBit);
    return true;
}

void display_select_one_bit_panel(bool only_if_changed)
{
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

// Message screens are 1-bit
void display_prepare_message(void)
{
    display_select_one_bit_panel(true);
    bbep.setMemoryMode(BB_MODE_1BPP); // use 1-bit mode to save memory
    bbep.allocBuffer(false);
}

#endif // BB_EPAPER

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void)
{
    if (pDevice == NULL) {
        hw_config_init();
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
    if (pDevice->epd_mosi_pin != 0 || pDevice->epd_sck_pin != 0) {
        bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit); // must be set BEFORE calling initio
        bbep.initIO(pDevice->epd_dc_pin, pDevice->epd_rst_pin, pDevice->epd_busy_pin, pDevice->epd_cs_pin,
        pDevice->epd_mosi_pin, pDevice->epd_sck_pin, 8000000);
    } else { // it's a pre-defined PCB+display in bb_epaper
        bbep.begin(dpList[pDevice->panel_set][0].OneBit);
    }
#else // Parallel eink devices
#if defined (BOARD_SEEED_RETERMINAL_E1003)
    bbep.initIT8951(EPD_MOSI_PIN, EPD_MISO_PIN, EPD_SCK_PIN, EPD_CS_PIN, EPD_BUSY_PIN, EPD_RST_PIN, EPD_EN_PIN, EPD_VCC_EN);
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
//#elif defined( BOARD_TRMNL_X_SENSORIAS3 )
//    bbep.initPanel(BB_PANEL_V7_RAW);
//    bbep.setPanelSize(1280, 720, BB_PANEL_FLAG_MIRROR_X, -1600);
//#elif defined( BOARD_TRMNL_X_SENSORIAC5 )
//    bbep.initPanel(BB_PANEL_SENSORIA_C5);
//#elif defined(BOARD_TRMNL_X_PAPERS3)
//    bbep.initPanel(BB_PANEL_M5PAPERS3);
//#elif defined(BOARD_TRMNL_X_LILYGO)
//    bbep.initPanel(BB_PANEL_EPDIY_V7);
//    bbep.setPanelSize(960, 540);
#endif // bb_epaper
    Log_info("dev module end");
}


/**
 * @brief Enable or disable light sleep at runtime
 * @param enabled true to enable light sleep, false to disable
 * @return none
 */
void display_set_light_sleep(uint8_t enabled)
{
#ifdef BB_EPAPER
    bLightSleepWanted = enabled;
    display_apply_light_sleep(enabled);
#endif
}

// Clear any ghosting on the X display by wiping it black/white many times
void display_wipe(void)
{
#ifdef BB_EPAPER

#ifdef BOARD_TRMNL_4CLR
    int refreshCount = 2;
#else
    int refreshCount = 60;
#endif

    bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit);
    bbep.fillScreen(BBEP_WHITE);
    for (int i=0; i<refreshCount; i++) {
        bbep.refresh(REFRESH_FULL); // 2 to 3 minutes of Black/White clearing of the display
    }
    bbep.sleep(LIGHT_SLEEP);
#else
    bbep.setMode(BB_MODE_1BPP);
    bbep.fillScreen(BBEP_WHITE);
    for (int i=0; i<100; i++) { // 200 black/white cycles should remove any ghosting
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
void display_sleep(uint32_t u32Millis)
{
#ifdef DO_NOT_LIGHT_SLEEP
    delay(u32Millis);
#else
    // Runtime control for light sleep (true = enabled, false = disabled)
    static bool light_sleep_enabled = true;
    if (!light_sleep_enabled) {
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
void display_reset(void)
{
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
uint16_t display_height()
{
    return bbep.height();
}

/**
 * @brief Function to read the display width
 * @return uint16_t - width of display in pixels
 */
uint16_t display_width()
{
    return bbep.width();
}

/**
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */
void display_show_image(uint8_t *image_buffer, int data_size, bool bWait, bool bSkipClear)

{
    bool isPNG = data_size >= 4 && MOTOLONG(image_buffer) == (int32_t)0x89504e47;
    auto width = display_width();
    auto height = display_height();
//    uint32_t *d32;
    bool bAlloc = false;
    static int i426Workaround = 0;
#ifdef BB_EPAPER
    int iRefreshMode = REFRESH_FULL; // assume full (slow) refresh
#else
    int iRefreshMode = 0;
#endif

   // Log_info("Paint_NewImage %d", reverse);
    Log_info("display_show_image start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef FUTURE
    if (reverse)
    {
        d32 = (uint32_t *)image_buffer; // get framebuffer as a 32-bit pointer
        d32 = (uint32_t *)image_buffer; // get framebuffer as a 32-bit pointer
        Log_info("inverse the image");
        for (size_t i = 0; i < buf_size; i+=sizeof(uint32_t))
        for (size_t i = 0; i < buf_size; i+=sizeof(uint32_t))
        {
            d32[0] = ~d32[0];
            d32++;
            d32[0] = ~d32[0];
            d32++;
        }
    }
#endif
#ifdef BB_EPAPER
    display_select_one_bit_panel();
#endif // BB_EPAPER
    if (isPNG == true && data_size < MAX_IMAGE_SIZE)
    {
        Log_info("Drawing PNG");
        iRefreshMode = png_to_epd(image_buffer, data_size, false);
    }
    else if (MOTOSHORT(image_buffer) == 0xffd8) {
        Log_info("Drawing JPEG");
        iRefreshMode = jpeg_to_epd(image_buffer, data_size);
    }
    else // uncompressed BMP or Group5 compressed image
    {
        if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER)
        {
            // G5 compressed image
            BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
#ifdef BB_EPAPER
            bbep.setMemoryMode(BB_MODE_1BPP); // use 1-bit mode to save memory
            if (bbep.allocBuffer(false) != BBEP_SUCCESS) {
                Log_info("Error allocating bb_epaper frame buffer");
                return;
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
        }
        else
        {
         // This work-around is due to a lack of RAM; the correct method would be to use loadBMP()
            const int iBmpWidth = image_buffer[18] | (image_buffer[19] << 8);
            const int iBmpHeight = image_buffer[22] | (image_buffer[23] << 8);
            bmpNormalizePolarity(image_buffer, image_buffer+62, (iBmpWidth / 8) * iBmpHeight); // palette may be [white, black]
            flip_image(image_buffer+62, iBmpWidth, iBmpHeight, false); // fix bottom-up bitmap images
#ifdef BB_EPAPER
            bbep.setMemoryMode(BB_MODE_1BPP);
            bbep.setBuffer(image_buffer+62); // uncompressed 1-bpp bitmap
#endif // BB_EPAPER
        }
#ifdef BB_EPAPER
        bbep.writePlane(); // send image data to the EPD (converting it if needed to 4/6-clr)
        iRefreshMode = REFRESH_PARTIAL;
#endif // BB_EPAPER
        iUpdateCount = 1; // use partial update
    }
    Log_info("Display refresh start");
    if (iTempProfile != apiDisplayResult.response.temp_profile) {
        iTempProfile = apiDisplayResult.response.temp_profile;
        Log_info("Saving new temperature profile (%" PRIu32 ") to FLASH", iTempProfile);
        preferences.putUInt(PREFERENCES_TEMP_PROFILE, iTempProfile);
    }
#ifdef BB_EPAPER
    if ((iUpdateCount & 7) == 0 || apiDisplayResult.response.maximum_compatibility == true) {
        Log_info("%s [%d]: Forcing full refresh; desired refresh mode was: %d\r\n", __FILE__, __LINE__, iRefreshMode);
        iRefreshMode = REFRESH_FULL; // force full refresh every 8 partials
    }
    int refresh_seconds = refreshInterval.seconds();
    if (refresh_seconds >= 30*60 && iRefreshMode == REFRESH_PARTIAL) {
        // For users who set updates 30 minutes or longer, use the "fast" update to prevent ghosting
        Log_info("%s [%d]: Forcing fast refresh (not partial) since the TRMNL refresh_rate is set to > 30 min\n", __FILE__, __LINE__);
        iRefreshMode = REFRESH_FAST;
    }
    if (bbep.capabilities() & (BBEP_4GRAY | BBEP_4COLOR | BBEP_3COLOR | BBEP_7COLOR)) bWait = 1;
    if (!bWait) {
        iRefreshMode = REFRESH_FAST; // fast update when showing loading screen
    }
    Log_info("%s [%d]: EPD refresh mode: %d\r\n", __FILE__, __LINE__, iRefreshMode);
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
        return;
    }

    if ((bbep.getPanelType() == EP426_800x480 || bbep.getPanelType() == EP426_800x480_4GRAY || bbep.getPanelType() == EP397_800x480 || bbep.getPanelType() == EP397_800x480_4GRAY) /* && iRefreshMode == REFRESH_PARTIAL*/) {
        i426Workaround = 1; // need to re-initialize the controller for another update before sleeping
    }
    if (bAlloc) {
        bbep.freeBuffer();
    }
#else
 {
    int rc = bbep.setCustomMatrix(u8_graytable, sizeof(u8_graytable));
    Log_info("%s [%d]: setCustomMatrix returned %d\r\n", __FILE__, __LINE__, rc);

 //   if (bbep.getPreviousMode() != BB_MODE_NONE && (bbep.getMode() == BB_MODE_1BPP || bbep.getMode() == BB_MODE_2BPP)) {
 //       Log_info("%s [%d]: Using partial update since we have a copy of the previous image\n", __FILE__, __LINE__);
 //       bbep.setPasses(6,6);
 //       bbep.partialUpdate(false); // we have a previous image to diff against; use a non-flickering update
 //   } else {
        // bWait=false means loading screen: skip clearing passes so it appears
        // faster. Ghosting from the previous image is acceptable since the
        // real content refresh (bWait=true) immediately follows.
        int iClearMode = bSkipClear ? CLEAR_NONE
                                   : ((iUpdateCount & 7) == 0 || (iTempProfile > 0)) ? CLEAR_SLOW : CLEAR_FAST;
        Log_info("fullUpdate clear mode = %d\n", iClearMode);
        bbep.fullUpdate(iClearMode, false);
 //   }
 }
#endif
    iUpdateCount++;
    Log_info("display_show_image end");
}
/**
 * @brief Function to read an image from the file system
 * @param filename
 * @param pointer to file size returned
 * @return pointer to allocated buffer
 */
uint8_t * display_read_file(const char *filename, int *file_size)
{
File f = FS.open(filename, "r");
uint8_t *buffer;

  if (!f) {
    Serial.println("Failed to open file!");
    *file_size = 0;
    return nullptr;
  }
  *file_size = f.size();
  if (*file_size == 0) {
    Serial.println("File is empty!");
    f.close();
    return nullptr;
  }
  Serial.printf("File size to allocate: %d bytes\n", *file_size);
  #ifdef CONFIG_SPIRAM
  Serial.println("Allocating file buffer in PSRAM");
  buffer = (uint8_t *)ps_malloc(*file_size);
  #else
  Serial.println("Allocating file buffer in regular RAM");
  buffer = (uint8_t *)malloc(*file_size);
  #endif
  if (!buffer) {
    Serial.println("Memory allocation failed!");
    *file_size = 0;
    return nullptr;
  }
  f.read(buffer, *file_size);
  f.close();
  return buffer;
} /* display_read_file() */

/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void)
{
    Log_info("Goto Sleep...");
#ifdef BB_EPAPER
    bbep.sleep(LIGHT_SLEEP);
#else
    bbep.einkPower(0);
    bbep.deInit();
#endif
}
