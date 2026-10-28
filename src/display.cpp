#include <Arduino.h>
#include <display.h>
#include <power.h>
#include <PNGdec.h>
#include <JPEGDEC.h>
#include <Preferences.h>
#include <WiFi.h>
#include <preferences_persistence.h>
#include <refresh_interval.h>
#include "battery_small.h"
#include "battery_hollow.h"
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
#include "wifi_connect_qr.h"
#include "wifi_failed_qr.h"
#include <ctype.h> //iscntrl()
#include <api-client/display.h>
#include <inttypes.h>
#include <trmnl_log.h>
#include "png_flip.h"
#include "bmp.h"
#include "fonts.h"
#include <globals.h>
static uint8_t *pDither;

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

void display_fill_pill(int x, int y, int w, int h, uint8_t color)
{
    float r = h / 2.0f;
    for (int row = 0; row < h; row++) {
        float dy = row + 0.5f - r;
        int inset = (int)lroundf(r - sqrtf(r * r - dy * dy));
        if (w - 2 * inset > 0) {
            bbep.fillRect(x + inset, y + row, w - 2 * inset, 1, color);
        }
    }
}

void display_draw_battery_icon(int x, int y)
{
#ifdef BOARD_TRMNL_X
    // The charging icon while on USB power (whether actually charging or not)
    if (power().usbStatus() == UsbStatus::CONNECTED) {
        Log_info("Displaying 'battery is charging' icon");
        bbep.loadG5Image(battery_small, x, y, BBEP_WHITE, BBEP_BLACK);
        return;
    }
    bbep.loadG5Image(battery_hollow, x, y, BBEP_WHITE, BBEP_BLACK);
    Log_info("Displaying 'battery charge level' icon");
    int batt_percent = battery().readSoc(); // -1 if no battery or the gauge reading failed
    if (batt_percent >= 0) {
        if (batt_percent >= 97) batt_percent = 100; // can sometimes report 98% when full
        // Draw a black rectangle to represent the battery charge level
        bbep.fillRect(x + 10, y + 18, (97 * batt_percent) / 100, 39, BBEP_BLACK);
    } else { // a cross over the empty battery: each diagonal a band 6 px tall, column by column
        const int left = x + 10, top = y + 18, w = 97, h = 39, half = 3;
        for (int i = 0; i < w; i++) {
            int down = top + i * h / w;     // top left to bottom right
            int up = top + h - i * h / w;   // bottom left to top right
            for (int c : {down, up}) {
                int y0 = max(c - half, top), y1 = min(c + half, top + h);
                bbep.fillRect(left + i, y0, 1, y1 - y0, BBEP_BLACK);
            }
        }
    }
#endif // BOARD_TRMNL_X
}

/**
 * @brief Function to draw multi-line text onto the display
 * @param x_start X coordinate to start drawing
 * @param y_start Y coordinate to start drawing
 * @param message Text message to draw
 * @param max_width Maximum width in pixels for each line
 * @param color_fg Foreground color
 * @param color_bg Background color
 * @param font Font to use
 * @return none
 */
void Paint_DrawMultilineText(UWORD x_start, UWORD y_start, const char *message,
                             uint16_t max_width, UWORD color_fg, UWORD color_bg, const void *font)
{
char szTemp[80]; // up to 79 characters per line: getStringBox() copies the string to an 80-byte buffer
char c, *d, *s = (char *)message;
BB_RECT rect;
bool bDone = false;
int iWidthLimit = (max_width * 3)/4; // don't let the text go all the way to the edges

    bbep.setFont(font);
    bbep.setTextColor(color_fg, color_bg);
    d = szTemp;
    bbep.setCursor(x_start, y_start); // start on the requested col/row
    while (!bDone) {
        c = *s++;
        *d++ = c;
        d[0] = 0; // test the string length
        bbep.getStringBox(szTemp, &rect);
        if (c == 0 || c == '\n' || (c == ' ' && rect.w >= iWidthLimit) || d == &szTemp[sizeof(szTemp) - 1]) {
            if (c == 0 || s[0] == 0) bDone = true;
            if (c == ' ' || c == '\n') {
                d[-1] = 0; // don't print the space/newline
                bbep.getStringBox(szTemp, &rect); // and don't center it either
            }
            // Display the current partial string
            bbep.setCursor(x_start + (max_width - rect.w) / 2, -1);
            bbep.println(szTemp);
            d = szTemp;
        }
    }
} /* Paint_DrawMultilineText() */

/**
 * @brief JPEGDEC callback function passed blocks of MCUs (minimum coded units)
 * @param pointer to the JPEGDRAW structure
 * @return 1 to continue decoding or 0 to abort
 */
int jpeg_draw(JPEGDRAW *pDraw)
{
#ifdef BB_EPAPER
int x, y;
int iPlane = *(int *)pDraw->pUser;
uint8_t src=0, uc=0, ucMask, *s, *d, *pTemp = bbep.getCache();

    bbep.setAddrWindow(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
    if (iPlane == 0) { // 1-bit mode
        bbep.startWrite(PLANE_0); // start writing image data to plane 0
        for (y=0; y<pDraw->iHeight; y++) { // this is 8 or 16 depending on the color subsampling
            s = (uint8_t *)pDraw->pPixels;
            s += (y * (pDraw->iWidth >> 3));
            // The pixel format of the display is the same as JPEGDEC, so just copy it
            bbep.writeData(s, (pDraw->iWidth+7)/8);
        } // for y
    } else {
        bbep.startWrite((iPlane == 1) ? PLANE_0 : PLANE_1); // start writing image data to plane 0
        for (y=0; y<pDraw->iHeight; y++) { // this is 8 or 16 depending on the color subsampling
            d = pTemp;
            s = (uint8_t *)pDither;
            s += (y * (pDraw->iWidth >> 2));
            ucMask = (iPlane == 1) ? 0x40 : 0x80; // lower or upper source bit
            for (x=0; x<pDraw->iWidth; x++) {
                if ((x & 3) == 0) { // new input byte
                    src = *s++;
                }
                uc <<= 1;
                if (src & ucMask) {
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
            bbep.writeData(pTemp, (pDraw->iWidth+7)/8);
        } // for y
    }
#else // FastEPD
  int x, y, iPitch = bbep.width()/2; // assume 4-bpp drawing mode
  uint8_t *s, *d, *pBuffer = bbep.currentBuffer();
  for (y=0; y<pDraw->iHeight; y++) {
    d = &pBuffer[((pDraw->y + y)*iPitch) + (pDraw->x/2)];
    s = (uint8_t *)pDraw->pPixels;
    s += (y * (pDraw->iWidth/2));
    memcpy(d, s, pDraw->iWidth/2); // source & dest format are the same
  } // for y
#endif
    return 1; // continue decoding
} /* jpeg_draw() */
/**
 * @brief Function to decode and display a JPEG image from memory
 *        The decoded lines are written directly into the EPD framebuffer
 *        due to insufficient RAM to hold the fully decoded image
 * @param pointer to the buffer holding the JPEG file
 * @param size of the JPEG file
 * @return refresh mode based on image type and presence of old image
 */
int jpeg_to_epd(const uint8_t *pJPEG, int iDataSize)
{
JPEGDEC *jpg = new JPEGDEC();
int rc = -1; // invalid mode
int iPlane = 0;

    if (!jpg) {
        Log_error("%s [%d]: Not enough memory for the JPEG decoder instance", __FILE__, __LINE__);
        return JPEG_ERROR_MEMORY; // not enough memory for the decoder instance
    }
    rc = jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
    if (rc) {
        if (jpg->getWidth() != bbep.width() || jpg->getHeight() != bbep.height()) {
            Log_error("JPEG image size doesn't match display size");
            rc = -1;
        } else { // okay to decode
#ifdef BB_EPAPER
            //bbep.setPanelType(TWO_BIT_PANEL);
            Log_info("%s [%d]: Decoding jpeg as 1-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(ONE_BIT_DITHERED); // request 1-bit dithered output
#else
            bbep.setMode(BB_MODE_4BPP);
            Log_info("%s [%d]: Decoding jpeg as 4-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(FOUR_BIT_DITHERED); // request 4-bit dithered output
#endif
            pDither = (uint8_t *)malloc(jpg->getWidth() * 16);
            iPlane = 0;//1; // Decode first plane
            Log_info("%s [%d]: Decoding plane 0\r\n", __FILE__, __LINE__);
            jpg->setUserPointer((void *)&iPlane);
            jpg->decodeDither(pDither, 0);
            jpg->close();
            // Decode the second plane
//            iPlane = 2;
//            Log_info("%s [%d]: Decoding plane 1\r\n", __FILE__, __LINE__);
//            jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
//            jpg->setPixelType(TWO_BIT_DITHERED); // request 1-bit dithered output
//            jpg->setUserPointer((void *)&iPlane);
//            jpg->decodeDither(pDither, 0);
            free(pDither);
#ifdef BB_EPAPER
            rc = REFRESH_FULL;
#endif
        }
    }
    jpg->close();
    delete(jpg);
    return rc;
} /* jpeg_to_epd() */
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
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type, const char *message_text)
{
    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef BB_EPAPER
    display_prepare_message();
#else
    bbep.setMode(BB_MODE_1BPP); // message screens are 1-bit
#endif
    if (image_buffer && *(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        if (x > 0 || y > 0) // only clear if the image is smaller than the display
        {
            bbep.fillScreen(BBEP_WHITE);
        }
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
#ifdef BB_EPAPER
        if (image_buffer) memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
#endif
    }

#ifdef PARALLEL_EPD
    bbep.setFont(Inter_18);
#else
    bbep.setFont(nicoclean_8);
#endif
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);

    switch (message_type)
    {
    case OTG_TURNED_ON:
    {
        const char string1[] = "OTG turned on!";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);
    break;
    }
    case OTG_TURNED_OFF:
    {
        const char string1[] = "OTG turned off!";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);
    break;
    }
    case MODEM_FLASHING:
    {
        const char string1[] = "Flashing modem firmware...";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);
    break;
    }
    case MODEM_FLASH_FAILED:
    {
        const char string1[] = "Failed to flash modem firmware,";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);

        const char string2[] = "device would only operate with 2.4Ghz WiFi.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 500);
        if (message_text) {
            bbep.println(string2);
            bbep.getStringBox(message_text, &rect);
            bbep.setCursor((bbep.width() - rect.w)/2, 570);
            bbep.print(message_text);
        } else {
            bbep.print(string2);
        }
    break;
    }
    case READY_TO_SHIP:
    {
        const char string1[] = "Device is ready to ship!";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);

        const char string2[] = "Unplug the USB-C to complete shutdown.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 500);
        bbep.print(string2);
    break;
    }
    case SHIPPING_MODE:
    {
        const char string1[] = "Welcome to TRMNL.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);

        const char string2[] = "Attach the dock and a USB-C to get started.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 500);
        bbep.print(string2);
    break;
    }
    case WIFI_CONNECT:
    {
        const char string1[] = "Connect to TRMNL WiFi";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 430);
        bbep.println(string1);
        const char string2[] = "on your phone or computer";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, -1);
        bbep.print(string2);
    }
    break;
    case WIFI_FAILED:
    {
        String string0 = "TRMNL firmware ";
        string0 += Messages::firmware_version();
#ifdef __BB_EPAPER__
        bbep.setCursor(40, 48); // place in upper left corner
#else
        bbep.setCursor(80, 104); // place in upper left corner
#endif
        bbep.println(string0);
        const char string1[] = "Can't establish WiFi connection. Will keep trying.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, bbep.height() - (rect.h*2)-140);
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Hold button on the back to reset WiFi, or scan QR Code for help.";
#else
        const char string2[] = "Hold left and right corner of touch bar to reset WiFi, or scan QR Code for help.";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - 66 - 40, 40, BBEP_WHITE, BBEP_BLACK);
#else // bigger for X
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
    }
    break;
    case WIFI_INTERNAL_ERROR:
    {
        const char string1[] = "WiFi connected, but";
#ifdef __BB_EPAPER__
        int x = 132;
#else
        int x = 0;
#endif
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w)/2, bbep.height() - (rect.h*2)-140);
#endif
        bbep.println(string1);
        const char string2[] = "API connection cannot be";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - x - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "established. Try to refresh,";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - x - rect.w) / 2, -1);
        bbep.println(string3);
        const char string4[] = "or scan QR Code for help.";
        bbep.getStringBox(string4, &rect);
        bbep.setCursor((bbep.width() - x - rect.w) / 2, -1);
        bbep.print(string4);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_failed_qr, 639, 336, BBEP_WHITE, BBEP_BLACK);
#else // bigger for X
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
    }
    break;
    case WIFI_WEAK:
    {
        const char string1[] = "WiFi connected but signal is weak";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case IMAGE_DOWNLOAD_FAILED:
    {
        const char string1[] = "API responded, but image download failed.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Short click the button on back";
#else
        const char string2[] = "Tap the middle of touch bar";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "to advance to the next screen.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case API_UNABLE_TO_CONNECT:
    {
        const char string1[] = "WiFi connected, unable connect to API.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Short click the button on back,";
#else
        const char string2[] = "Tap the middle of touch bar,";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "otherwise check your internet.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case API_SETUP_FAILED:
    {
        const char string1[] = "WiFi connected, /api/setup returned error.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*3));
#endif
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Short click the button on back,";
#else
        const char string2[] = "Tap the middle of touch bar,";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "otherwise check your internet.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case API_SIZE_ERROR:
    {
        const char string1[] = "WiFi connected, TRMNL content malformed.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*2));
#endif
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Wait or reset by holding button on back.";
#else
        const char string2[] = "Wait or reset by holding left and right corner of touch bar.";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case API_FIRMWARE_UPDATE_ERROR:
    {
        const char string1[] = "WiFi connected, could not get firmware update from api.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Wait or reset by holding button on back.";
#else
        const char string2[] = "Wait or reset by holding left and right corner of touch bar.";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case WIFI_IMAGE_TIMEOUT:
    {
        const char string1[] = "Image download timed out; check your network status.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case API_IMAGE_DOWNLOAD_ERROR:
    {
        const char string1[] = "WiFi connected, API could not deliver image to device.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.println(string1);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Wait or reset by holding button on back.";
#else
        const char string2[] = "Wait or reset by holding left and right corner of touch bar.";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case FW_UPDATE:
    {
        const char string1[] = "Firmware update available! Starting now...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case FW_UPDATE_FAILED:
    {
        const char string1[] = "Firmware update failed. Device will restart...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case FW_UPDATE_SUCCESS:
    {
        const char string1[] = "Firmware update success. Device will restart...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case QA_START:
    {
        const char string1[] = "Starting QA test";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(string1);
    }
    break;
    case MSG_TOO_BIG:
    {
        const char string1[] = "The image file from this URL is too large.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 360);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h*4);
#endif
        bbep.println(string1);
        if (strlen(filename) > 40) {
            filename[40] = 0; // truncate and add elipses
            strcat(filename, "...");
        }
        bbep.getStringBox(filename, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(filename);

        const char string2[] = "PNG images can be a maximum of";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
#ifdef __BB_EPAPER__
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each and 1 or 2-bpp");
#else
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each");
#endif
        bbep.getStringBox(string3.c_str(), &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
    break;
    case MSG_FORMAT_ERROR:
    {
        const char string1[] = "The image format is incorrect";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
    }
    break;
    case TEST:
    {
        bbep.setCursor(0, 40);
        bbep.println("ABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIY");
        bbep.println("abcdefghiyabcdefghiyabcdefghiyabcdefghiyabcdefghiy");
        bbep.println("A B C D E F G H I Y A B C D E F G H I Y A B C D E");
        bbep.println("a b c d e f g h i y a b c d e f g h i y a b c d e");
    }
    break;
    case FILL_WHITE:
    {
        Log_info("Display set to white");
#ifdef PARALLEL_EPD
        if (bbep.getMode() == BB_MODE_4BPP) {
            bbep.fillScreen(15); // in 4-bit mode, color 15 = white
        } else {
            bbep.fillScreen(BBEP_WHITE);
        }
#else
        bbep.fillScreen(BBEP_WHITE);
#endif
    }
    break;
    case WIFI_RETRY_LIMIT:
    {
        const char string1[] = "Maximum WiFi retries reached.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*3));
#endif
        bbep.println(string1);
        const char string2[] = "Press button to manually refresh.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "Hold button to reset WiFi and try another network.";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    }
        break;
    case CAPTIVE_WIFI_TIMEOUT:
    {
        const char string1[] = "Wifi Captive Portal timed out";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*2));
#endif
        bbep.println(string1);
#ifdef BOARD_TRMNL_X
        const char string2[] = "Tap touchbar to try again";
#else
        const char string2[] = "Press button to try again";
#endif
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
    }
        break;
    default:
        break;
    }
#ifdef BB_EPAPER
    if (!display_update_epaper(REFRESH_FULL, true, true, PLANE_0)) {
        Log_error("display_show_msg: e-paper update failed");
        bbep.freeBuffer();
        return;
    }
    bbep.freeBuffer();
#else
    Serial.println("FastEPD full update");
    bbep.fullUpdate(CLEAR_SLOW, true);
#endif
    Log_info("display_show_msg end");
}


void display_show_msg_qa(const float *voltage, const float *temperature, bool qa_result)
{
    BB_RECT rect;

    Log_info("display_show_msg start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef BB_EPAPER
    display_prepare_message();
    bbep.fillScreen(BBEP_WHITE); // the results go on a blank screen
#else
    bbep.setMode(BB_MODE_1BPP);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
#endif

    bbep.setFont(nicoclean_8); //Roboto_20);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);

    String voltageString = String("Initial voltage: ")
    + String(voltage[0], 4)
    + String(" V, ")
    + String("  Final voltage: ")
    + String(voltage[1], 4)
    + String(" V, ")
    + String("  Diff: ")
    + String(voltage[2], 4)
    + String(" V");

    String temperatureString = String("Initial temperature: ")
    + String(temperature[0], 4)
    + String(" C, ")
    + String("  Final temperature: ")
    + String(temperature[1], 4)
    + String(" C")
    + String("  Diff: ")
    + String(temperature[2], 4)
    + String(" C");


    bbep.getStringBox(voltageString.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 340);
    bbep.print(voltageString);

    bbep.getStringBox(temperatureString.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 370);
    bbep.print(temperatureString);

    String qaResultInstruction = (qa_result)
    ? "QA passed, press button to clear screen"
    : "QA failed, please use another board and put in failure pile for investigation";

    bbep.getStringBox(qaResultInstruction.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 400);
    bbep.println(qaResultInstruction);

    String qaResultString = (qa_result) ? "PASS" : "FAIL";
    bbep.setFont(Roboto_Black_24);
    bbep.getStringBox(qaResultString.c_str(), &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, 250);
    bbep.print(qaResultString);

    #ifdef BB_EPAPER
        if (!display_update_epaper(REFRESH_FULL, true, true, PLANE_0)) {
            Log_error("display_show_msg_qa: e-paper update failed");
            bbep.freeBuffer();
            return;
        }
        bbep.freeBuffer();
    #else
        bbep.fullUpdate();
    #endif
        Log_info("display_show_msg end");
    /*
     const char string2[] = "PNG images can be a maximum of";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.println(string2);
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each and 1 or 2-bpp");
        bbep.getStringBox(string3.c_str(), &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
    */
}

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @param friendly_id device friendly ID
 * @param id shows if ID exists
 * @param fw_version version of the firmware
 * @param message additional message
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type, String friendly_id, bool id, const char *fw_version, String message)
{
    Log_info("Free heap in display_show_msg - %" PRIu32, ESP.getMaxAllocHeap());
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
#ifdef BB_EPAPER
    display_prepare_message();
    Log_info("Free heap after bbep.allocBuffer() - %" PRIu32, ESP.getMaxAllocHeap());
#else
    bbep.setMode(BB_MODE_1BPP); // message screens are 1-bit
#endif

    if (message_type == WIFI_CONNECT)
    {
        Log_info("Display set to white");
        bbep.fillScreen(BBEP_WHITE);
#ifdef BB_EPAPER
        if (!display_update_epaper(apiDisplayResult.response.maximum_compatibility ? REFRESH_FULL : REFRESH_FAST, true, true, PLANE_0)) {
            Log_error("display_show_msg: WiFi connect update failed");
            bbep.freeBuffer();
            return;
        }
#else
        bbep.fullUpdate();
#endif
        display_sleep(1000);
    }

    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg2 start");

    // Load the image into the bb_epaper framebuffer
    if (image_buffer && *(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        if (x > 0 || y > 0) // only clear if the image is smaller than the display
        {
            bbep.fillScreen(BBEP_WHITE);
        }
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
#ifdef BB_EPAPER
        if (image_buffer) memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
#endif
    }

#if defined( PARALLEL_EPD )
    bbep.setFont(Inter_18);
#else
    bbep.setFont(nicoclean_8);
#endif
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
    switch (message_type)
    {
    case FRIENDLY_ID:
    {
        Log_info("friendly id case");
        const char string1[] = "Please visit trmnl.com/start";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w)/2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w)/2, bbep.height() - 140 - rect.h*2);
#endif
        bbep.println(string1);

        String string2 = "with Friendly ID ";
        if (id)
        {
            string2 += friendly_id;
        }
        string2 += " to finish setup";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, -1);
        bbep.print(string2);
    }
    break;
    case WIFI_CONNECT:
    {
        Log_info("wifi connect case");

        String string1 = "TRMNL firmware ";
        string1 += fw_version;
        bbep.setCursor(40, 48); // place in upper left corner
        bbep.println(string1);
        String string2 = "Connect your phone or computer to ";
        string2 += (message.length() > 0) ? "\"" + message + "\"" : String("the TRMNL");
        string2 += " Wi-Fi";
        bbep.getStringBox(string2, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 386);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 100 - rect.h);
#endif
        bbep.println(string2);
        const char string3[] = "or scan the QR code for help";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string3);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_connect_qr, bbep.width() - 40 - 66, 40, BBEP_WHITE, BBEP_BLACK); // 66x66 QR code
#else // bigger for X
        bbep.loadG5Image(wifi_connect_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
    }
    break;
    case MAC_NOT_REGISTERED:
    {
        UWORD y_start = 340;
#ifdef PARALLEL_EPD
        const uint8_t *pFont = Inter_18;
#else
        const uint8_t *pFont = nicoclean_8;
#endif
        Paint_DrawMultilineText(0, y_start, message.c_str(), width, BBEP_BLACK, BBEP_WHITE, pFont);
    }
    break;
    default:
        break;
    }
    Log_info("Start drawing...");
#ifdef BB_EPAPER
    if (!display_update_epaper(REFRESH_FULL, true, true, PLANE_0)) {
        Log_error("display_show_msg2: e-paper update failed");
        bbep.freeBuffer();
        return;
    }
    bbep.freeBuffer();
#else
    bbep.fullUpdate();
#endif
    Log_info("display_show_msg2 end");
}

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
