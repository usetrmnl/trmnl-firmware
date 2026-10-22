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
#define MAX_BIT_DEPTH 8
TRMNL_DEVICE *pDevice = NULL;
#ifdef PARALLEL_EPD
#include "FastEPD.h"
FASTEPD bbep;
#else
#define BB_EPAPER
#include <bb_epaper.h>
BBEPAPER bbep;
#ifndef DEVICE_MODEL
#error "Must have a device model name defined in platformio.ini!"
#endif // DEVICE_MODEL
#include <SPIFFS.h>
#define FS SPIFFS
#endif // !PARALLEL_EPD

// List of supported TRMNL devices with SPI ePaper displays. The list can be in any order since the name is matched
// The final parameter is the panel type which is from an enumerated list
#ifdef PARALLEL_EPD
const TRMNL_DEVICE device_list[] = 
{
// name           board_name            panel_type.            sda    scl    intr   batt  batt_en batt_type
  "x",            BB_PANEL_TRMNL_X,     BB_PANEL_NONE,         0xff,  0xff,  0xff,  0xff, 0xff,   BATT_ADC,
  "m5_papers3",   BB_PANEL_M5PAPERS3,   BB_PANEL_NONE,         0xff,  0xff,  0xff,  3,    0xff,   BATT_ADC,
  "sensoria_c5",  BB_PANEL_SENSORIA_C5, BB_PANEL_NONE,         7,     6,     0,     0xff, 0xff,   BATT_ADC,
  "lilygo_t5pro", BB_PANEL_EPDIY_V7,    BBEP_DISPLAY_ED047TC1, 39,    40,    0,     0xff, 0xff,   BATT_BQ27220,
  "reterminal_e1003", BB_PANEL_NONE,    BB_PANEL_NONE,         0xff,  0xff,  3,     1,    40,     BATT_ADC, // IT8951 init is in display_init()
  NULL, 0, 0, 0, 0, 0, 0, 0, 0,
}; // Parallel Eink device list

#else // BB_EPAPER boards
const TRMNL_DEVICE device_list[] = 
{
// name            sck   mosi   cs   rst   dc   busy  sda   scl   intr   batt  batt_en, epd_en  batt_type, panel
  "og",            7,    8,     6,   10,   5,   4,    21,   20,   2,     3,    0xff,    0xff,   BATT_ADC,  EPD_75,
  "og_4clr",       7,    8,     6,   10,   5,   4,    21,   20,   2,     3,    0xff,    0xff,   BATT_ADC,  EPD_75_4CLR,
  "og_gen2",       6,    1,     4,   2,    5,   0,    23,   10,   3,     0xff, 0xff,    0xff,   BATT_BQ27427,  EPD_75, // fake battery == 0xff
  "og_gen2_4clr",  6,    1,     4,   2,    5,   0,    23,   10,   3,     0xff, 0xff,    0xff,   BATT_BQ27427,  EPD_75_4CLR, // fake battery == 0xff
  "xteink_x4",     8,    10,    21,  5,    4,   6,    0xff, 0xff, 3,     0,    0xff,    0xff,   BATT_ADC,  EPD_426,
  "xteink_x3",     8,    10,    21,  5,    4,   6,    20,   0,    3,     0xff, 0xff,    0xff,   BATT_BQ27220,  EPD_368,
  "waveshare",     13,   14,    15,  26,   27,  25,   0xff, 0xff, 33,    0xff, 0xff,    0xff,   BATT_ADC,  EPD_75,
  "waveshare_397", 11,   12,    10,  46,   9,   3,    41,   42,   0,     0xff, 0xff,    0xff,   BATT_AXP2101,  EPD_397,
  "seeed_sticky",  13,   14,    15,  17,   16,  18,   1,    0,    4,     0xff, 0xff,    0xff,   BATT_BQ27220,  EPD_397,  
  "seeed_esp32c3", 8,    10,    3,   2,    5,   4,    0xff, 0xff, 9,     0xff, 0xff,    0xff,   BATT_ADC,  EPD_75,
  "seeed_esp32s3", 7,    9,     2,   1,    4,   3,    0xff, 0xff, 0,     0xff, 0xff,    0xff,   BATT_ADC,  EPD_75,
  "xiao_epaper_mini", 7, 9,     44,  38,   10,  4,    0xff, 0xff, 2,     1,    6,       0xff,   BATT_ADC,  EPD_426,
  "xiao_epaper_display", 7, 9,  44,  38,   10,  4,    0xff, 0xff, 5,     1,    6,       0xff,   BATT_ADC,  EPD_75,
  "xiao_epaper_3clr", 7, 9,     44,  38,   10,  4,    0xff, 0xff, 5,     1,    6,       0xff,   BATT_ADC,  EPD_75_3CLR,
  "xiao_epaper_6clr", 7, 9,     44,  38,   10,  4,    0xff, 0xff, 5,     1,    6,       0xff,   BATT_ADC,  EPD_75_6CLR,
  "reterminal_e1001", 7, 9,     10,  12,   11,  13,   0xff, 0xff, 3,     1,    21,      0xff,   BATT_ADC,  EPD_75,
  "reterminal_e1002", 7, 9,     10,  12,   11,  13,   0xff, 0xff, 3,     1,    21,      12,     BATT_ADC,  EPD_75_6CLR,
  "crowpanel42",   0,    0,     0,   0,    0,   0,    0xff, 0xff, 2,     0xff, 0xff,    0xff,   BATT_NONE, EPD_CROWPANEL, 
  "m5_paper_mono", 0,    0,     0,   0,    0,   0,    47,   48,   2,     0xff, 0xff,    0xff,   BATT_NONE, EPD_PAPER_MONO, 
  "m5_paper_color", 0,   0,     0,   0,    0,   0,    3,    2,    1,     0xff, 0xff,    0xff,   BATT_NONE, EPD_PAPER_COLOR, 
  "reterminal_e1004", 0, 0,     0,   0,    0,   0,    0xff, 0xff, 4,     1,    21,      12,     BATT_ADC,  EPD_133_COLOR,
  "trmnl_steam",   7,    8,     6,   10,   5,   4,    21,   20,   2,     3,    0xff,    0xff,   BATT_ADC,  EPD_583,
  NULL,            0,    0,     0,   0,    0,   0,    0,    0,    0,     0,    0,       0,      0,         0,
}; // device_list

// TRMNL SPI ePaper panel types list. The list order is fixed and based on enumerated values
// N.B. ALWAYS ADD NEW PANELS TO THE END OF THE LIST
const DISPLAY_PROFILE dpList[12][3] = { // 1-bit and 2-bit display types for each profile
    {{EP75_800x480, EP75_800x480_4GRAY}, {EP75_800x480_GEN2, EP75_800x480_4GRAY_GEN2}, {EP75_800x480, EP75_800x480_4GRAY_V2}},
    {{EP426_800x480, EP426_800x480_4GRAY}, {EP426_800x480, EP426_800x480_4GRAY}, {EP426_800x480, EP426_800x480_4GRAY}},
    {{EP397_800x480, EP397_800x480_4GRAY}, {EP397_800x480, EP397_800x480_4GRAY}, {EP397_800x480, EP397_800x480_4GRAY}},
    {{EP75R_800x480, EP75R_800x480}, {EP75R_800x480, EP75R_800x480}, {EP75R_800x480, EP75R_800x480}}, 
    {{EP75YR_800x480, EP75YR_800x480}, {EP75YR_800x480, EP75YR_800x480}, {EP75YR_800x480, EP75YR_800x480}}, 
    {{EP73_SPECTRA_800x480, EP73_SPECTRA_800x480}, {EP73_SPECTRA_800x480, EP73_SPECTRA_800x480}, {EP73_SPECTRA_800x480, EP73_SPECTRA_800x480}},
    {{EPD_CROWPANEL42, EPD_CROWPANEL42_4GRAY},{EPD_CROWPANEL42, EPD_CROWPANEL42_4GRAY},{EPD_CROWPANEL42, EPD_CROWPANEL42_4GRAY}},
    {{EP583_648x480, EP583_648x480_4GRAY}, {EP583_648x480, EP583_648x480_4GRAY}, {EP583_648x480, EP583_648x480_4GRAY}},
    {{EPD_M5_PAPER_MONO, EPD_M5_PAPER_MONO_4GRAY},{EPD_M5_PAPER_MONO, EPD_M5_PAPER_MONO_4GRAY},{EPD_M5_PAPER_MONO, EPD_M5_PAPER_MONO_4GRAY}},
    {{EPD_M5_PAPER_COLOR, EPD_M5_PAPER_COLOR},{EPD_M5_PAPER_COLOR, EPD_M5_PAPER_COLOR},{EPD_M5_PAPER_COLOR, EPD_M5_PAPER_COLOR}},
    {{EPD_SEEED_E1004, EPD_SEEED_E1004},{EPD_SEEED_E1004, EPD_SEEED_E1004},{EPD_SEEED_E1004, EPD_SEEED_E1004}},
    {{EP368_792x528, EP368_792x528_4GRAY}, {EP368_792x528, EP368_792x528_4GRAY}, {EP368_792x528, EP368_792x528_4GRAY}},
};
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
#include "image_decoders.h"
#include "epaper_refresh.h"

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

static void display_select_one_bit_panel(bool only_if_changed = false)
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

static void WriteSPIByte(uint8_t data)
{
  for (int i=0; i<8; i++) {
    digitalWrite(pDevice->epd_mosi_pin, (data & 0x80) ? 1:0);
    digitalWrite(pDevice->epd_sck_pin, 1);
    digitalWrite(pDevice->epd_sck_pin, 0);
    data <<= 1;
  }
}

static uint8_t ReadSPIByte(void)
{
uint8_t u8 = 0;
  for (int i=0; i<8; i++) {
    digitalWrite(pDevice->epd_sck_pin, 1);
    digitalWrite(pDevice->epd_sck_pin, 0);
    u8 <<= 1;
    u8 |= digitalRead(pDevice->epd_mosi_pin);
  }
return u8;
}
/**
 * @brief Function to read the EPD revision to know what temperature profile to use
 * @param none
 * @return 32-bit value read from the panel's revision (REV) command
 */
uint32_t get_panel_rev(void)
{
    uint32_t u32 = 0;
    uint8_t u8;

    if (pDevice->epd_mosi_pin == 0 && pDevice->epd_sck_pin == 0) { // pre-defined PCB+display in bb_epaper; pins unknown
        return 0;
    }
    if (pDevice->panel_set != EPD_75) { // REV (0x70) is only known to be a harmless read on the 7.5" B/W UC8179
        return 0;
    }

    // Initialize the SPI bus in 'bit-bang' mode before running the normal init sequence
    // This will allow us to use MOSI as a bidirectional line for reading data from the panel
    pinMode(pDevice->epd_sck_pin, OUTPUT);
    pinMode(pDevice->epd_cs_pin, OUTPUT);
    digitalWrite(pDevice->epd_cs_pin, 1);
    pinMode(pDevice->epd_rst_pin, OUTPUT);
    pinMode(pDevice->epd_dc_pin, OUTPUT);
    pinMode(pDevice->epd_busy_pin, INPUT);

    // Reset the panel
    digitalWrite(pDevice->epd_rst_pin, 0);
    delay(20);
    digitalWrite(pDevice->epd_rst_pin, 1);
    delay(20);
    if (digitalRead(pDevice->epd_busy_pin) == 0) { // it's a SSD16xx; not useful at the moment because our 7.5" B/W is a UC81xx type panel
        return 0;
    }
    digitalWrite(pDevice->epd_dc_pin, 0); // command mode
    digitalWrite(pDevice->epd_cs_pin, 0);
    pinMode(pDevice->epd_mosi_pin, OUTPUT);
    WriteSPIByte(0x70); // Revision (REV) command
    digitalWrite(pDevice->epd_dc_pin, 1); // data mode
    pinMode(pDevice->epd_mosi_pin, INPUT);
    for (int i=0; i<7; i++) {
        u8 = ReadSPIByte();
        if (i >= 3) { // first 3 bytes are 0xff
            u32 <<= 8;
            u32 |= u8;
        }
    }
    digitalWrite(pDevice->epd_cs_pin, 1);
    return u32;
} /* get_panel_rev() */
#endif // BB_EPAPER

String display_panel_rev_string(void)
{
    if (panel_rev == 0) {
        return String();
    }
    char sz[9];
    snprintf(sz, sizeof(sz), "%08" PRIx32, panel_rev);
    return String(sz);
} /* display_panel_rev_string() */

void hw_config_init(void)
{
    int i = 0;
    // Match the device name with the configuration in the list
    while (device_list[i].device_name && strcmp(device_list[i].device_name, DEVICE_MODEL) != 0) {
        i++;
    }
    if (device_list[i].device_name) {
        Log_info("Found device model at index %d\n", i);
        pDevice = (TRMNL_DEVICE *)&device_list[i];
    } else {
        Log_info("Device name (%s) not found in supported list!", DEVICE_MODEL);
    }
} /* hw_config_init() */

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

#ifdef BOARD_TRMNL_X

#define TCA9535_INT 38

void BQ27427_reset()
{
    bbep.ioPinMode(10, OUTPUT);

    bbep.ioWrite(10, LOW);
    delay(10);

    bbep.ioWrite(10, HIGH);
    Serial.println("BQ27427 reset performed");
}

void config_tca95535_pins_for_lp()
{
    bbep.ioPinMode(0, INPUT);

    // Pin 1 (OTG) is configured separately in init_otg()

    bbep.ioPinMode(2, INPUT);

    bbep.ioPinMode(3, INPUT);

    bbep.ioPinMode(4, INPUT);

    bbep.ioPinMode(5, INPUT);

    bbep.ioPinMode(6, OUTPUT);
    bbep.ioWrite(6, LOW);

    bbep.ioPinMode(7, INPUT);

    bbep.ioPinMode(8, OUTPUT);
    bbep.ioWrite(8, LOW);

    bbep.ioPinMode(9, OUTPUT);
    bbep.ioWrite(9, LOW);

    bbep.ioPinMode(10, INPUT);

    bbep.ioPinMode(11, OUTPUT);
    bbep.ioWrite(11, LOW);

    bbep.ioPinMode(12, OUTPUT);
    bbep.ioWrite(12, LOW);

    bbep.ioPinMode(13, OUTPUT);
    bbep.ioWrite(13, LOW);

    bbep.ioPinMode(14, INPUT);

    bbep.ioPinMode(15, INPUT);
}

void config_bma530_interrupt()
{
    bbep.ioPinMode(3, INPUT);
    bbep.ioRead(10);
}

uint8_t tca9535_interrupt_clear()
{
    return bbep.ioRead(3);
}

void otg_turn_on()
{
    bbep.ioPinMode(1, OUTPUT);
    bbep.ioWrite(1, HIGH);
    Log_info("OTG turned on");
}

void otg_turn_off()
{
    bbep.ioPinMode(1, OUTPUT);
    bbep.ioWrite(1, LOW);
    Log_info("OTG turned off");
}

#define BAT_DET_PIN       7    // TCA9535 P0_7 in bbep pin numbering
#define BAT_CHARGE_MS     2    // drive HIGH for 2 ms to charge RC network
#define BAT_TIMEOUT_US    6000 // no battery if pin is still HIGH after this
#define BAT_THRESHOLD_US  750 // >750 µs → 1 cell; ≤750 µs → 2 cells

static battery_count_t measure_battery_once()
{
    // -- Charge RC network: drive P0_7 HIGH for 2 ms --
    bbep.ioPinMode(BAT_DET_PIN, OUTPUT);
    bbep.ioWrite(BAT_DET_PIN, HIGH);
    delay(BAT_CHARGE_MS);

    // -- Release: switch to input and start timing --
    bbep.ioPinMode(BAT_DET_PIN, INPUT);

    unsigned long start = micros();
    unsigned long now   = start;
    bool timeout = false;

    while (true) {
        bool still_high = (bbep.ioRead(BAT_DET_PIN) != 0);
        now = micros();

        if (!still_high) break;               // pin went LOW → discharged

        if ((now - start) >= BAT_TIMEOUT_US) {
            timeout = true;
            break;
        }
    }

    if (timeout)                         return BATTERY_NONE;
    if ((now - start) > BAT_THRESHOLD_US) return BATTERY_ONE;
    return BATTERY_TWO;
}

battery_count_t detect_battery_count()
{
    battery_count_t last = (battery_count_t)-1;

    for (int attempt = 0; attempt < 20; attempt++) {
        battery_count_t result = measure_battery_once();

        if (result == last) {
            // Two identical results in a row — confident reading
            Log_info("Battery detection: %s (confirmed after %d attempt(s))",
                result == BATTERY_NONE ? "none" :
                result == BATTERY_ONE  ? "1 cell" : "2 cells",
                attempt + 1);
            return result;
        }

        last = result;
        delay(10);
    }

    // Fallback: return last reading if we never got two in a row
    Log_error("Battery detection: unstable reading — defaulting to last result");
    return last;
}

void enter_shipment_sleep()
{
    config_tca95535_pins_for_lp();
    bbep.ioPinMode(0, INPUT);  // Pin 0 = charger detect

    // Clear any pending TCA9535 interrupts
    for (uint8_t pin = 0; pin < 16; pin++) {
        bbep.ioPinMode(pin, INPUT);
        bbep.ioRead(pin);
    }
    delay(50);

    pinMode(TCA9535_INT, INPUT_PULLUP);

    // Check initial GPIO38 state
    int gpio38_state = digitalRead(TCA9535_INT);

    // display_sleep() leaves its light-sleep timer armed, which would wake us every second.
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);

    // Enable GPIO wakeup for light sleep
    esp_err_t err = esp_sleep_enable_gpio_wakeup();
    if (err != ESP_OK) {
        Serial.printf("ERROR: Failed to enable GPIO wakeup: %d\n", err);
        return;
    }

    err = gpio_wakeup_enable((gpio_num_t)TCA9535_INT, GPIO_INTR_LOW_LEVEL);
    if (err != ESP_OK) {
        Serial.printf("ERROR: Failed to configure GPIO38 wakeup: %d\n", err);
        return;
    }

    Serial.println("=== Entering shipment mode light sleep loop ===");
    delay(100);

    uint32_t sleep_count = 0;

    // Main sleep loop
    while (true) {
        sleep_count++;

        esp_light_sleep_start();

        // Re-initialize Serial after light sleep (USB may have been disabled)
        delay(100);
        Serial.begin(115200);
        delay(100);

        Serial.println("\n=== WAKEUP from light sleep ===");
        Serial.printf("Sleep cycle: %lu\n", sleep_count);

        // Check wakeup cause
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
        Serial.printf("Wakeup cause: %d (GPIO=%d)\n", wakeup_reason, ESP_SLEEP_WAKEUP_GPIO);

        if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
            Serial.println("GPIO wakeup detected");

            // Check GPIO38 state
            gpio38_state = digitalRead(TCA9535_INT);
            Serial.printf("GPIO38 state after wakeup: %d\n", gpio38_state);

            // Check if charger connected (TCA9535 pin 0 LOW)
            bbep.ioPinMode(0, INPUT);
            delay(10);
            uint8_t pin0_state = bbep.ioRead(0);
            Serial.printf("TCA9535 pin 0 (charger detect): %d (0=charger present)\n", pin0_state);

            if (pin0_state == 0) {
                Serial.println("*** CHARGER DETECTED - Exiting shipment mode ***");
                Serial.flush();
                delay(100);
                break;  // Exit shipment mode
            }

            // False wakeup - clear interrupt and continue
            for (uint8_t pin = 0; pin < 16; pin++) {
                bbep.ioPinMode(pin, INPUT);
                bbep.ioRead(pin);
            }
            delay(50);

            gpio38_state = digitalRead(TCA9535_INT);
            Serial.printf("GPIO38 after clear: %d\n", gpio38_state);

        } else {
            Serial.printf("Unexpected wakeup cause: %d\n", wakeup_reason);
        }

        Serial.println("Returning to sleep...\n");
        Serial.flush();
        delay(100);
    }

    gpio_wakeup_disable((gpio_num_t)TCA9535_INT);
    gpio_set_intr_type((gpio_num_t)TCA9535_INT, GPIO_INTR_DISABLE);
    Serial.println("=== Exited shipment mode successfully ===");
    Serial.flush();
}

#define PIN_ESP32C5_SPI_BOOT 4
#define PIN_ESP32C5_USB_BOOT 5
#define PIN_ESP32C5_EN 6

void modem_enter_bootloader(void) {
  // Disable target and wait for full power down
  bbep.ioWrite(PIN_ESP32C5_EN, 0);
  delay(100);

  // Configure boot pins for bootloader mode
  bbep.ioPinMode(PIN_ESP32C5_SPI_BOOT, OUTPUT);
  bbep.ioPinMode(PIN_ESP32C5_USB_BOOT, OUTPUT);
  bbep.ioWrite(PIN_ESP32C5_SPI_BOOT, 0);
  bbep.ioWrite(PIN_ESP32C5_USB_BOOT, 1);
  delay(50);

  // Enable target in bootloader mode
  bbep.ioWrite(PIN_ESP32C5_EN, 1);
  delay(300);
}

void modem_reset_target(void) {
  // Disable target
  bbep.ioWrite(PIN_ESP32C5_EN, 0);
  delay(50);

  // Release boot pins (set to input for normal boot)
  bbep.ioPinMode(PIN_ESP32C5_SPI_BOOT, INPUT);
  bbep.ioPinMode(PIN_ESP32C5_USB_BOOT, INPUT);
  delay(50);

  // Enable target (normal boot)
  bbep.ioWrite(PIN_ESP32C5_EN, 1);
}

#endif

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
