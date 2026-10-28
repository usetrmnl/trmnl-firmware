#include <board_config.h>
#include <epd_driver.h>
#include <trmnl_log.h>

#ifndef PARALLEL_EPD
#ifndef DEVICE_MODEL
#error "Must have a device model name defined in platformio.ini!"
#endif // DEVICE_MODEL
#endif // !PARALLEL_EPD

TRMNL_DEVICE *pDevice = NULL;

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
  "m5paper",      BB_PANEL_M5PAPER,     BB_PANEL_NONE,         21,    22,    38,    0xff, 0xff,   BATT_NONE,
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
