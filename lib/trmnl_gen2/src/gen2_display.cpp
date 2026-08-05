// High-level display integration for TRMNL Gen2.
// Wires together gen2_comm / gen2_epd / gen2_pmic / gen2_img.

#include "gen2_display.h"
#include "gen2_comm.h"
#include "gen2_epd.h"
#include "gen2_pmic.h"
//#include "gen2_status.h"
#include "gen2_pins.h"
#include "gen2_img.h"
#include "gen2_battery.h"
#include <Arduino.h>
#include <bb_epaper.h>
#include <PNGdec.h>
extern BBEPAPER bbep;
int png_draw_6clr(PNGDRAW *pDraw);
void CreateSpectra6Pal(void);

void gen2_display_init(void)
{
    Serial.println("[GEN2] gen2_display_init");
    gen2_i2cInit();   // powers EXP_EN, Wire, expanders, PMIC_EN LOW, PMIC_PG/EPD_BUSY as INPUT

    // PMIC power-save off (active-low, write LOW before enabling OUTPUT)
    expander1.write1(EXP1_PMIC_PS, LOW);
    expander1.pinMode1(EXP1_PMIC_PS, OUTPUT);

    // Board-level power rail sequencing (matches main_TEST6_NEW_PCB_full1_wifi.cpp setup())
    setGpioLevel_EXPANDER_2(EXP2_EN_VIN_SCREEN_3V3, GPIO_HIGH);  // 3.3 V screen rail
    delay(50);
    setGpioLevel(EXP1_VNCP_EN, GPIO_HIGH);                         // negative charge pump
    Serial.println("[GEN2] 3V3 screen + VNCP on — waiting 1 s");
    delay(1000);

    setGpioLevel_EXPANDER_2(EXP2_EN_VIN_DC12, GPIO_HIGH);         // 12 V input switch
    delay(500);
    delay(500);  // two consecutive 500 ms delays matching source lines 376+378
    Serial.println("[GEN2] 12 V switch settled");

    setGpioLevel_EXPANDER_2(EXP2_EN_DC12, GPIO_HIGH);             // 12 V rail
    Serial.println("[GEN2] 12 V rail on — waiting 1 s");
    delay(1000);

    setGpioLevel(EXP1_EN_V5A, GPIO_HIGH);                          // 5 V analog
    setGpioLevel_EXPANDER_2(EXP2_EN_V5D, GPIO_HIGH);              // 5 V digital
    Serial.println("[GEN2] 5 V rails on — waiting 500 ms");
    delay(500);

    gen2_spiInit();
    // epdInit() is deliberately deferred to gen2_display_image() so it runs
    // immediately before the panel write, matching the source firmware sequence.
    Serial.println("[GEN2] gen2_display_init complete");
}

bool gen2_display_image(const uint8_t *buf, uint32_t len)
{
int rc;
PNG *png;

    Serial.printf("[GEN2] gen2_display_image: %u bytes\n", len);
    png = new PNG();
    if (!png) {
        Serial.println("Error instantiating PNGdec - out of memory?");
        return false;
    }
    CreateSpectra6Pal(); // create a fast color matching palette
    if (png->openRAM((uint8_t *)buf, len, png_draw_6clr) == PNG_SUCCESS) {
        rc = png->decode(NULL, 0);
        png->close();
    } else {
        Serial.println("Error opening PNG image, aborting...");
        delete(png);
        return false;
    }
    if (rc == PNG_DECODE_ERROR) {
        Serial.println("Error decoding PNG image");
    }
    delete(png); // free the decoder instance

    // Init EPD immediately before write (matches source firmware sequence)
    epdInit();

    if (epdCheckDriverICStatus() != DONE)
        Serial.println("[GEN2] WARNING: one or more EPD driver ICs did not respond");

    // Calibrate PMIC voltage rails by reading VCOM from the panel
    if (epdSetPower() != DONE)
        Serial.println("[GEN2] WARNING: epdSetPower failed — using default PMIC voltages");

    epdWriteImage(bbep.getBuffer());
    epdDisplay();

    // Power down EPD supply rails (10 s total discharge time per E Ink spec)
    powerSwitchDisable();
    return true;
} /* gen2_display_image() */

bool gen2_display_image_url(const char *url)
{
    Serial.printf("[GEN2] gen2_display_image_url: %s\n", url);
    int rc = bbep.createVirtual(2560, 1440, BBEP_7COLOR); // Allocate the framebuffer memory for drawing
    if (rc != BBEP_SUCCESS) {
        Serial.println("bbep.createVirtual() failed - have you enabled PSRAM?");
        return false;
    }
    bbep.fillScreen(BBEP_WHITE); // start with white in case the decode doesn't succeed fully
    bool ok = gen2_imgFetchUrl(url);
    if (!ok) {
        Serial.println("[GEN2] Fetch failed — filling white");
//        gen2_imgSolid(EPD_WHITE);
    }

    epdInit();

    if (epdCheckDriverICStatus() != DONE)
        Serial.println("[GEN2] WARNING: one or more EPD driver ICs did not respond");

    if (epdSetPower() != DONE)
        Serial.println("[GEN2] WARNING: epdSetPower failed — using default PMIC voltages");

    epdWriteImage(bbep.getBuffer());
    epdDisplay();
    powerSwitchDisable();
    bbep.freeBuffer();
    return ok;
}

int gen2_battery_soc(void)
{
    Gen2BatteryStatus s = gen2_batteryRead();
    return s.valid ? (int)s.soc_pct : -1;
}

uint16_t gen2_battery_voltage_mv(void)
{
    Gen2BatteryStatus s = gen2_batteryRead();
    return s.valid ? s.voltage_mV : 0u;
}

bool gen2_battery_power_good(void)
{
    Gen2BatteryStatus s = gen2_batteryRead();
    return s.valid && s.power_good;
}
