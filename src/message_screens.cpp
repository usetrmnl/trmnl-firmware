#include <Arduino.h>
#include <display.h>
#include <power.h>
#include <WiFi.h>
#include "battery_small.h"
#include "battery_hollow.h"
#include "messages.h"
#include "config.h"
#include "battery.h"
#ifdef PARALLEL_EPD
#include "FastEPD.h"
extern FASTEPD bbep; // display.cpp
#else
#define BB_EPAPER
#include <bb_epaper.h>
extern BBEPAPER bbep; // display.cpp
#endif // !PARALLEL_EPD
#include "Group5.h"
#include "wifi_connect_qr.h"
#include "wifi_failed_qr.h"
#include <inttypes.h>
#include <trmnl_log.h>
#include "fonts.h"
#include <globals.h>
#include "epaper_refresh.h"
extern TRMNL_DEVICE *pDevice; // display.cpp

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
