#include <display_internal.h>
#include <display_widgets.h>
#include <message_screens.h>
#include <globals.h>
#include <trmnl_log.h>

#include "Group5.h"
#include "fonts.h"
#include "messages.h"
#include "wifi_connect_qr.h"
#include "wifi_failed_qr.h"

// The message bodies below keep the indentation they had as cases of the display_show_msg()
// switches, so open changes to those lines still apply; a formatting pass can re-indent them.

// Prints text centered across the screen at row y (-1 = the current row), then moves to the next row.
// A template so it calls the same getStringBox()/println() overload the text's type always did.
template <typename T>
static void draw_centered_line(const T &text, int y)
{
    BB_RECT rect;
    bbep.getStringBox(text, &rect);
    bbep.setCursor((bbep.width() - rect.w) / 2, y);
    bbep.println(text);
}

static void draw_otg_turned_on_message(void)
{
        const char string1[] = "OTG turned on!";
        draw_centered_line(string1, 430);
}

static void draw_otg_turned_off_message(void)
{
        const char string1[] = "OTG turned off!";
        draw_centered_line(string1, 430);
}

static void draw_modem_flashing_message(void)
{
        const char string1[] = "Flashing modem firmware...";
        draw_centered_line(string1, 430);
}

static void draw_modem_flash_failed_message(const char *message_text)
{
        const char string1[] = "Failed to flash modem firmware,";
        draw_centered_line(string1, 430);

        const char string2[] = "device would only operate with 2.4Ghz WiFi.";
        draw_centered_line(string2, 500);
        if (message_text) {
            draw_centered_line(message_text, 570);
        }
}

static void draw_ready_to_ship_message(void)
{
        const char string1[] = "Device is ready to ship!";
        draw_centered_line(string1, 430);

        const char string2[] = "Unplug the USB-C to complete shutdown.";
        draw_centered_line(string2, 500);
}

static void draw_shipping_mode_message(void)
{
        const char string1[] = "Welcome to TRMNL.";
        draw_centered_line(string1, 430);

        const char string2[] = "Attach the dock and a USB-C to get started.";
        draw_centered_line(string2, 500);
}

static void draw_wifi_connect_message(void)
{
        const char string1[] = "Connect to TRMNL WiFi";
        draw_centered_line(string1, 430);
        const char string2[] = "on your phone or computer";
        draw_centered_line(string2, -1);
}

static void draw_wifi_failed_message(void)
{
        BB_RECT rect;

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
        draw_centered_line(string2, -1);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - 66 - 40, 40, BBEP_WHITE, BBEP_BLACK);
#else // bigger for X
        bbep.loadG5Image(wifi_failed_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
}

static void draw_wifi_internal_error_message(void)
{
        BB_RECT rect;

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

static void draw_wifi_weak_message(void)
{
        BB_RECT rect;

        const char string1[] = "WiFi connected but signal is weak";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
}

static void draw_image_download_failed_message(void)
{
        const char string1[] = "API responded, but image download failed.";
        draw_centered_line(string1, 340);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Short click the button on back";
#else
        const char string2[] = "Tap the middle of touch bar";
#endif
        draw_centered_line(string2, -1);
        const char string3[] = "to advance to the next screen.";
        draw_centered_line(string3, -1);
}

static void draw_api_unable_to_connect_message(void)
{
        const char string1[] = "WiFi connected, unable connect to API.";
        draw_centered_line(string1, 340);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Short click the button on back,";
#else
        const char string2[] = "Tap the middle of touch bar,";
#endif
        draw_centered_line(string2, -1);
        const char string3[] = "otherwise check your internet.";
        draw_centered_line(string3, -1);
}

static void draw_api_setup_failed_message(void)
{
        BB_RECT rect;

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
        draw_centered_line(string2, -1);
        const char string3[] = "otherwise check your internet.";
        draw_centered_line(string3, -1);
}

static void draw_api_size_error_message(void)
{
        BB_RECT rect;

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
        draw_centered_line(string2, -1);
}

static void draw_api_firmware_update_error_message(void)
{
        const char string1[] = "WiFi connected, could not get firmware update from api.";
        draw_centered_line(string1, 400);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Wait or reset by holding button on back.";
#else
        const char string2[] = "Wait or reset by holding left and right corner of touch bar.";
#endif
        draw_centered_line(string2, -1);
}

static void draw_wifi_image_timeout_message(void)
{
        BB_RECT rect;

        const char string1[] = "Image download timed out; check your network status.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
}

static void draw_api_image_download_error_message(void)
{
        const char string1[] = "WiFi connected, API could not deliver image to device.";
        draw_centered_line(string1, 400);
#ifndef BOARD_TRMNL_X
        const char string2[] = "Wait or reset by holding button on back.";
#else
        const char string2[] = "Wait or reset by holding left and right corner of touch bar.";
#endif
        draw_centered_line(string2, -1);
}

static void draw_free_text_message(const char *message_text)
{
        BB_RECT rect;

        const char *text = message_text ? message_text : "";
        bbep.getStringBox(text, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(text);
}

static void draw_fw_update_message(void)
{
        BB_RECT rect;

        const char string1[] = "Firmware update available! Starting now...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
}

static void draw_fw_update_failed_message(void)
{
        BB_RECT rect;

        const char string1[] = "Firmware update failed. Device will restart...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
}

static void draw_fw_update_success_message(void)
{
        BB_RECT rect;

        const char string1[] = "Firmware update success. Device will restart...";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
}

static void draw_qa_start_message(void)
{
        const char string1[] = "Starting QA test";
        draw_centered_line(string1, 400);
}

static void draw_msg_too_big_message(void)
{
        BB_RECT rect;

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
        draw_centered_line(filename, -1);

        const char string2[] = "PNG images can be a maximum of";
        draw_centered_line(string2, -1);
#ifdef __BB_EPAPER__
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each and 1 or 2-bpp");
#else
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each");
#endif
        draw_centered_line(string3.c_str(), -1);
}

static void draw_msg_format_error_message(void)
{
        BB_RECT rect;

        const char string1[] = "The image format is incorrect";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - rect.h);
#endif
        bbep.print(string1);
}

static void draw_test_message(void)
{
        bbep.setCursor(0, 40);
        bbep.println("ABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIY");
        bbep.println("abcdefghiyabcdefghiyabcdefghiyabcdefghiyabcdefghiy");
        bbep.println("A B C D E F G H I Y A B C D E F G H I Y A B C D E");
        bbep.println("a b c d e f g h i y a b c d e f g h i y a b c d e");
}

static void draw_fill_white_message(void)
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

static void draw_wifi_retry_limit_message(void)
{
        BB_RECT rect;

        const char string1[] = "Maximum WiFi retries reached.";
        bbep.getStringBox(string1, &rect);
#ifdef __BB_EPAPER__
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
#else
        bbep.setCursor((bbep.width() - rect.w) / 2, bbep.height() - 140 - (rect.h*3));
#endif
        bbep.println(string1);
        const char string2[] = "Press button to manually refresh.";
        draw_centered_line(string2, -1);
        const char string3[] = "Hold button to reset WiFi and try another network.";
        draw_centered_line(string3, -1);
}

static void draw_captive_wifi_timeout_message(void)
{
        BB_RECT rect;

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
        draw_centered_line(string2, -1);
}

// Message screens are 1-bit
static void begin_message_screen(void)
{
#ifdef BB_EPAPER
    display_prepare_message();
#else
    bbep.setMode(BB_MODE_1BPP); // message screens are 1-bit
#endif
}

// Draws the logo (Group5, or an uncompressed 1-bpp BMP) in the middle of the screen
static void draw_centered_logo(uint8_t *image_buffer)
{
    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;

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
}

static void use_message_font(void)
{
#ifdef PARALLEL_EPD
    bbep.setFont(Inter_18);
#else
    bbep.setFont(nicoclean_8);
#endif
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
}

// Refreshes the panel with the finished status message
static void present_status_message(void)
{
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

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type, const char *message_text)
{
    Log_info("display_show_msg start");
    Log_info("maximum_compatibility = %d\n", apiDisplayResult.response.maximum_compatibility);
    begin_message_screen();
    draw_centered_logo(image_buffer);
    use_message_font();
    switch (message_type)
    {
    case OTG_TURNED_ON:
        draw_otg_turned_on_message();
        break;
    case OTG_TURNED_OFF:
        draw_otg_turned_off_message();
        break;
    case MODEM_FLASHING:
        draw_modem_flashing_message();
        break;
    case MODEM_FLASH_FAILED:
        draw_modem_flash_failed_message(message_text);
        break;
    case READY_TO_SHIP:
        draw_ready_to_ship_message();
        break;
    case SHIPPING_MODE:
        draw_shipping_mode_message();
        break;
    case WIFI_CONNECT:
        draw_wifi_connect_message();
        break;
    case WIFI_FAILED:
        draw_wifi_failed_message();
        break;
    case WIFI_INTERNAL_ERROR:
        draw_wifi_internal_error_message();
        break;
    case WIFI_WEAK:
        draw_wifi_weak_message();
        break;
    case IMAGE_DOWNLOAD_FAILED:
        draw_image_download_failed_message();
        break;
    case API_UNABLE_TO_CONNECT:
        draw_api_unable_to_connect_message();
        break;
    case API_SETUP_FAILED:
        draw_api_setup_failed_message();
        break;
    case API_SIZE_ERROR:
        draw_api_size_error_message();
        break;
    case API_FIRMWARE_UPDATE_ERROR:
        draw_api_firmware_update_error_message();
        break;
    case WIFI_IMAGE_TIMEOUT:
        draw_wifi_image_timeout_message();
        break;
    case API_IMAGE_DOWNLOAD_ERROR:
        draw_api_image_download_error_message();
        break;
    case MSG_FREE_TEXT:
        draw_free_text_message(message_text);
        break;
    case FW_UPDATE:
        draw_fw_update_message();
        break;
    case FW_UPDATE_FAILED:
        draw_fw_update_failed_message();
        break;
    case FW_UPDATE_SUCCESS:
        draw_fw_update_success_message();
        break;
    case QA_START:
        draw_qa_start_message();
        break;
    case MSG_TOO_BIG:
        draw_msg_too_big_message();
        break;
    case MSG_FORMAT_ERROR:
        draw_msg_format_error_message();
        break;
    case TEST:
        draw_test_message();
        break;
    case FILL_WHITE:
        draw_fill_white_message();
        break;
    case WIFI_RETRY_LIMIT:
        draw_wifi_retry_limit_message();
        break;
    case CAPTIVE_WIFI_TIMEOUT:
        draw_captive_wifi_timeout_message();
        break;
    default:
        break;
    }
    present_status_message();
}


void display_show_msg_qa(const float *voltage, const float *temperature, bool qa_result)
{
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


    draw_centered_line(voltageString.c_str(), 340);

    draw_centered_line(temperatureString.c_str(), 370);

    String qaResultInstruction = (qa_result)
    ? "QA passed, press button to clear screen"
    : "QA failed, please use another board and put in failure pile for investigation";

    draw_centered_line(qaResultInstruction.c_str(), 400);

    String qaResultString = (qa_result) ? "PASS" : "FAIL";
    bbep.setFont(Roboto_Black_24);
    draw_centered_line(qaResultString.c_str(), 250);

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
        draw_centered_line(string2, -1);
        String string3 = String(MAX_IMAGE_SIZE) + String(" bytes each and 1 or 2-bpp");
        draw_centered_line(string3.c_str(), -1);
    */
}

// Clears the screen to white before the Wi-Fi setup screen; false when the update failed
static bool clear_for_wifi_connect_setup(void)
{
        Log_info("Display set to white");
        bbep.fillScreen(BBEP_WHITE);
#ifdef BB_EPAPER
        if (!display_update_epaper(apiDisplayResult.response.maximum_compatibility ? REFRESH_FULL : REFRESH_FAST, true, true, PLANE_0)) {
            Log_error("display_show_msg: WiFi connect update failed");
            bbep.freeBuffer();
            return false;
        }
#else
        bbep.fullUpdate();
#endif
        display_sleep(1000);
        return true;
}

static void draw_friendly_id_setup_message(String friendly_id, bool id)
{
        BB_RECT rect;

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
        draw_centered_line(string2, -1);
}

static void draw_wifi_connect_setup_message(const char *fw_version, String message)
{
        BB_RECT rect;

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
        draw_centered_line(string3, -1);
#ifdef __BB_EPAPER__
        bbep.loadG5Image(wifi_connect_qr, bbep.width() - 40 - 66, 40, BBEP_WHITE, BBEP_BLACK); // 66x66 QR code
#else // bigger for X
        bbep.loadG5Image(wifi_connect_qr, bbep.width() - (66*2) - 80, 80, BBEP_WHITE, BBEP_BLACK, 2.0f);
#endif
}

static void draw_mac_not_registered_setup_message(String message)
{
        auto width = display_width();
        UWORD y_start = 340;
#ifdef PARALLEL_EPD
        const uint8_t *pFont = Inter_18;
#else
        const uint8_t *pFont = nicoclean_8;
#endif
        Paint_DrawMultilineText(0, y_start, message.c_str(), width, BBEP_BLACK, BBEP_WHITE, pFont);
}

// Refreshes the panel with the finished setup message
static void present_setup_message(void)
{
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
    begin_message_screen();
#ifdef BB_EPAPER
    Log_info("Free heap after bbep.allocBuffer() - %" PRIu32, ESP.getMaxAllocHeap());
#endif

    if (message_type == WIFI_CONNECT && !clear_for_wifi_connect_setup())
    {
        return;
    }

    Log_info("display_show_msg2 start");
    draw_centered_logo(image_buffer);
    use_message_font();
    switch (message_type)
    {
    case FRIENDLY_ID:
        draw_friendly_id_setup_message(friendly_id, id);
        break;
    case WIFI_CONNECT:
        draw_wifi_connect_setup_message(fw_version, message);
        break;
    case MAC_NOT_REGISTERED:
        draw_mac_not_registered_setup_message(message);
        break;
    default:
        break;
    }
    Log_info("Start drawing...");
    present_setup_message();
}

#ifdef BYOS_PROTOCOL_V1
// Declared in display.h upstream but never defined; BYOS v1 uses it for "waiting for update".
void display_show_msg_api(uint8_t *image_buffer, String message)
{
    display_show_msg(image_buffer, MSG_FREE_TEXT, message.c_str());
}
#endif
