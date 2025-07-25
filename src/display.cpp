#include <Arduino.h>
#include <display.h>
#include "DEV_Config.h"
#include "bb_epaper.h"
#include "Group5.h"
#include <config.h>
#include "wifi_connect_qr.h"
#include "wifi_failed_qr.h"
#include <ctype.h> //iscntrl()
#include <trmnl_log.h>
#include "png_flip.h"
#include "../lib/bb_epaper/Fonts/Roboto_Black_16.h"

#if defined(BOARD_SEEED_RETERMINAL_E1002)
BBEPAPER bbep(EP73_SPECTRA_800x480);
#else
BBEPAPER bbep(EP75_800x480);
#endif

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void)
{
    bbep.initIO(EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN, EPD_CS_PIN, EPD_MOSI_PIN, EPD_SCK_PIN, 10000000);
    Log_info("Display init done");
}

/**
 * @brief Function to sleep the ESP32 while saving power
 * @param u32Millis represents the sleep time in milliseconds
 * @return none
 */
void display_sleep(uint32_t u32Millis)
{
  esp_sleep_enable_timer_wakeup(u32Millis * 1000L);
  esp_light_sleep_start();
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
    bbep.refresh(REFRESH_FULL, true);
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
 * @brief Function to draw multi-line text onto the display
 * @param x_start X coordinate to start drawing
 * @param y_start Y coordinate to start drawing
 * @param message Text message to draw
 * @param max_width Maximum width in pixels for each line
 * @param font_width Width of a single character in pixels
 * @param color_fg Foreground color
 * @param color_bg Background color
 * @param font Font to use
 * @param is_center_aligned If true, center the text; if false, left-align
 * @return none
 */
void Paint_DrawMultilineText(UWORD x_start, UWORD y_start, const char *message,
                             uint16_t max_width, uint16_t font_width,
                             UWORD color_fg, UWORD color_bg, const void *font,
                             bool is_center_aligned)
{
    BB_FONT_SMALL *pFont = (BB_FONT_SMALL *)font;
    uint16_t display_width_pixels = max_width;
    int max_chars_per_line = display_width_pixels / font_width;
    const int font_height = pFont->height;
    uint8_t MAX_LINES = 4;

    char lines[MAX_LINES][max_chars_per_line + 1] = {0};
    uint16_t line_count = 0;

    int text_len = strlen(message);
    int current_width = 0;
    int line_index = 0;
    int line_pos = 0;
    int word_start = 0;
    int i = 0;
    char word_buffer[max_chars_per_line + 1] = {0};
    int word_length = 0;

    bbep.setFont(font);
    bbep.setTextColor(color_fg, color_bg);

    while (i <= text_len && line_index < MAX_LINES)
    {
        word_length = 0;
        word_start = i;

        // Skip leading spaces
        while (i < text_len && message[i] == ' ')
        {
            i++;
        }
        word_start = i;

        // Find end of word or end of text
        while (i < text_len && message[i] != ' ')
        {
            i++;
        }

        word_length = i - word_start;
        if (word_length > max_chars_per_line)
        {
            word_length = max_chars_per_line; // Truncate if word is too long
        }

        if (word_length > 0)
        {
            strncpy(word_buffer, message + word_start, word_length);
            word_buffer[word_length] = '\0';
        }
        else
        {
            i++;
            continue;
        }

        int word_width = word_length * font_width;

        // Check if adding the word exceeds max_width
        if (current_width + word_width + (current_width > 0 ? font_width : 0) <= display_width_pixels)
        {
            // Add space before word if not the first word in the line
            if (current_width > 0 && line_pos < max_chars_per_line - 1)
            {
                lines[line_index][line_pos++] = ' ';
                current_width += font_width;
            }

            // Add word to current line
            if (line_pos + word_length <= max_chars_per_line)
            {
                strcpy(&lines[line_index][line_pos], word_buffer);
                line_pos += word_length;
                current_width += word_width;
            }
        }
        else
        {
            // Current line is full, draw it
            if (line_pos > 0)
            {
                lines[line_index][line_pos] = '\0'; // Null-terminate the current line
                line_index++;
                line_count++;

                if (line_index >= MAX_LINES)
                {
                    break;
                }

                // Start new line with this word
                strncpy(lines[line_index], word_buffer, word_length);
                line_pos = word_length;
                current_width = word_width;
            }
            else
            {
                // Single long word case
                strncpy(lines[line_index], word_buffer, max_chars_per_line);
                lines[line_index][max_chars_per_line] = '\0';
                line_index++;
                line_count++;
                line_pos = 0;
                current_width = 0;
            }
        }

        // Move to next word
        if (message[i] == ' ')
        {
            i++;
        }
    }

    // Store the last line if any
    if (line_pos > 0 && line_index < MAX_LINES)
    {
        lines[line_index][line_pos] = '\0';
        line_count++;
    }

    // Draw the lines
    for (int j = 0; j < line_count; j++)
    {
        uint16_t line_width = strlen(lines[j]) * font_width;
        uint16_t draw_x = x_start;

        if (is_center_aligned)
        {
            if (line_width < max_width)
            {
                draw_x = x_start + (max_width - line_width) / 2;
            }
        }
        bbep.setCursor(draw_x, y_start + j * (font_height + 5));
        bbep.print(lines[j]);
    }
}

/**
 * @brief Function to render a bitmap, be compatible with the colored ePaper
 * @param image_buffer pointer to the uint8_t image buffer
 * @return true if the image buffer was allocated, false otherwise
 */
static bool display_render_bitmap(const uint8_t *image_buffer)
{
    bool bAlloc = false;

#if defined(BOARD_SEEED_RETERMINAL_E1002)
    //currently only reTerminal E1002 is using the 7color ePaper, and it's using ESP32-S3, RAM is not an issue
    const unsigned char bmp_header[62] = {
        // BITMAPFILEHEADER (14 bytes)
        0x42, 0x4D,             // bfType: 'BM'
        0x4E, 0xBB, 0x00, 0x00, // bfSize: 48062 bytes = 0x0000BB4E
        0x00, 0x00,             // bfReserved1
        0x00, 0x00,             // bfReserved2
        0x3E, 0x00, 0x00, 0x00, // bfOffBits: 62 bytes (header + palette)

        // BITMAPINFOHEADER (40 bytes)
        0x28, 0x00, 0x00, 0x00, // biSize: 40 bytes
        0x20, 0x03, 0x00, 0x00, // biWidth: 800 px (0x0320)
        0x20, 0xFE, 0xFF, 0xFF, // biHeight: -480 (top-down)
        0x01, 0x00,             // biPlanes: 1
        0x01, 0x00,             // biBitCount: 1bpp
        0x00, 0x00, 0x00, 0x00, // biCompression: BI_RGB (no compression)
        0x80, 0xBB, 0x00, 0x00, // biSizeImage: 48000 bytes (0x0000BB80)
        0x13, 0x0B, 0x00, 0x00, // biXPelsPerMeter: 2835 (72 DPI)
        0x13, 0x0B, 0x00, 0x00, // biYPelsPerMeter: 2835 (72 DPI)
        0x02, 0x00, 0x00, 0x00, // biClrUsed: 2 colors
        0x00, 0x00, 0x00, 0x00, // biClrImportant: 0

        // Color Table (8 bytes)
        0x00, 0x00, 0x00, 0x00, // Color 0: Black (B,G,R,0)
        0xFF, 0xFF, 0xFF, 0x00  // Color 1: White (B,G,R,0)
    };
    bbep.allocBuffer(false);
    bAlloc = true;
    uint8_t *p_buff = (uint8_t *)malloc(DISPLAY_BMP_IMAGE_SIZE);
    memcpy(p_buff, bmp_header, 62);  // fillin a dummy header
    memcpy(p_buff + 62, image_buffer, DISPLAY_BMP_IMAGE_SIZE - 62);
    int ret = bbep.loadBMP(p_buff, 0, 0, BBEP_WHITE, BBEP_BLACK);  //loadBMP will handle bpp for the color ePaper
    Log_verbose_serial("load BMP decoded from PNG, ret: %d", ret);
    free(p_buff);
#else
    bbep.setBuffer((uint8_t *)image_buffer);
#endif
    return bAlloc;
}

/**
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */
void display_show_image(uint8_t *image_buffer, bool reverse, bool isPNG)
{
    auto width = display_width();
    auto height = display_height();
    uint32_t *d32;
    bool bAlloc = false;
    const uint32_t buf_size = ((width + 7)/8) * height; // size in bytes

    Log_info("display_show_image, reverse: %d, isPNG: %d", reverse, isPNG);

    if (reverse)
    {
        d32 = (uint32_t *)image_buffer; // get framebuffer as a 32-bit pointer
        for (size_t i = 0; i < buf_size; i+=sizeof(uint32_t))
        {
            d32[0] = ~d32[0];
            d32++;
        }
    }
    if (isPNG == true)
    {
        Log_info("Drawing PNG");
        bAlloc = display_render_bitmap(image_buffer);
    }
    else // uncompressed BMP or Group5 compressed image
    {
        if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER)
        {
            // G5 compressed image
            Log_verbose_serial("Show G5 compressed bmp");

            BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
            bbep.allocBuffer(false);
            bAlloc = true;
            int x = (width - pBBB->width)/2;
            int y = (height - pBBB->height)/2; // center it
            bbep.fillScreen(BBEP_WHITE); // draw the image centered on a white background
            bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
        }
        else
        { // This work-around is due to a lack of RAM; the correct method would be to use loadBMP()
            flip_image(image_buffer+62, bbep.width(), bbep.height(), false); // fix bottom-up bitmap images
            bAlloc = display_render_bitmap(image_buffer+62); // uncompressed 1-bpp bitmap
        }
    }
    bbep.writePlane(PLANE_0); // send image data to the EPD
    Log_info("Display refresh start");
    bbep.refresh(REFRESH_FULL, true);
    if (bAlloc) {
        bbep.freeBuffer();
    }
    Log_info("Display refresh end");
}

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type)
{
    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg, message_type: %d", message_type);
    bbep.allocBuffer(false);
    if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        Log_verbose_serial("Show G5 compressed bmp");

        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        bbep.fillScreen(BBEP_WHITE); // draw the image centered on a white background
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
        memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
    }

    bbep.setFont(Roboto_Black_16);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);

    switch (message_type)
    {
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
        const char string1[] = "Can't establish WiFi";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w)/2, 380);
        bbep.println(string1);
        const char string2[] = "connection. Hold button on";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "the back to reset WiFi";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.println(string3);
        const char string4[] = "or scan QR Code for help.";
        bbep.getStringBox(string4, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.print(string4);

        bbep.loadG5Image(wifi_failed_qr, 639, 336, BBEP_WHITE, BBEP_BLACK);
    }
    break;
    case WIFI_INTERNAL_ERROR:
    {
        const char string1[] = "WiFi connected, but";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, 340);
        bbep.println(string1);
        const char string2[] = "API connection cannot be";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "established. Try to refresh,";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.println(string3);
        const char string4[] = "or scan QR Code for help.";
        bbep.getStringBox(string4, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.print(string4);

        bbep.loadG5Image(wifi_failed_qr, 639, 336, BBEP_WHITE, BBEP_BLACK);
    }
    break;
    case WIFI_WEAK:
    {
        const char string1[] = "WiFi connected but signal is weak";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(string1);
    }
    break;
    case API_ERROR:
    {
        const char string1[] = "WiFi connected, TRMNL not responding.";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 340);
        bbep.println(string1);
        const char string2[] = "Short click the button on back,";
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
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.println(string1);
        const char string2[] = "Wait or reset by holding button on back.";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, -1);
        bbep.print(string2);
    }
    break;
    case FW_UPDATE:
    {
        const char string1[] = "Firmware update available! Starting now...";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(string1);
    }
    break;
    case FW_UPDATE_FAILED:
    {
        const char string1[] = "Firmware update failed. Device will restart...";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(string1);
    }
    break;
    case FW_UPDATE_SUCCESS:
    {
        const char string1[] = "Firmware update success. Device will restart..";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
        bbep.print(string1);
    }
    break;
    case MSG_FORMAT_ERROR:
    {
        const char string1[] = "The image format is incorrect";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w) / 2, 400);
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
    default:
        break;
    }

    bbep.writePlane(PLANE_0);
    Log_info("Display refresh start");
    bbep.refresh(REFRESH_FULL, true);
    bbep.freeBuffer();
    Log_info("Display refresh end");
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
    Log_info("Free heap before bbep.allocBuffer() - %d", ESP.getMaxAllocHeap());
    bbep.allocBuffer(false);
    Log_info("Free heap after bbep.allocBuffer() - %d", ESP.getMaxAllocHeap());

#if !defined(BOARD_SEEED_RETERMINAL_E1002)    //for E1002 the screen refresh takes long, remove this unnecessary refresh
    if (message_type == WIFI_CONNECT)
    {
        Log_info("Display set to white");
        bbep.fillScreen(BBEP_WHITE);
        bbep.writePlane(PLANE_0);
        bbep.refresh(REFRESH_FULL, true);
        display_sleep(1000);
    }
#endif

    auto width = display_width();
    auto height = display_height();
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    BB_RECT rect;

    Log_info("display_show_msg, message_type: %d, friendly_id: %s, id: %d, fw_version: %s, message: %s",
             message_type, friendly_id.c_str(), id, fw_version, message.c_str());

    // Load the image into the bb_epaper framebuffer
    if (*(uint16_t *)image_buffer == BB_BITMAP_MARKER)
    {
        // G5 compressed image
        Log_verbose_serial("Show G5 compressed bmp");

        BB_BITMAP *pBBB = (BB_BITMAP *)image_buffer;
        int x = (width - pBBB->width)/2;
        int y = (height - pBBB->height)/2; // center it
        bbep.fillScreen(BBEP_WHITE); // draw the image centered on a white background
        bbep.loadG5Image(image_buffer, x, y, BBEP_WHITE, BBEP_BLACK);
    }
    else
    {
        memcpy(bbep.getBuffer(), image_buffer+62, Imagesize); // uncompressed 1-bpp bitmap
    }

    bbep.setFont(Roboto_Black_16);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
    switch (message_type)
    {
    case FRIENDLY_ID:
    {
        Log_info("friendly id case");
        const char string1[] = "Please sign up at usetrmnl.com/signup";
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - rect.w)/2, 400);
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

        String string1 = "FW: ";
        string1 += fw_version;
        bbep.getStringBox(string1, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, 330);
        bbep.println(string1);
        const char string2[] = "Connect phone or computer";
        bbep.getStringBox(string2, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.println(string2);
        const char string3[] = "to \"TRMNL\" WiFi network";
        bbep.getStringBox(string3, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.println(string3);
        const char string4[] = "or scan QR code for help.";
        bbep.getStringBox(string4, &rect);
        bbep.setCursor((bbep.width() - 132 - rect.w) / 2, -1);
        bbep.print(string4);
        bbep.loadG5Image(wifi_connect_qr, 639, 336, BBEP_WHITE, BBEP_BLACK);
    }
    break;
    case MAC_NOT_REGISTERED:
    {
        UWORD y_start = 340;
        UWORD font_width = 16; // DEBUG
        Paint_DrawMultilineText(0, y_start, message.c_str(), width, font_width, BBEP_BLACK, BBEP_WHITE, Roboto_Black_16, true);
    }
    break;
    default:
        break;
    }
    bbep.writePlane(PLANE_0);
    Log_info("Display refresh start");
    bbep.refresh(REFRESH_FULL, true);
    bbep.freeBuffer();
    Log_info("Display refresh end");
}

/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void)
{
    Log_info("Display goto sleep...");
    bbep.sleep(DEEP_SLEEP);
}