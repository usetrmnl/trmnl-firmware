#include <Arduino.h>
#include "png_flip.h"
#include <display.h>
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#include <config.h>
#include <ImageData.h>
#include <ctype.h> //iscntrl()
#include <trmnl_log.h>

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void)
{
    Log_info("dev module start");
    DEV_Module_Init();
    Log_info("dev module end");

    Log_info("screen hw start");
    EPD_7IN5_V2_Init_New();
    Log_info("screen hw end");
}

/**
 * @brief Function to reset the display
 * @param none
 * @return none
 */
void display_reset(void)
{
    Log_info("e-Paper Clear start");
    EPD_7IN5_V2_Clear();
    Log_info("e-Paper Clear end");
    // DEV_Delay_ms(500);
}

/**
 * @brief Function to read the display height
 * @return uint16_t - height of display in pixels
 */
uint16_t display_height()
{
    return EPD_7IN5_V2_HEIGHT;
}

/**
 * @brief Function to read the display width
 * @return uint16_t - width of display in pixels
 */
uint16_t display_width()
{
    return EPD_7IN5_V2_WIDTH;
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
                             UWORD color_fg, UWORD color_bg, sFONT *font,
                             bool is_center_aligned)
{
    uint16_t display_width_pixels = max_width;
    int max_chars_per_line = display_width_pixels / font_width;

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

        Paint_DrawString_EN(draw_x, y_start + j * (font->Height + 5), lines[j], font, color_fg, color_bg);
    }
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
    //  Create a new image cache
    UBYTE *BlackImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;

    Log_info("free heap - %d", ESP.getFreeHeap());
    Log_info("free alloc heap - %d", ESP.getMaxAllocHeap());
    if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
    {
        Log_fatal("Failed to apply for black memory...");
        ESP.restart();
    }
    Log_info("Paint_NewImage %d", reverse);
    
    Paint_NewImage(BlackImage, width, height, 0, WHITE);

    Log_info("show image for array");
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);
    if (reverse)
    {
        Log_info("inverse the image");
        for (size_t i = 0; i < DISPLAY_BMP_IMAGE_SIZE; i++)
        {
            image_buffer[i] = ~image_buffer[i];
        }
    }
    if (isPNG == true)
    {
        Log_info("Drawing PNG");
        flip_image(image_buffer, width, height);
        horizontal_mirror(image_buffer, width, height);
        Paint_DrawBitMap(image_buffer);
    }
    else
    {
        Paint_DrawBitMap(image_buffer + 62);
    }
    EPD_7IN5_V2_Display(BlackImage);
    Log_info("display");

    free(BlackImage);
    BlackImage = NULL;
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
    UBYTE *BlackImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    Log_info("free heap - %d", ESP.getFreeHeap());
    Log_info("free alloc heap - %d", ESP.getMaxAllocHeap());
    if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
    {
        Log_fatal("Failed to apply for black memory...");
        ESP.restart();
    }

    Log_info("Paint_NewImage");
    Paint_NewImage(BlackImage, width, height, 0, WHITE);

    Log_info("show image for array");
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);
    Paint_DrawBitMap(image_buffer + 62);
    switch (message_type)
    {
    case WIFI_CONNECT:
    {
        char string1[] = "Connect to TRMNL WiFi";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
        char string2[] = "on your phone or computer";
        Paint_DrawString_EN((800 - sizeof(string2) * 17 > 9) ? (800 - sizeof(string2) * 17) / 2 + 9 : 0, 430, string2, &Font24, WHITE, BLACK);
    }
    break;
    case WIFI_FAILED:
    {
        char string1[] = "Can't establish WiFi";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 340, string1, &Font24, WHITE, BLACK);
        char string2[] = "connection. Hold button on";
        Paint_DrawString_EN((800 - sizeof(string2) * 17 > 9) ? (800 - sizeof(string2) * 17) / 2 + 9 : 0, 370, string2, &Font24, WHITE, BLACK);
        char string3[] = "the back to reset WiFi";
        Paint_DrawString_EN((800 - sizeof(string3) * 17 > 9) ? (800 - sizeof(string3) * 17) / 2 + 9 : 0, 400, string3, &Font24, WHITE, BLACK);
        char string4[] = "or scan QR Code for help.";
        Paint_DrawString_EN((800 - sizeof(string4) * 17 > 9) ? (800 - sizeof(string4) * 17) / 2 + 9 : 0, 430, string4, &Font24, WHITE, BLACK);

        Paint_DrawImage(wifi_failed_qr, 640, 337, 130, 130);
    }
    break;
    case WIFI_INTERNAL_ERROR:
    {
        char string1[] = "WiFi connected, but";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 340, string1, &Font24, WHITE, BLACK);
        char string2[] = "API connection cannot be";
        Paint_DrawString_EN((800 - sizeof(string2) * 17 > 9) ? (800 - sizeof(string2) * 17) / 2 + 9 : 0, 370, string2, &Font24, WHITE, BLACK);
        char string3[] = "established. Try to refresh,";
        Paint_DrawString_EN((800 - sizeof(string3) * 17 > 9) ? (800 - sizeof(string3) * 17) / 2 + 9 : 0, 400, string3, &Font24, WHITE, BLACK);
        char string4[] = "or scan QR Code for help.";
        Paint_DrawString_EN((800 - sizeof(string4) * 17 > 9) ? (800 - sizeof(string4) * 17) / 2 + 9 : 0, 430, string4, &Font24, WHITE, BLACK);

        Paint_DrawImage(wifi_failed_qr, 640, 337, 130, 130);
    }
    break;
    case WIFI_WEAK:
    {
        char string1[] = "WiFi connected but signal is weak";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
    }
    break;
    case API_ERROR:
    {
        char string1[] = "WiFi connected, TRMNL not responding.";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 340, string1, &Font24, WHITE, BLACK);
        char string2[] = "Short click the button on back,";
        Paint_DrawString_EN((800 - sizeof(string2) * 17 > 9) ? (800 - sizeof(string2) * 17) / 2 + 9 : 0, 400, string2, &Font24, WHITE, BLACK);
        char string3[] = "otherwise check your internet.";
        Paint_DrawString_EN((800 - sizeof(string3) * 17 > 9) ? (800 - sizeof(string3) * 17) / 2 + 9 : 0, 430, string3, &Font24, WHITE, BLACK);
    }
    break;
    case API_SIZE_ERROR:
    {
        char string1[] = "WiFi connected, TRMNL content malformed.";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
        char string2[] = "Wait or reset by holding button on back.";
        Paint_DrawString_EN((800 - sizeof(string2) * 17 > 9) ? (800 - sizeof(string2) * 17) / 2 + 9 : 0, 430, string2, &Font24, WHITE, BLACK);
    }
    break;
    case FW_UPDATE:
    {
        char string1[] = "Firmware update available! Starting now...";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
    }
    break;
    case FW_UPDATE_FAILED:
    {
        char string1[] = "Firmware update failed. Device will restart...";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
    }
    break;
    case FW_UPDATE_SUCCESS:
    {
        char string1[] = "Firmware update success. Device will restart..";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
    }
    break;
    case MSG_FORMAT_ERROR:
    {
        char string1[] = "The image format is incorrect";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);
    }
    break;
    case TEST:
    {
        Paint_DrawString_EN(0, 0, "ABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIYABCDEFGHIY", &Font24, WHITE, BLACK);
        Paint_DrawString_EN(0, 40, "abcdefghiyabcdefghiyabcdefghiyabcdefghiyabcdefghiy", &Font24, WHITE, BLACK);
        Paint_DrawString_EN(0, 80, "A B C D E F G H I Y A B C D E F G H I Y A B C D E", &Font24, WHITE, BLACK);
        Paint_DrawString_EN(0, 120, "a b c d e f g h i y a b c d e f g h i y a b c d e", &Font24, WHITE, BLACK);
    }
    break;
    default:
        break;
    }

    EPD_7IN5_V2_Display(BlackImage);
    Log_info("display");
    free(BlackImage);
    BlackImage = NULL;
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
    if (message_type == WIFI_CONNECT)
    {
        Log_info("Display set to white");
        EPD_7IN5_V2_ClearWhite();
        delay(1000);
    }

    auto width = display_width();
    auto height = display_height();
    UBYTE *BlackImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((width % 8 == 0) ? (width / 8) : (width / 8 + 1)) * height;
    Log_info("free heap - %d", ESP.getFreeHeap());
    Log_info("free alloc heap - %d", ESP.getMaxAllocHeap());
    if ((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL)
    {
        Log_fatal("Failed to apply for black memory...");
        ESP.restart();
    }

    Log_info("Paint_NewImage");
    Paint_NewImage(BlackImage, width, height, 0, WHITE);

    Log_info("show image for array");
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);
    Paint_DrawBitMap(image_buffer + 62);
    switch (message_type)
    {
    case FRIENDLY_ID:
    {
        Log_info("friendly id case");
        char string1[] = "Please sign up at usetrmnl.com/signup";
        Paint_DrawString_EN((800 - sizeof(string1) * 17 > 9) ? (800 - sizeof(string1) * 17) / 2 + 9 : 0, 400, string1, &Font24, WHITE, BLACK);

        String string2 = "with Friendly ID ";
        if (id)
        {
            string2 += friendly_id;
        }
        string2 += " to finish setup";
        Paint_DrawString_EN((800 - string2.length() * 17 > 9) ? (800 - string2.length() * 17) / 2 + 9 : 0, 430, string2.c_str(), &Font24, WHITE, BLACK);
    }
    break;
    case WIFI_CONNECT:
    {
        Log_info("wifi connect case");

        String string1 = "FW: ";
        string1 += fw_version;
        Paint_DrawString_EN((800 - string1.length() * 17 > 9) ? (800 - string1.length() * 17) / 2 + 9 : 0, 340, string1.c_str(), &Font24, WHITE, BLACK);
        char string2[] = "Connect phone or computer";
        Paint_DrawString_EN((800 - sizeof(string2) * 17 > 9) ? (800 - sizeof(string2) * 17) / 2 + 9 : 0, 370, string2, &Font24, WHITE, BLACK);
        char string3[] = "to \"TRMNL\" WiFi network";
        Paint_DrawString_EN((800 - sizeof(string3) * 17 > 9) ? (800 - sizeof(string3) * 17) / 2 + 9 : 0, 400, string3, &Font24, WHITE, BLACK);
        char string4[] = "or scan QR code for help.";
        Paint_DrawString_EN((800 - sizeof(string4) * 17 > 9) ? (800 - sizeof(string4) * 17) / 2 + 9 : 0, 430, string4, &Font24, WHITE, BLACK);

        Paint_DrawImage(wifi_connect_qr, 640, 337, 130, 130);
    }
    break;
    case MAC_NOT_REGISTERED:
    {
        UWORD y_start = 340;
        Paint_DrawMultilineText(0, y_start, message.c_str(), width, Font24.Width, WHITE, BLACK, &Font24, true);
    }
    break;
    default:
        break;
    }
    Log_info("Start drawing...");
    EPD_7IN5_V2_Display(BlackImage);
    Log_info("display");
    free(BlackImage);
    BlackImage = NULL;
}

/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void)
{
    Log_info("Goto Sleep...");
    EPD_7IN5B_V2_Sleep();
}
