#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "DEV_Config.h"

enum MSG
{
  NONE,
  FRIENDLY_ID,
  WIFI_CONNECT,
  WIFI_FAILED,
  WIFI_WEAK,
  WIFI_INTERNAL_ERROR,
  API_ERROR,
  API_REQUEST_FAILED,
  API_SIZE_ERROR,
  API_UNABLE_TO_CONNECT,
  API_SETUP_FAILED,
  API_IMAGE_DOWNLOAD_ERROR,
  API_FIRMWARE_UPDATE_ERROR,
  FW_UPDATE,
  QA_START,
  FW_UPDATE_FAILED,
  FW_UPDATE_SUCCESS,
  MSG_FORMAT_ERROR,
  MSG_TOO_BIG,
  MAC_NOT_REGISTERED,
  TEST,
  FILL_WHITE,
};

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void);

/**
 * @brief Diagnostic function to continuously show the battery voltage
 * @param none
 * @return none
 */
void display_show_battery(float f);

/**
 * @brief Function to reset the display
 * @param none
 * @return none
 */
void display_reset(void);

/**
 * @brief Function to read the display height
 * @return uint16_t - height of display in pixels
 */
uint16_t display_height();

/**
 * @brief Function to read the display width
 * @return uint16_t - width of display in pixels
 */
uint16_t display_width();

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
                             UWORD color_fg, UWORD color_bg, void *font,
                             bool is_center_aligned);

/**
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */

void display_show_image(uint8_t *image_buffer, int data_size, bool bWait);

/**
 * @brief Function to read an image from the file system
 * @param filename
 * @param pointer to file size returned
 * @return pointer to allocated buffer
 */
uint8_t * display_read_file(const char *filename, int *file_size);

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type);

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @param friendly_id device friendly ID
 * @param id shows if ID exists
 * @param fw_version version of the firmaware
 * @param message additional message
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type, String friendly_id, bool id, const char *fw_version, String message);

/**
 * @brief Function to show the image with API response message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message message text from API response
 * @return none
 */
void display_show_msg_api(uint8_t *image_buffer, String message);

void display_show_msg_qa(uint8_t *image_buffer, const float *voltage, const float *temperature, bool qa_result);

/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void);

#endif
