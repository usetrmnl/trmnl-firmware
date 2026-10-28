#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#ifndef _NO_DEV_CONFIG_
#include "config.h"
#endif
#include "x_hardware.h" // the TRMNL X IO expander functions were declared here; existing callers rely on it

enum MSG {
  NONE,
  FRIENDLY_ID,
  OTG_TURNED_ON,
  OTG_TURNED_OFF,
  MODEM_FLASHING,
  MODEM_FLASH_FAILED,
  READY_TO_SHIP,
  SHIPPING_MODE,
  WIFI_CONNECT,
  WIFI_FAILED,
  WIFI_WEAK,
  WIFI_INTERNAL_ERROR,
  WIFI_IMAGE_TIMEOUT,
  API_ERROR,
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
  WIFI_RETRY_LIMIT,
  CAPTIVE_WIFI_TIMEOUT,
  IMAGE_DOWNLOAD_FAILED,
};

typedef struct theBrand {
  char name[16];
  char api_url[128];
  uint8_t u8Images[3952];
} BRAND;

/**
 * @brief Function to init the display
 * @param none
 * @return none
 */
void display_init(void);

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
 * @brief Function to show the image on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param reverse shows if the color scheme is reverse
 * @return none
 */

void display_show_image(uint8_t *image_buffer, int data_size, bool bWait, bool bSkipClear = false);

/**
 * @brief Enable or disable light sleep at runtime
 * @param enabled true to enable light sleep, false to disable
 * @return none
 */
void display_set_light_sleep(uint8_t enabled);

/**
 * @brief Function to got the display to the sleep
 * @param none
 * @return none
 */
void display_sleep(void);

/**
 * @brief Function to wipe the display
 * @param none
 * @return none
 */
void display_wipe(void);

#endif
