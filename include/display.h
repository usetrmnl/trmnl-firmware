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

// How many times the panel has been drawn since boot, whatever drew it
uint32_t display_panel_draws(void);

// Whether the next draw is due a full refresh, which clears ghosting
bool display_full_refresh_due(void);

// Redraws the TRMNL OG's panel from one 1-bit frame (1 = white, rows (width + 7) / 8 bytes) to the next with live
// mode's fast balanced waveform, which moves only the pixels that change. begin sends the frame on screen (false on
// other panels), so the caller can then reuse its buffer for the next frame and pass it to finish.
bool display_live_supported(void);
extern String liveSteps; // experiment
bool display_live_begin(const uint8_t *on_screen);
void display_live_finish(const uint8_t *frame);
void display_live_cancel(void); // after begin, when there's no next frame to finish with

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
