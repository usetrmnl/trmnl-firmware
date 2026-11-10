#pragma once

#include <Arduino.h>
#include <display.h> // MSG

/**
 * @brief Function to show the image with message on the display
 * @param image_buffer pointer to the uint8_t image buffer
 * @param message_type type of message that will show on the screen
 * @return none
 */
void display_show_msg(uint8_t *image_buffer, MSG message_type, const char *message_text = nullptr);

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
void display_show_msg(uint8_t *image_buffer, MSG message_type, String friendly_id, bool id, const char *fw_version,
                      String message);

void display_show_msg_qa(const float *voltage, const float *temperature, bool qa_result);

#ifdef BOARD_TRMNL_X
/**
 * @brief Fills the progress bar of the MODEM_DOWNLOADING / MODEM_FLASHING screens, partial refresh every 5 %
 */
void display_update_progress(int percent);

/**
 * @brief Replaces the MODEM_FLASHING text with "You're caught up!", partial refresh of that line only
 */
void display_show_modem_up_to_date(void);
#endif // BOARD_TRMNL_X
