#ifndef BL_H
#define BL_H

#include "trmnl_log.h"

/**
 * @brief Function to init business logic module
 * @param none
 * @return none
 */
void bl_init(void);

/**
 * @brief Function to process business logic module
 * @param none
 * @return none
 */
void bl_process(void);

enum LogAction { LOG_ACTION_STORE, LOG_ACTION_SUBMIT, LOG_ACTION_SUBMIT_OR_STORE };

void logWithAction(LogAction action, LogLevel level, const char *message, time_t time, int line, const char *file);

#ifdef BOARD_TRMNL_X
/// @brief Draw the cached playlist image `offset` places from the current one.
/// @param sleep_after false keeps the device awake after the draw instead of sleeping
void show_cached_image_by_offset(int offset, bool sleep_after = true);
#endif

bool submitLogString(const char *log_buffer);
bool storeLogString(const char *log_buffer);

#endif