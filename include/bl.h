#ifndef BL_H
#define BL_H

#include "display.h"
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

/**
 * @brief Prepares peripherals for deep sleep and enters it. Does not return.
 */
void goToSleep(void);

/**
 * @brief Clears stored WiFi/API credentials and restarts the device.
 */
void resetDeviceCredentials(void);

#endif