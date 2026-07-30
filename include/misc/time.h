#pragma once

#include <stdint.h>

/// @brief Current epoch time in seconds, or 0 if the clock has not been synced.
uint32_t getTime(void);
