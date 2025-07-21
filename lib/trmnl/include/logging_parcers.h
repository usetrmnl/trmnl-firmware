#pragma once

#include "esp_sleep.h"

bool parseWakeupReasonToStr(char *buffer, size_t buffer_size, esp_sleep_source_t wakeup_reason);

