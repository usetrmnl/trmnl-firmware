#pragma once

#include <Arduino.h>
#include <cstdarg>

void format_message_truncated(char *buffer, int max_size, const char *format, va_list args);

// Escape special characters (\,") for use inside quoted ESP-AT string parameter.
// https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/index.html
String escape_modem_param(const String &param);