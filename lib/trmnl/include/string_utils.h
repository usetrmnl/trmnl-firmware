#pragma once

#include <Arduino.h>
#include <compiler_attrs.h>
#include <cstdarg>

void format_message_truncated(char *buffer, int max_size, const char *format, va_list args) PRINTF_LIKE(3, 0);

// Escape special characters (\,") for use inside quoted ESP-AT string parameter.
// https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/index.html
String escape_modem_param(const String &param);

// Returns true if the cached image path (e.g. /plugin-1a2b3c-1771674964) starts with one of the
// playlist image names (separated by '|') sent by the server.
bool playlist_has_image(const char *path, const char *names);
