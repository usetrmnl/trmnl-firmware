#pragma once

#include <Arduino.h>

// Escape a string for use inside a quoted ESP-AT string parameter.
// Per Espressif's ESP-AT command reference, backslash (\), double quote (")
// and comma (,) are special characters in string parameters and must be
// backslash-escaped — an unescaped comma is treated as a parameter delimiter
// even inside the surrounding quotes.
// https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/index.html
String escapeAtParam(const String& param);
