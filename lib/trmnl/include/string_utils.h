#pragma once

#include <cstdarg>

void format_message_truncated(char* buffer, int max_size, const char* format, va_list args);