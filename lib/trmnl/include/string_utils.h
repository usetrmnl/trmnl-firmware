#pragma once

#include <cstdarg>
#include <compiler_attrs.h>

void format_message_truncated(char* buffer, int max_size, const char* format, va_list args) PRINTF_LIKE(3, 0);