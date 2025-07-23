#include <string_utils.h>
#include <cstdio>
#include <cstring>

void format_message_truncated(char* buffer, int max_size, const char* format, va_list args) {
    int actual_len = vsnprintf(buffer, max_size, format, args);
    if (actual_len >= max_size) {
        strcpy(buffer + max_size - 4, "...");
    }
}