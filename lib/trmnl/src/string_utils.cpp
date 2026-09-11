#include <cstdio>
#include <cstring>
#include <string_utils.h>

void format_message_truncated(char *buffer, int max_size, const char *format, va_list args) {
  int actual_len = vsnprintf(buffer, max_size, format, args);
  if (actual_len >= max_size) {
    strcpy(buffer + max_size - 4, "...");
  }
}

bool playlist_has_image(const char *path, const char *names) {
  const char *s = names;
  int iLen;

  path++; // skip the leading '/'
  while (*s) {
    iLen = strcspn(s, "|");
    if (strncmp(path, s, iLen) == 0 && path[iLen] == '-') // the name is followed by the timestamp
      return true;
    s += iLen;
    if (*s == '|') s++;
  }
  return false;
} /* playlist_has_image() */

String escape_modem_param(const String &param) {
  String escaped = param;
    // Backslash first, so it doesn't re-escape the backslashes added below.
  escaped.replace("\\", "\\\\");
  escaped.replace("\"", "\\\"");
  escaped.replace(",", "\\,");
  return escaped;
}