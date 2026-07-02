#include "at_escape.h"

String escapeAtParam(const String& param) {
  String escaped = param;
  // Backslash first, so it doesn't re-escape the backslashes added below.
  escaped.replace("\\", "\\\\");
  escaped.replace("\"", "\\\"");
  escaped.replace(",", "\\,");
  return escaped;
}
