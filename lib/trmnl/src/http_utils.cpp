#include <http_utils.h>

String httpOriginOf(const String &url) {
  int schemeEnd = url.indexOf("://");
  if (schemeEnd == -1) return url;
  int pathStart = url.indexOf('/', schemeEnd + 3);
  if (pathStart == -1) return url;
  return url.substring(0, pathStart);
}

String resolveRedirectLocation(const String &location, const String &origin) {
  if (location.startsWith("http://") || location.startsWith("https://")) return location;
  return origin + location;
}

bool isSameOrigin(const String &url, const String &base) {
  if (base.length() == 0) return false;
  return url.startsWith(base);
}

const char *sniffImageContentType(const uint8_t *data, size_t len) {
  if (!data) return nullptr;
  if (len >= 2 && data[0] == 'B' && data[1] == 'M') return "image/bmp";
  if (len >= 8 && data[0] == 0x89 && data[1] == 'P' && data[2] == 'N' && data[3] == 'G' && data[4] == 0x0D &&
      data[5] == 0x0A && data[6] == 0x1A && data[7] == 0x0A)
    return "image/png";
  if (len >= 2 && data[0] == 0xFF && data[1] == 0xD8) return "image/jpeg";
  return nullptr;
}

bool shouldFastRetryCode(int code) {
  // <= 0: no HTTP response at all. Negative values are HTTPClient transport errors (connection
  // refused, reset, read timeout); 0 is "no client" on Wi-Fi or a modem ERROR; -1 is a modem timeout.
  if (code <= 0) return true;
  // 408: the server gave up waiting for the request; 5xx: server-side, usually transient.
  return code == 408 || (code >= 500 && code < 600);
}