#include "displayed_image.h"

#include <string.h>
#include <trmnl_log.h>

#include "esp_attr.h"

// SPIFFS path of the image currently on the display
RTC_DATA_ATTR static char szPrevFile[36] = {0}; // must be OUTSIDE of namespace

namespace DisplayedImage {

    // filename must already be a 32-byte fixed SPIFFS path (see Storage::fix_file_name)
  void remember(const char *filename) {
    strncpy(szPrevFile, filename, sizeof(szPrevFile) - 1);
    szPrevFile[sizeof(szPrevFile) - 1] = '\0';
    Log_info("%s [%d]: Remembering previous file: %s\n", __FILE__, __LINE__, szPrevFile);
  }

  void clear() { memset(szPrevFile, 0, sizeof(szPrevFile)); }

  bool exists() {
      Log_info("%s [%d]: Checking if szPrevFile[] exists: %s\n", __FILE__, __LINE__, szPrevFile);
      return szPrevFile[0] != '\0';
  }

  bool matches(const char *filename) { return strcmp(szPrevFile, filename) == 0; }

  char *get() { return szPrevFile; }
} // namespace DisplayedImage
