#include "cached_images.h"

#include <Preferences.h>
#include <bl.h>
#include <config.h>
#include <cstdlib>
#include <cstring>
#include <display.h>
#include <filesystem.h>
#include <globals.h>
#include <trmnl_log.h>

#include "displayed_image.h"

void show_last_image(void) {
  int file_size = 0;
  String curPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
  if (!curPath.isEmpty()) {
    uint8_t *buf = display_read_file(curPath.c_str(), &file_size);
    if (buf && file_size > 0) {
      display_show_image(buf, file_size, true);
      free(buf);
      DisplayedImage::remember(curPath.c_str());
    }
  }
}

void show_last_image_and_sleep(void) {
  show_last_image();
  goToSleep();
}
