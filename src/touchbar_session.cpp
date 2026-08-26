#include <touchbar_session.h>

#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <config.h>
#include <display.h>
#include <filesystem.h>
#include <globals.h>
#include <trmnl_log.h>

#include "displayed_image.h"

void goToSleep(void);

void show_cached_image_by_offset(int offset) {
  String order = preferences.getString(PREFERENCES_PLAYLIST_ORDER_KEY, "");

  if (order.isEmpty()) {
    String path = (offset > 0) ? preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "")
                               : preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    if (path.isEmpty()) {
      Log_info("No cached image for gesture");
      return;
    }
    int file_size = 0;
    buffer = display_read_file(path.c_str(), &file_size);
    if (buffer && file_size > 0) {
      display_show_image(buffer, file_size, true);
      DisplayedImage::remember(path.c_str());
      goToSleep();
    }
    return;
  }

  char images[MAX_CACHED_IMAGES][36];
  int count = 0;
  int start = 0;
  while (start <= (int)order.length() && count < MAX_CACHED_IMAGES) {
    int sep = order.indexOf('|', start);
    String entry = (sep < 0) ? order.substring(start) : order.substring(start, sep);
    if (!entry.isEmpty() && filesystem_file_exists(entry.c_str())) {
      strncpy(images[count], entry.c_str(), 35);
      images[count][35] = '\0';
      count++;
    }
    if (sep < 0) break;
    start = sep + 1;
  }

  if (count == 0) {
    Log_info("No cached images available");
    return;
  }

  String browsePath = preferences.getString(PREFERENCES_BROWSE_PATH_KEY, "");
  if (browsePath.isEmpty()) {
    // Seed from last_path so first RIGHT shows curr_path (forward) and first LEFT shows older (backward).
    // Falls back to curr_path if last_path is absent (e.g. only one image cached).
    String lp = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
    browsePath = lp.isEmpty() ? preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "") : lp;
  }

  int cur_idx = count - 1;
  for (int i = 0; i < count; i++) {
    if (browsePath == String(images[i])) {
      cur_idx = i;
      break;
    }
  }

  int new_idx = (cur_idx + offset + count) % count;
  Log_info("Playlist browse: %d/%d -> %d (%s)", cur_idx, count, new_idx, images[new_idx]);

  int file_size = 0;
  buffer = display_read_file(images[new_idx], &file_size);
  if (!buffer || file_size == 0) {
    Log_info("Failed to read %s", images[new_idx]);
    return;
  }

  preferences.putString(PREFERENCES_BROWSE_PATH_KEY, String(images[new_idx]));
  display_show_image(buffer, file_size, true);
  DisplayedImage::remember(images[new_idx]);
  goToSleep();
}

#endif // BOARD_TRMNL_X
