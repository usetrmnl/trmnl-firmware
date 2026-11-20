#pragma once

// Tracks which image is physically on the display, surviving deep sleep,
// so the next wakeup can skip repainting when the server re-serves it.
namespace DisplayedImage {
  void remember(const char *filename);
  void cover(); // the loading screen is over it: exists() and matches() are false until the next remember()
  bool exists();
  bool matches(const char *filename);
  char *get(); // the last image drawn, also while covered
} // namespace DisplayedImage
