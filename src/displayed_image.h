#pragma once

// Tracks which image is physically on the display, surviving deep sleep,
// so the next wakeup can skip repainting when the server re-serves it.
namespace DisplayedImage
{
    void remember(const char *filename);
    void clear();
    bool exists();
    bool matches(const char *filename);
    char *get();
}
