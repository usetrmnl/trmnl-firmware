#ifndef CACHED_IMAGES_H
#define CACHED_IMAGES_H

// Browsing the images cached on the filesystem: the TRMNL X touch bar's left and right taps,
// and the menu's Previous Screen and Close. Playlist order NVS lives in filesystem.h
// (update_playlist_order).

// Shows the cached image `offset` places from the one being browsed (the playlist order wraps
// around), then sleeps. Returns without sleeping when there is no cached image to show.
void show_cached_image_by_offset(int offset);

// Redraws the current image (after something covered it) and sleeps.
void show_last_image_and_sleep(void);

#endif // CACHED_IMAGES_H
