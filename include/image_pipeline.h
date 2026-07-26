#pragma once

/**
 * Image pipeline: filesystem paths, stream download, and cache bookkeeping.
 *
 * Extracted from bl.cpp so download/display orchestration can live separately
 * from wake/UI/business-logic flow. Prefer adding image I/O here rather than
 * growing bl.cpp.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <WiFiClient.h>
#include <cstddef>
#include <cstdint>

namespace ImagePipeline {

/** Read up to content_size bytes from stream into buffer (with timeouts). */
uint32_t downloadStream(WiFiClient *stream, int content_size, uint8_t *buffer);

/** Write image bytes to filesystem; logs success/failure. */
void writeImageToFile(const char *name, uint8_t *in_buffer, size_t size);

/**
 * Rotate NVS path keys after a successful image become "current":
 * last ← previous current (if needed), current ← newPath, browse ← newPath.
 * On X, also updates playlist order via updatePlaylistOrder.
 */
void persistImagePaths(Preferences &preferences, const char *newPath);

/** Update X playlist order string in NVS (no-op on non-X builds if unused). */
void updatePlaylistOrder(Preferences &preferences, const char *new_path, const char *prev_path);

/**
 * Decode the previously displayed image into FastEPD "old" buffer when present.
 * Used before painting a new frame for partial-update balance on some X boards.
 */
void loadPreviousImageIntoEpd();

/** True if shortened SPIFFS name for newName already exists on the filesystem. */
bool cachedFileExists(String &newName);

} // namespace ImagePipeline
