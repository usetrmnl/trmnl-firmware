#include <Arduino.h>
#include <bmp.h>
#include <filesystem.h>
#include <inttypes.h>
#include <trmnl_log.h>

#if defined(BOARD_X_CLASS)
#include <LittleFS.h>
#define FS LittleFS
#else
#include <SPIFFS.h>
#define FS SPIFFS
#endif

/**
 * @brief Function to init the filesystem
 * @param none
 * @return bool result
 */
bool filesystem_init(void) {
  if (!FS.begin(true)) {
    Log_fatal("Failed to mount filesystem");
    ESP.restart();
    return false;
  } else {
    Log_info("Filesystem mounted");
    return true;
  }
}

/**
 * @brief Function to de-init the filesystem
 * @param none
 * @return none
 */
void filesystem_deinit(void) { FS.end(); }

/**
 * @brief Function to read a file into a newly allocated buffer
 * @param name filename
 * @param out_buffer pointer to pointer of the output buffer
 * @return filesize in bytes or 0 if failed
 */
size_t filesystem_read_and_allocate(const char *name, uint8_t **out_buffer) {
  uint8_t *buffer;
  size_t size;

  if (FS.exists(name)) {
    Log_info("file %s exists", name);
    File file = FS.open(name, FILE_READ);
    if (file) {
      size = file.size();
      if (size == 0) {
        Log_error("File %s is empty", name);
        file.close();
        return 0;
      }
#ifdef CONFIG_SPIRAM
      buffer = (uint8_t *)ps_malloc(size);
#else
      buffer = (uint8_t *)malloc(size);
#endif
      if (!buffer) {
        Log_error("Failed to allocate %d bytes", (int)size);
        file.close();
        return 0;
      }
      file.readBytes((char *)buffer, size);
      file.close();
      if (bmpIsTruncated(buffer, size)) {
        Log_error("File %s lost bytes on the way to flash", name);
        free(buffer);
        return 0;
      }
      *out_buffer = buffer;
      return size;
    } else {
      Log_error("File %s open error", name);
      return 0;
    }
  } else {
    Log_info("file %s doesn\'t exist", name);
  }
  return 0;
} /* filesystem_read_and_allocate() */

/**
 * @brief Function to read data from file
 * @param name filename
 * @param out_buffer pointer to output buffer
 * @return result - true if success; false - if failed
 */
bool filesystem_read_from_file(const char *name, uint8_t *out_buffer, size_t size) {
  if (FS.exists(name)) {
    Log_info("file %s exists", name);
    File file = FS.open(name, FILE_READ);
    if (file) {
      file.readBytes((char *)out_buffer, size);
      return true;
    } else {
      Log_error("File %s open error", name);
      return false;
    }
  } else {
    Log_info("file %s doesn\'t exists", name);
    return false;
  }
}

/**
 * @brief Function to delete old versions of plugin images (by comparing the timestamp)
 *        It also deletes files that are older than 24h to keep SPIFFS from filling up
 * @param name filename
 * @return nothing
 */
void filesystem_purge_old_file(const char *name) {
  uint32_t timestamp;
  time_t now;
  File rootDir;
  char *s, szTemp[36];
  bool bDel;

  time(&now); // get the current epoch time
  rootDir = FS.open("/");
  while (File file = rootDir.openNextFile()) {
    Log_info("Checking file \"%s\" for deletion", file.name());

    if (file.isDirectory()) {
      Log_info("Skipping directory \"%s\"", file.name());
      file.close();
      continue;
    }

    s = (char *)file.name();

    timestamp = filesystem_extract_timestamp(s);
    bDel = false;

    snprintf(szTemp, sizeof(szTemp), "/%s", file.name()); // needed on this file operation

    Log_info("Comparing name %s with %s, timestamp %" PRIu32 ", current time %" PRIu32, name, file.name(), timestamp,
             (uint32_t)now);
    if (strncmp(name, szTemp, 14) == 0) { // older version of the same file
      Log_info("Deleting older version of plugin image %s - %s", name, file.name());
      bDel = true;
    } else if ((uint32_t)now > timestamp + 60 * 60 * 24) { // More than 24h old, or no timestamp
      Log_info("Deleting image older than 24h - %s", file.name());
      bDel = true;
    }
    if (bDel) { // to avoid double code
      Log_info("Deleting file %s", szTemp);
      file.close();
      FS.remove(szTemp);
    }
  }
  rootDir.close();

} /* filesystem_purge_old_file() */

/**
 * @brief Function to delete the oldest cached image
 *        Images with no timestamp are the setup logo and the last displayed image, the only
 *        picture a device has when it cannot reach the API, so they are left alone.
 * @param none
 * @return true if an image was deleted; false if there was nothing we are willing to delete
 */
static bool filesystem_evict_oldest_image(void) {
  char oldest[36] = "";
  uint32_t oldestTimestamp = 0;

  File rootDir = FS.open("/");
  while (File file = rootDir.openNextFile()) {
    uint32_t timestamp = filesystem_extract_timestamp(file.name());
    if (!file.isDirectory() && timestamp != 0 && (oldest[0] == '\0' || timestamp < oldestTimestamp)) {
      oldestTimestamp = timestamp;
      snprintf(oldest, sizeof(oldest), "/%s", file.name());
    }
    file.close();
  }
  rootDir.close();

  if (oldest[0] == '\0') {
    Log_info("Nothing left to evict");
    return false;
  }

  Log_info("Evicting %s", oldest);
  return FS.remove(oldest);
} /* filesystem_evict_oldest_image() */

/**
 * @brief Function to write a buffer to a file in chunks, without any recovery
 * @param name filename
 * @param in_buffer pointer to input buffer
 * @param size size of the input buffer
 * @return size of written bytes, less than size if the filesystem ran out of room
 */
static size_t filesystem_write_chunks(const char *name, uint8_t *in_buffer, size_t size) {
  File file = FS.open(name, FILE_WRITE, true);
  if (!file) {
    Log_error("File open ERROR");
    return 0;
  }

  size_t bytesWritten = 0;
  while (bytesWritten < size) {
    size_t chunkSize = _min(4096, size - bytesWritten);
    if (file.write(in_buffer + bytesWritten, chunkSize) != chunkSize) {
      break;
    }
    bytesWritten += chunkSize;
  }

  file.close();
  return bytesWritten;
} /* filesystem_write_chunks() */

/**
 * @brief Function to write data to file
 * @param name filename
 * @param in_buffer pointer to input buffer
 * @param size size of the input buffer
 * @return size of written bytes
 */
size_t filesystem_write_to_file(const char *name, uint8_t *in_buffer, size_t size) {
  uint32_t FS_freeBytes = (FS.totalBytes() - FS.usedBytes());
  Log_info("FS free space - %" PRIu32 ", total -%zu", FS_freeBytes, FS.totalBytes());
  if (FS.exists(name)) {
    Log_info("file %s exists. Deleting...", name);
    if (FS.remove(name))
      Log_info("file %s deleted", name);
    else
      Log_info("file %s deleting failed", name);
  } else {
    Log_info("file %s doesn't exist.", name);
  }
//    delay(100);
  // SPIFFS hands out well under what it reports free, by an amount that moves with fragmentation
  // (measured on an OG: a 12K write refused with 22K free), so make room by trying rather than by
  // predicting. Formatting stays as the last resort, once there is nothing left to evict.
  for (;;) {
    size_t bytesWritten = filesystem_write_chunks(name, in_buffer, size);
    if (bytesWritten == size) {
      Log_info("file %s writing success - %d bytes", name, bytesWritten);
      return bytesWritten;
    }

    FS.remove(name); // the partial file is dead weight, and its space is worth reclaiming

    if (!filesystem_evict_oldest_image()) {
      Log_info("Erasing FS...");
      if (FS.format()) {
        Log_info("FS erased successfully.");
      } else {
        Log_error("Error erasing FS.");
      }
      return bytesWritten;
    }
  }
}

/**
 * @brief Function to check if file exists
 * @param name filename
 * @return result - true if exists; false - if not exists
 */
bool filesystem_file_exists(const char *name) {
  if (FS.exists(name)) {
    Log_info("file %s exists.", name);
    return true;
  } else {
    Log_info("file %s does not exist.", name);
    return false;
  }
}

/**
 * @brief Function to delete the file
 * @param name filename
 * @return result - true if success; false - if failed
 */
bool filesystem_file_delete(const char *name) {
  if (FS.exists(name)) {
    if (FS.remove(name)) {
      Log_info("file %s deleted", name);
      return true;
    } else {
      Log_error("file %s deleting failed", name);
      return false;
    }
  } else {
    Log_info("file %s doesn't exist", name);
    return true;
  }
}

/**
 * @brief Function to rename the file
 * @param old_name old filename
 * @param new_name new filename
 * @return result - true if success; false - if failed
 */
bool filesystem_file_rename(const char *old_name, const char *new_name) {
  if (FS.exists(old_name)) {
    Log_info("file %s exists.", old_name);
    bool res = FS.rename(old_name, new_name);
    if (res) {
      Log_info("file %s renamed to %s.", old_name, new_name);
      return true;
    } else
      Log_error("file %s wasn't renamed.", old_name);
    return false;
  } else {
    Log_info("file %s not exists.", old_name);
    return false;
  }
}

void list_files() {
  Log_info("Filesystem Usage: %d/%d", FS.usedBytes(), FS.totalBytes());
  File rootDir = FS.open("/");

  while (File file = rootDir.openNextFile()) {
    Log_info("  %d  %s", file.size(), file.name());
  }
  rootDir.close();
}

uint32_t filesystem_extract_timestamp(const char *filename) {
  if (strlen(filename) < 10) {
    return 0;
  }

  const char *szTimestamp = &filename[strlen(filename) - 10];
  bool is_numeric = true;

  for (int i = 0; i < 10 && is_numeric; i++) {
    is_numeric = (szTimestamp[i] >= '0' && szTimestamp[i] <= '9');
  }

  if (is_numeric) {
    return (uint32_t)atoi(szTimestamp);
  } else {
    return 0;
  }
}

void writeImageToFile(const char *name, uint8_t *in_buffer, size_t size) {
  size_t res = filesystem_write_to_file(name, in_buffer, size);
  if (res != size) {
    Log_error_submit("File writing ERROR. Result - %d", res);
  } else {
    Log_info("file %s writing success - %d bytes", name, res);
  }
}

void filesystem_fix_filename(const char *src, char *dest) {
  int iLen;

  // SPIFFS only allows 32 bytes for the name, so if it's too long, fix it
  dest[0] = '/'; // SPIFFS requires files to start with the root dir
  iLen = strlen(src);
  if (iLen > 31) {
    memcpy(&dest[1], src, 7);          // first 7 chars are "plugin-" or "mashup-"
    strcpy(&dest[8],
           &src[iLen - 17]); // get the prefix name and unique id plus timestamp (e.g. mashup-066cc3-1771674964)
  } else {
    strcpy(&dest[1], src); // use it as-is; strncpy left a 31 character name unterminated
  }
} /* filesystem_fix_filename() */

bool filesystem_fixed_file_exists(String &newName) {
  char szTemp[36];

  filesystem_fix_filename(
    newName.c_str(),
    szTemp); // shorten the name (if needed) to fit the SPIFFS file length limit of 31 chars + 0 terminator
  return filesystem_file_exists(szTemp);
} /* filesystem_fixed_file_exists() */
