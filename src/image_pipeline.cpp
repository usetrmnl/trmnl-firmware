#include <image_pipeline.h>
#include <ArduinoLog.h>

#include <bl.h>
#include <config.h>
#include <display.h>
#include <filesystem.h>
#include <trmnl_log.h>
#include "displayed_image.h"

namespace ImagePipeline {

uint32_t downloadStream(WiFiClient *stream, int content_size, uint8_t *buffer)
{
  int iteration_counter = 0;
  int counter2 = content_size;
  unsigned long download_start = millis();
  unsigned long last_data_time = millis();
  int counter = 0;

  while (counter < content_size && millis() - download_start < 30000)
  {
    if (stream->available())
    {
      Log.info("%s [%d]: Downloading... Available bytes: %d\r\n", __FILE__, __LINE__, stream->available());
      int bytes_to_read = min(stream->available(), counter2 - counter);
      counter += stream->readBytes(buffer + counter, bytes_to_read);
      iteration_counter++;
      last_data_time = millis();
    }
    else if (!stream->connected() || millis() - last_data_time > 5000)
    {
      break;
    }
    delay(10);
  }

  Log_info("Download end: %d/%d bytes in %d ms (%d iterations)", counter, content_size,
           (int)(millis() - download_start), iteration_counter);
  return counter;
}

void writeImageToFile(const char *name, uint8_t *in_buffer, size_t size)
{
  size_t res = filesystem_write_to_file(name, in_buffer, size);
  if (res != size)
  {
    Log_error_submit("File writing ERROR. Result - %d", res);
  }
  else
  {
    Log.info("%s [%d]: file %s writing success - %d bytes\r\n", __FILE__, __LINE__, name, res);
  }
}

void updatePlaylistOrder(Preferences &preferences, const char *new_path, const char *prev_path)
{
  String order = preferences.getString(PREFERENCES_PLAYLIST_ORDER_KEY, "");
  String newStr = String(new_path);
  String prefix = newStr.substring(0, 14); // same-plugin identity (matches purge logic)
  String prevStr = String(prev_path);

  if (order.isEmpty())
  {
    preferences.putString(PREFERENCES_PLAYLIST_ORDER_KEY, newStr);
    return;
  }

  bool found = false;
  String result = "";
  int start = 0;
  while (start <= (int)order.length())
  {
    int sep = order.indexOf('|', start);
    String entry = (sep < 0) ? order.substring(start) : order.substring(start, sep);
    if (!entry.isEmpty())
    {
      if (!found && entry.startsWith(prefix))
      {
        result += (result.isEmpty() ? "" : "|") + newStr;
        found = true;
      }
      else if (entry == prevStr || filesystem_file_exists(entry.c_str()))
      {
        result += (result.isEmpty() ? "" : "|") + entry;
      }
    }
    if (sep < 0)
      break;
    start = sep + 1;
  }
  if (found)
  {
    preferences.putString(PREFERENCES_PLAYLIST_ORDER_KEY, result);
    return;
  }

  String result2 = "";
  bool inserted = false;
  start = 0;
  while (start <= (int)result.length())
  {
    int sep = result.indexOf('|', start);
    String entry = (sep < 0) ? result.substring(start) : result.substring(start, sep);
    if (!entry.isEmpty())
    {
      result2 += (result2.isEmpty() ? "" : "|") + entry;
      if (!inserted && entry == prevStr)
      {
        result2 += "|" + newStr;
        inserted = true;
      }
    }
    if (sep < 0)
      break;
    start = sep + 1;
  }
  if (!inserted)
    result2 += (result2.isEmpty() ? "" : "|") + newStr;
  preferences.putString(PREFERENCES_PLAYLIST_ORDER_KEY, result2);
}

void persistImagePaths(Preferences &preferences, const char *newPath)
{
  String curPath = preferences.getString(PREFERENCES_CURRENT_PATH_KEY, "");
  String lastPath = preferences.getString(PREFERENCES_LAST_PATH_KEY, "");
  if (!curPath.isEmpty() && (curPath != String(newPath) || lastPath.isEmpty()))
    preferences.putString(PREFERENCES_LAST_PATH_KEY, curPath);
  preferences.putString(PREFERENCES_CURRENT_PATH_KEY, String(newPath));
#ifdef BOARD_TRMNL_X
  updatePlaylistOrder(preferences, newPath, curPath.c_str());
#endif
  preferences.putString(PREFERENCES_BROWSE_PATH_KEY, String(newPath));
}

void loadPreviousImageIntoEpd()
{
#if defined(BOARD_X_CLASS) && !defined(BOARD_SEEED_RETERMINAL_E1003)
  if (!DisplayedImage::exists())
    return;

  char *path = DisplayedImage::get();
  if (!path || path[0] == '\0')
    return;

  int file_size = 0;
  uint8_t *prev = display_read_file(path, &file_size);
  if (!prev || file_size <= 0)
  {
    Log_info("loadPreviousImageIntoEpd: could not read %s", path ? path : "(null)");
    return;
  }

  Log.info("%s [%d]: Decoding previous image (%s) into the EPD 'old' buffer\r\n", __FILE__, __LINE__,
           path);
  png_to_epd(prev, file_size, true);
  free(prev);
#else
  (void)0;
#endif
}

bool cachedFileExists(String &newName)
{
  char szTemp[36];
  fixFileName(newName.c_str(), szTemp);
  return filesystem_file_exists(szTemp);
}

} // namespace ImagePipeline
