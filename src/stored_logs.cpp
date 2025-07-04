#include "stored_logs.h"
#include "config.h"
#include "trmnl_log.h"

void store_log(const char *log_buffer, size_t size, Preferences &preferences)
{
  bool result = false;

  for (uint8_t i = 0; i < LOG_MAX_NOTES_NUMBER; i++)
  {
    String key = PREFERENCES_LOG_KEY + String(i);
    if (preferences.isKey(key.c_str()))
    {
      Log_info("key %s exists", key.c_str());
      result = false;
    }
    else
    {
      Log_info("key %s not exists", key.c_str());
      size_t res = preferences.putString(key.c_str(), log_buffer);
      Log_info("Initial size %d. Received size - %d", size, res);
      if (res == size)
      {
        Log_info("log note written success");
      }
      else
      {
        Log_info("log note writing failed");
      }
      result = true;
      break;
    }
  }
  if (!result)
  {
    uint8_t head = 0;
    if (preferences.isKey(PREFERENCES_LOG_BUFFER_HEAD_KEY))
    {
      Log_info("head exists");
      head = preferences.getUChar(PREFERENCES_LOG_BUFFER_HEAD_KEY, 0);
    }
    else
    {
      Log_info("head NOT exists");
    }

    String key = PREFERENCES_LOG_KEY + String(head);
    size_t res = preferences.putString(key.c_str(), log_buffer);
    if (res == size)
    {
      Log_info("log note written success");
    }
    else
    {
      Log_info("log note writing failed");
    }

    head += 1;
    if (head == LOG_MAX_NOTES_NUMBER)
    {
      head = 0;
    }

    uint8_t result_write = preferences.putUChar(PREFERENCES_LOG_BUFFER_HEAD_KEY, head);
    if (result_write)
      Log_info("head written success");
    else
      Log_info("head note writing failed");
  }
}

void gather_stored_logs(String &log, Preferences &preferences)
{

  for (uint8_t i = 0; i < LOG_MAX_NOTES_NUMBER; i++)
  {
    String key = PREFERENCES_LOG_KEY + String(i);
    if (preferences.isKey(key.c_str()))
    {
      Log_info("log note exists");
      String note = preferences.getString(key.c_str(), "");
      if (note.length() > 0)
      {
        log += note;

        String next_key = PREFERENCES_LOG_KEY + String(i + 1);
        if (preferences.isKey(next_key.c_str()))
        {
          log += ",";
        }
      }
    }
  }
}

void clear_stored_logs(Preferences &preferences)
{

  for (uint8_t i = 0; i < LOG_MAX_NOTES_NUMBER; i++)
  {
    String key = PREFERENCES_LOG_KEY + String(i);
    if (preferences.isKey(key.c_str()))
    {
      Log_info("log note exists");
      bool note_del = preferences.remove(key.c_str());
      if (note_del)
        Log_info("log note deleted");
      else
        Log_info("log note not deleted");
    }
  }
}