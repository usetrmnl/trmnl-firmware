#include <stored_logs.h>
#include <trmnl_log.h>
#include <persistence_interface.h>

StoredLogs::StoredLogs(uint8_t max_notes, const char *log_key, const char *head_key, Persistence& persistence)
    : max_notes(max_notes), log_key(log_key), head_key(head_key), persistence(persistence) {}

void StoredLogs::store_log(const String& log_buffer)
{
  bool result = false;

  for (uint8_t i = 0; i < max_notes; i++)
  {
    String key = log_key + String(i);
    if (persistence.recordExists(key.c_str()))
    {
      Log_info("key %s exists", key.c_str());
      result = false;
    }
    else
    {
      Log_info("key %s not exists", key.c_str());
      size_t res = persistence.writeString(key.c_str(), log_buffer.c_str());
      Log_info("Initial size %d. Received size - %d", log_buffer.length(), res);
      if (res == log_buffer.length())
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
    if (persistence.recordExists(head_key))
    {
      Log_info("head exists");
      head = persistence.readUChar(head_key, 0);
    }
    else
    {
      Log_info("head NOT exists");
    }

    String key = log_key + String(head);
    size_t res = persistence.writeString(key.c_str(), log_buffer.c_str());
    if (res == log_buffer.length())
    {
      Log_info("log note written success");
    }
    else
    {
      Log_info("log note writing failed");
    }

    head += 1;
    if (head == max_notes)
    {
      head = 0;
    }

    uint8_t result_write = persistence.writeUChar(head_key, head);
    if (result_write)
      Log_info("head written success");
    else
      Log_info("head note writing failed");
  }
}

String StoredLogs::gather_stored_logs()
{
  String log;
  for (uint8_t i = 0; i < max_notes; i++)
  {
    String key = log_key + String(i);
    if (persistence.recordExists(key.c_str()))
    {
      Log_info("log note exists");
      String note = persistence.readString(key.c_str(), "");
      if (note.length() > 0)
      {
        log += note;

        String next_key = log_key + String(i + 1);
        if (persistence.recordExists(next_key.c_str()))
        {
          log += ",";
        }
      }
    }
  }
  return log;
}

void StoredLogs::clear_stored_logs()
{
  for (uint8_t i = 0; i < max_notes; i++)
  {
    String key = log_key + String(i);
    if (persistence.recordExists(key.c_str()))
    {
      Log_info("log note exists");
      bool note_del = persistence.remove(key.c_str());
      if (note_del)
        Log_info("log note deleted");
      else
        Log_info("log note not deleted");
    }
  }
}