#include <stored_logs.h>
#include <trmnl_log.h>
#include <persistence_interface.h>

StoredLogs::StoredLogs(uint8_t max_notes, const char *log_key, const char *head_key, Persistence& persistence)
    : max_notes(max_notes), log_key(log_key), head_key(head_key), persistence(persistence), overwrite_count(0) {}

LogStoreResult StoredLogs::store_log(const String& log_buffer)
{
  // Try to find an empty slot first
  for (uint8_t i = 0; i < max_notes; i++)
  {
    String key = log_key + String(i);
    if (!persistence.recordExists(key.c_str()))
    {
      size_t res = persistence.writeString(key.c_str(), log_buffer.c_str());
      if (res == log_buffer.length())
      {
        return {LogStoreResult::SUCCESS, "Log stored in new slot", i};
      }
      else
      {
        return {LogStoreResult::FAILURE, "Failed to write to new slot", i};
      }
    }
  }

  // All slots full, use circular buffer behavior
  uint8_t head = persistence.readUChar(head_key, 0);
  
  String key = log_key + String(head);
  size_t res = persistence.writeString(key.c_str(), log_buffer.c_str());
  if (res != log_buffer.length())
  {
    return {LogStoreResult::FAILURE, "Failed to overwrite slot", head};
  }

  uint8_t next_head = (head + 1) % max_notes;
  if (!persistence.writeUChar(head_key, next_head))
  {
    return {LogStoreResult::FAILURE, "Log written but head update failed", head};
  }

  overwrite_count++;

  return {LogStoreResult::SUCCESS, "Log overwrote existing slot", head};
}

String StoredLogs::gather_stored_logs()
{
  String log;
  for (uint8_t i = 0; i < max_notes; i++)
  {
    String key = log_key + String(i);
    if (persistence.recordExists(key.c_str()))
    {
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
  int count = 0;
  for (uint8_t i = 0; i < max_notes; i++)
  {
    String key = log_key + String(i);
    if (persistence.recordExists(key.c_str()))
    {
      bool note_del = persistence.remove(key.c_str());
      if (note_del)
        count++;
    }
  }
  overwrite_count = 0;
  Log_info("Cleared %d stored logs", count);
}

uint32_t StoredLogs::get_overwrite_count()
{
  return overwrite_count;
}