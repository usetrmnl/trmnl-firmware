#include <stored_logs.h>
#include <trmnl_log.h>
#include <persistence_interface.h>

StoredLogs::StoredLogs(uint8_t old_count, uint8_t new_count, const char *log_key, const char *head_key, Persistence &persistence)
    : old_count(old_count), new_count(new_count), log_key(log_key), head_key(head_key), persistence(persistence), overwrite_count(0) {}

LogStoreResult StoredLogs::store_log(const String &log_buffer)
{
  uint8_t total_slots = old_count + new_count;

  // Find first empty slot across entire range
  for (uint8_t i = 0; i < total_slots; i++)
  {
    String key = log_key + String(i);
    if (!persistence.recordExists(key.c_str()))
    {
      size_t res = persistence.writeString(key.c_str(), log_buffer.c_str());
      if (res == log_buffer.length())
      {
        return {LogStoreResult::SUCCESS, "Log stored in empty slot", i};
      }
      else
      {
        return {LogStoreResult::FAILURE, "Failed to write to empty slot", i};
      }
    }
  }

  if (new_count == 0)
  {
    return {LogStoreResult::SUCCESS, "Log discarded - slots full", 0};
  }

  // Use circular buffer in newest section
  uint8_t head = persistence.readUChar(head_key, old_count);
  uint8_t slot = old_count + ((head - old_count) % new_count);

  String key = log_key + String(slot);
  size_t res = persistence.writeString(key.c_str(), log_buffer.c_str());
  if (res != log_buffer.length())
  {
    return {LogStoreResult::FAILURE, "Failed to overwrite slot", slot};
  }

  uint8_t next_head = old_count + ((head - old_count + 1) % new_count);
  if (!persistence.writeUChar(head_key, next_head))
  {
    return {LogStoreResult::FAILURE, "Log written but head update failed", slot};
  }

  overwrite_count++;
  return {LogStoreResult::SUCCESS, "Log overwrote slot", slot};
}

String StoredLogs::gather_stored_logs()
{
  String log;
  uint8_t total_slots = old_count + new_count;

  if (total_slots == 0)
    return log;

  // Helper lambda to append log entry
  auto append_log = [&](uint8_t slot)
  {
    String key = log_key + String(slot);
    if (persistence.recordExists(key.c_str()))
    {
      String note = persistence.readString(key.c_str(), "");
      if (note.length() > 0)
      {
        if (log.length() > 0)
          log += ",";
        log += note;
      }
    }
  };

  // Add oldest slots in simple order
  for (uint8_t i = 0; i < old_count; i++)
  {
    append_log(i);
  }

  // Add newest slots
  if (new_count > 0)
  {
    uint8_t head = persistence.readUChar(head_key, old_count);
    for (uint8_t i = 0; i < new_count; i++)
    {
      uint8_t slot = old_count + ((head - old_count + i) % new_count);
      append_log(slot);
    }
  }

  return log;
}

void StoredLogs::clear_stored_logs()
{
  int count = 0;
  uint8_t max_notes = old_count + new_count;
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