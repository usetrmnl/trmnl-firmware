#include <ota_schedule.h>

bool otaAttemptDue(uint32_t now, uint32_t lastOta)
{
  if (lastOta == 0) return true;
  if (now == 0) return false;
  return (now - lastOta) >= OTA_RETRY_INTERVAL_SECONDS;
}

uint32_t otaLastAttempt(Persistence &persistence)
{
  return persistence.readUint(OTA_LAST_ATTEMPT_KEY, 0);
}

void otaRecordAttempt(Persistence &persistence, uint32_t now)
{
  if (now != 0)
    persistence.writeUint(OTA_LAST_ATTEMPT_KEY, now);
}
