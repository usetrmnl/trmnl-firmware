#pragma once

#include <Arduino.h>
#include <functional>
#include <persistence_interface.h>
#include <display.h>

struct FirmwareUpdateResult
{
  bool updated = false;
  MSG failureMessage = NONE;
};

class FirmwareUpdateService
{
public:
  using GetTimeFn = std::function<uint32_t()>;

  FirmwareUpdateService(
      Persistence &persistence,
      GetTimeFn getTime,
      int32_t wifiConnectionRssiThreshold);

  FirmwareUpdateResult tryUpdate(bool update_firmware, const String &firmware_url);

private:
  bool validateFirmwareUpdatePossible(bool update_firmware, const String &firmware_url);
  bool performFirmwareUpdate();

  Persistence &_persistence;
  GetTimeFn _getTime;
  int32_t _wifiConnectionRssiThreshold;
  MSG _failureMessage = NONE;
  char _firmwareUrl[1024];
};
