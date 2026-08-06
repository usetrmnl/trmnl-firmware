#include <ArduinoLog.h>
#include <battery.h>

float FakeBattery::readVoltage() {
  Log.warning("%s [%d]: FAKE_BATTERY_VOLTAGE is defined. Returning 4.2V.\r\n", __FILE__, __LINE__);
  return 4.2f;
}
