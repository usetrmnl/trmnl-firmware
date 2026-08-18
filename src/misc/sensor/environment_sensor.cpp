#include <misc/sensor.h>

#include <Arduino.h>
#include <esp_sleep.h>
#include <time.h>
#include <trmnl_log.h>
#include <config.h>
extern TRMNL_DEVICE *pDevice;

// bb_temperature device names, indexed by SensorReadings::sensorType
static const char *szDevices[] = {"None",    "AHT20",  "BMP180",  "BME280", "BMP388", "SHT3X",
                                  "HDC1080", "HTS221", "MCP9808", "BME68x", "SHTC3", "SHT40"};
static const char *szMakers[] = {"None", "ASAIR",   "Bosch",     "Bosch", "Bosch",    "Sensirion",
                                 "TI",   "STMicro", "MicroChip", "Bosch", "Sensirion", "Sensirion"};

void EnvironmentSensor::init() {
  bool co2Found = false;
  int sensorType = -1;

  if (pDevice->sensor_sda == 0xff) return; // no I2C bus defined for this device

  // check if there is a SCD41 or supported temperature sensor attached
  if (_scd41.init(pDevice->sensor_sda, pDevice->sensor_scl) == SCD41_SUCCESS) {
    co2Found = true;
    Log_info("SCD41 sensor found!");
    _scd41.wakeup();
    // The SCD41 needs to be re-initialized after big Vcc variations from the last wakeup
    // put it in a 'confused' state. If we don't re-initialize it, it won't generate more samples
    _scd41.sendCMD(SCD41_CMD_REINIT);
    vTaskDelay(3);          // allow time to reinitialize
    _scd41.triggerSample(); // trigger a 'one-shot' sample that takes about 5 seconds to complete
  }
  if (_bbt.init(pDevice->sensor_sda, pDevice->sensor_scl) == BBT_SUCCESS) {
    sensorType = _bbt.type();
    Log_info("supported sensor found! (%d)", sensorType);
    _bbt.start(); // start the sensor
  }
  if (!co2Found && sensorType < 0) {
    Log_info("No sensor found on I2C bus %d/%d", pDevice->sensor_sda, pDevice->sensor_scl);
    return;
  }

  // wait for the sensor(s) to generate a sample
  Log_info("Light sleep for 5 seconds to allow sensor to generate a sample");
  esp_sleep_enable_timer_wakeup(5000 * 1000L); // the SCD4x needs 5 seconds to get a sample
  esp_light_sleep_start();                     // use light sleep to save power

  if (co2Found) {
    if (_scd41.getSample() == SCD41_SUCCESS) {
      _readings.sampledAt = (int)time(nullptr);
      _readings.co2 = _scd41.co2();
      _readings.scdTemperature = _scd41.temperature();
      _readings.scdHumidity = _scd41.humidity();
      Log_info("Got SCD41 sample: CO2 = %dppm", _readings.co2);
    } else {
      Log_info("SCD41 sample failed");
    }
    _scd41.shutdown(); // conserve power since we completed getting a sample ready for the next TRMNL wakeup
  }
  if (sensorType >= 0) {
    BBT_SAMPLE bbts;
    if (_bbt.getSample(&bbts) == BBT_SUCCESS) {
      _readings.sampledAt = (int)time(nullptr);
      _readings.temperature = bbts.temperature;
      _readings.humidity = bbts.humidity;
      _readings.pressure = bbts.pressure;
      _readings.sensorType = sensorType;
      Log_info("Got bb_temperature sample: Temp = %d.%dC", _readings.temperature / 10, _readings.temperature % 10);
    } else {
      Log_info("bb_temperature sample failed");
    }
    _bbt.stop(); // turn off the sensor to conserve power
  }
}

bool EnvironmentSensor::buildSensorsHeader(char **szTemp) {
  if (_readings.co2 == 0 && _readings.sensorType < 0) return false;

  char szPart[128];
  *szTemp = (char *)malloc(1024); // make sure we have enough space, but don't use the stack because it's small
  (*szTemp)[0] = 0;
  if (_readings.co2 != 0) { // valid data from SCD4x for CO2, Temperature and Humidity
    // create the multi-value string to pass as a HTTP header
    sprintf(*szTemp,
            "make=Sensirion;model=SCD41;kind=carbon_dioxide;value=%d;unit=parts_per_million;created_at=%d,make="
            "Sensirion;model=SCD41;kind=temperature;value=%f;unit=celsius;created_at=%d,make=Sensirion;model=SCD41;"
            "kind=humidity;value=%d;unit=percent;created_at=%d",
            _readings.co2, _readings.sampledAt, (float)_readings.scdTemperature / 10.0f, _readings.sampledAt,
            _readings.scdHumidity, _readings.sampledAt);
    Log_info("Adding SCD41 data to api request: CO2: %d, Temp: %d.%dC, Humidity: %d%%", _readings.co2,
             _readings.scdTemperature / 10, _readings.scdTemperature % 10, _readings.scdHumidity);
  }
  if (_readings.sensorType >= 0 &&
      _readings.temperature != 0) { // we have data from another bb_temperature supported sensor too; add it
    if (_readings.co2 != 0) {
      strcat(*szTemp, ","); // separate from CO2 data
    } else {
      (*szTemp)[0] = 0;
    }
    Log_info("Adding bb_temperature data to api request: pressure: %d, Temp: %d.%dC, Humidity: %d%%",
             _readings.pressure, _readings.temperature / 10, _readings.temperature % 10, _readings.humidity);
    sprintf(szPart, "make=%s;model=%s;kind=temperature;value=%f;unit=celsius;created_at=%d",
            szMakers[_readings.sensorType], szDevices[_readings.sensorType], (float)_readings.temperature / 10.0f,
            _readings.sampledAt);
    strcat(*szTemp, szPart);
    if (_readings.humidity > 0) { // add humidity
      sprintf(szPart, ",make=%s;model=%s;kind=humidity;value=%d;unit=percent;created_at=%d",
              szMakers[_readings.sensorType], szDevices[_readings.sensorType], _readings.humidity, _readings.sampledAt);
      strcat(*szTemp, szPart);
    }
    if (_readings.pressure > 0) {
      sprintf(szPart, ",make=%s;model=%s;kind=pressure;value=%d;unit=hectopascal;created_at=%d",
              szMakers[_readings.sensorType], szDevices[_readings.sensorType], _readings.pressure, _readings.sampledAt);
      strcat(*szTemp, szPart);
    }
  }
  return true;
}
