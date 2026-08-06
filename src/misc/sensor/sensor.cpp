#include <misc/sensor.h>

#ifdef INCLUDE_ENV_SENSOR
static EnvironmentSensor sensorInstance(SENSOR_SDA, SENSOR_SCL);
#else
static BaseSensor sensorInstance;
#endif

BaseSensor &sensor() { return sensorInstance; }
