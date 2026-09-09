#include <misc/sensor.h>

static EnvironmentSensor sensorInstance;

BaseSensor &sensor() { return sensorInstance; }
