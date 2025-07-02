#include <Arduino.h>

String serializeApiLogRequest(String log_buffer)
{
  return "{\"logs\":[" + log_buffer + "]}";
}