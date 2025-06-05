#include <Arduino.h>

String serializeApiLogRequest(String log_buffer)
{
  return "{\"log\":{\"logs_array\":[" + log_buffer + "]}}";
}