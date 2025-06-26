#pragma once

#include <Arduino.h>

enum SPECIAL_FUNCTION
{
  SF_NONE,
  SF_IDENTIFY,
  SF_SLEEP,
  SF_ADD_WIFI,
  SF_RESTART_PLAYLIST,
  SF_REWIND,
  SF_SEND_TO_ME,
  SF_GUEST_MODE,
};

SPECIAL_FUNCTION parseSpecialFunction(String &special_function_str);
bool parseSpecialFunctionToStr(char *buffer, size_t buffer_size, SPECIAL_FUNCTION special_function);
