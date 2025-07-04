#include <special_function.h>
#include <trmnl_log.h>

struct SpecialFunctionMap
{
  const char *name;
  SPECIAL_FUNCTION value;
};

static const SpecialFunctionMap specialFunctionMap[] = {
    {"none", SF_NONE},
    {"identify", SF_IDENTIFY},
    {"sleep", SF_SLEEP},
    {"add_wifi", SF_ADD_WIFI},
    {"restart_playlist", SF_RESTART_PLAYLIST},
    {"rewind", SF_REWIND},
    {"send_to_me", SF_SEND_TO_ME},
    {"guest_mode", SF_GUEST_MODE},
};

SPECIAL_FUNCTION parseSpecialFunction(String &special_function_str)
{
  for (const auto &entry : specialFunctionMap)
  {
    if (special_function_str.equals(entry.name))
    {
      Log_info("New special function - %s", entry.name);
      return entry.value;
    }
  }
  Log_info("No new special function");
  return SF_NONE;
}

bool parseSpecialFunctionToStr(char *buffer, size_t buffer_size, SPECIAL_FUNCTION special_function)
{
  for (const SpecialFunctionMap &entry : specialFunctionMap)
  {
    if (special_function == entry.value)
    {
      strncpy(buffer, entry.name, buffer_size);
      return true;
    }
  }
  return false;
}
