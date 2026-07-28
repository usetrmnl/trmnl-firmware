
#include <WString.h>

struct LogApiInput {
  String api_key;
  const char *log_buffer;
};

bool submitLogToApi(LogApiInput &input, const char *api_url);