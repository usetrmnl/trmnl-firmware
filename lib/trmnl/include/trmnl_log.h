
#ifdef PIO_UNIT_TESTING
#include <mock_log.h>
#else
#include <ArduinoLog.h>
#endif

#define Log_verbose(format, ...) Log.verbose("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define Log_info(format, ...) Log.info("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define Log_error(format, ...) Log.error("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define Log_fatal(format, ...) Log.fatal("%s [%d]: " format "\r\n", __FILE__, __LINE__, ##__VA_ARGS__)
