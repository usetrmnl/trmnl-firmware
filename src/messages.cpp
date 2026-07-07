#include "config.h"
#include "messages.h"

namespace Messages
{
    String firmware_version()
    {
        if (FW_COMMIT[0] != '\0')
        {
            return String(FW_VERSION_STRING) + " (" + String(FW_COMMIT) + ")";
        }
        else
        {
            return FW_VERSION_STRING;
        }
    }
}
