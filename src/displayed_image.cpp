#include "displayed_image.h"
#include "esp_attr.h"
#include <string.h>

namespace DisplayedImage
{
    // SPIFFS path of the image currently on the display
    RTC_DATA_ATTR static char szPrevFile[36] = {0};

    // filename must already be a 32-byte fixed SPIFFS path (see Storage::fix_file_name)
    void remember(const char *filename)
    {
        strncpy(szPrevFile, filename, sizeof(szPrevFile) - 1);
        szPrevFile[sizeof(szPrevFile) - 1] = '\0';
    }

    void clear()
    {
        memset(szPrevFile, 0, sizeof(szPrevFile));
    }

    bool exists()
    {
        return szPrevFile[0] != '\0';
    }

    bool matches(const char *filename)
    {
        return strcmp(szPrevFile, filename) == 0;
    }

    char *get()
    {
        return szPrevFile;
    }
}
