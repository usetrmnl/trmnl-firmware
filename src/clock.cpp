//
// 1-minute clock for TRMNL X and OG
// Displays a non-flickering clock in a quadrant of a mashup
//
#include "config.h"
#ifdef PARALLEL_EPD
#include <FastEPD.h>
extern FASTEPD bbep;
#else
#include <bb_epaper.h>
extern BBEPAPER bbep;
#endif
#include "Roboto_Black_75.h"
// DEBUG - these will soon be variables
// timezone offset in seconds from GMT - e.g. GMT-5 = (-5 * 3600)
#define i32TZOffset (-3600*7)
RTC_DATA_ATTR uint32_t u32OldTime = 0;

static void DrawTime(uint32_t u32Epoch, BB_RECT *pRect)
{
struct tm myTime;
const time_t t = (time_t)u32Epoch;
char szTemp[32];
BB_RECT rect;

    bbep.fillRect(pRect->x, pRect->y, pRect->w, pRect->h, BBEP_WHITE);
    gmtime_r(&t, &myTime); // convert epoch into broken out hour/min/etc
    bbep.setFont(Roboto_Black_75);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
    snprintf(szTemp, sizeof(szTemp), "%02d:%02d", myTime.tm_hour, myTime.tm_min);
    bbep.getStringBox(szTemp, &rect);
    bbep.setCursor(pRect->x + ((pRect->w - rect.w)/2), pRect->y + pRect->h - 76); // horizontal center
    bbep.print(szTemp);
} /* DrawTime() */
//
// Display the current time and date in an attractive, bold font
// The rectangle passed in is the quadrant of the display to show the time
// Each time the ESP32 wakes up, the main memory contents have been lost so we
// need to keep track of what was previously drawn on the EPD. We can use the previous time
// to know the old pixels and set up the old vs new memory to only touch the area
// we are drawing into.
//
void ShowClock(CLOCK_INFO *pInfo, bool bFirst)
{
time_t now;
uint32_t u32Epoch;

    time(&now);
    u32Epoch = (uint32_t)now + pInfo->tz;
//#ifdef __BB_EPAPER__
//    bbep.setPanelType(iPanelType);
//    bbep.initIO(EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN, EPD_CS_PIN, EPD_MOSI_PIN, EPD_SCK_PIN, 8000000);
//    bbep.allocBuffer();
//#else // FastEPD
//    bbep.initPanel(iPanelType);
//#endif
    DrawTime(u32Epoch, &pInfo->rect);
#ifdef PARALLEL_EPD
    bbep.partialUpdate(false);
#else
    bbep.writePlane(PLANE_0); // draw the current time into the 'new' plane
    bbep.refresh(REFRESH_PARTIAL);
    bbep.sleep(DEEP_SLEEP);
#endif
} /* DisplayTime() */
