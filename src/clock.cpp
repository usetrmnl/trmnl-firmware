//
// 1-minute clock for TRMNL X and OG
// Displays a non-flickering clock in a quadrant of a mashup
//
#include "DEV_Config.h"
#ifdef BOARD_TRMNL_X
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
void ShowClock(BB_RECT *pRect, bool bFirst, int iPanelType)
{
struct timeval tv;

    bbep.setPanelType(iPanelType);
    bbep.initIO(EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN, EPD_CS_PIN, EPD_MOSI_PIN, EPD_SCK_PIN, 8000000);
    bbep.allocBuffer();
    bbep.fillScreen(BBEP_WHITE);
    if (/*!bFirst && */ u32OldTime) { // not the first clock wakeup, draw the last time value in the old plane
        DrawTime(u32OldTime, pRect);
    }
    bbep.writePlane(PLANE_0_TO_1); // set the data into plane 1 from buffer position 0 (inverted)
    bbep.fillScreen(BBEP_WHITE);
    gettimeofday(&tv, NULL); // current time
    tv.tv_sec += i32TZOffset; // add timezone offset
    u32OldTime = tv.tv_sec; // save for next wakeup
    DrawTime(u32OldTime, pRect);
    bbep.writePlane(PLANE_0); // draw the current time into the 'new' plane
    bbep.refresh(REFRESH_PARTIAL);
    bbep.sleep(DEEP_SLEEP);
} /* DisplayTime() */
