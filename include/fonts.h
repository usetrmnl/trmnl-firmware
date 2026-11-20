#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>

// The bitmap fonts (include/fonts/), defined once in fonts.cpp. A font header defines its array
// local to each file that includes it, so include this instead.
extern const uint8_t nicoclean_8[];
extern const uint8_t Inter_18[];
extern const uint8_t Roboto_Black_24[];

#endif // FONTS_H
