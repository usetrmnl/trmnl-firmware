#pragma once

#include <Arduino.h>
#include <api-client/setup.h>
#include <display.h>

// Full-screen logo images and messages, from the custom art in the top of flash when there is some.

// 0 = larger glyph for message screens
// 1 = loading screen (mostly blank, small glyph in lower right corner)
uint8_t *storedLogoOrDefault(int iType);

// Shows a full-screen message with the TRMNL logo.
void showMessageWithLogo(MSG message_type);

// Shows a full-screen message with the TRMNL logo and extra device details (friendly ID, firmware
// version, an extra message line).
void showMessageWithLogo(MSG message_type, String friendly_id, bool id, const char *fw_version, String message);

// Shows a full-screen message with the TRMNL logo and the message from the API setup response.
void showMessageWithLogo(MSG message_type, const ApiSetupResponse &apiResponse);

// Shows the loading logo on a wake the user caused, and makes the next screen redraw.
void showWakeLogo(void);
