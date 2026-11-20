#pragma once

// Gets an API key and friendly ID from the server when the device has none saved yet, showing the
// setup or error screen. May put the device to sleep.
void ensureDeviceSetUp(void);
