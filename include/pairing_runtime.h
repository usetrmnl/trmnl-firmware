#pragma once
#include <pairing.h>
#if !defined(TRMNL_DISABLE_BLE_PAIRING) && (defined(BOARD_TRMNL) || defined(BOARD_TRMNL_4CLR) || defined(BOARD_TRMNL_X))
#define TRMNL_BLE_PAIRING 1
#endif
// These functions are main-task lifecycle hooks. Control callbacks only enqueue.
void pairingBegin();
// Manual-wake foreground beacon probe. True means authenticated management took over.
bool pairingWakeBegin();
// Load or create the persistent QR identity; does not start the radio.
void pairingPrepare();
bool pairingIdentityReady();
// Only returns a secret after its persistent storage has been verified.
String pairingPersistentKey();
struct DeviceSetupResult;
void pairingTick(DeviceSetupResult *setupResult = nullptr);
void pairingWifiResult(bool ok);
bool pairingReservePortal(const String &server);
void pairingCredentialsReady(const String &code);
void pairingSetupFinished();
void pairingBeforeOta();
void pairingBeforeSleep();
void pairingResume();
// Called only by the device-credential reset, never by timeout or handoff.
void pairingClear();
const PairingIdentity &pairingIdentity();
bool pairingUsesBle();
