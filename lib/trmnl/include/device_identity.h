#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <persistence_interface.h>
#include <ed25519.h>

/**
 * Device identity containing Ed25519 keypair
 */
struct DeviceIdentity {
  uint8_t publicKey[ED25519_PUBLIC_KEY_SIZE];   // 32 bytes
  uint8_t privateKey[ED25519_PRIVATE_KEY_SIZE]; // 64 bytes
  bool initialized;
};

/**
 * Initialize device identity by loading from storage or generating new keypair
 *
 * If keys exist in NVS, loads them into the identity struct.
 * If keys don't exist, generates a new Ed25519 keypair and stores it.
 *
 * @param storage Persistence interface for NVS access
 * @param identity Output struct to populate with keys
 * @return true if identity was initialized (either loaded or generated)
 */
bool initDeviceIdentity(Persistence &storage, DeviceIdentity &identity);

/**
 * Clear device identity from storage
 *
 * Called during factory reset. Keys will be regenerated on next boot.
 *
 * @param storage Persistence interface for NVS access
 */
void clearDeviceIdentity(Persistence &storage);

/**
 * Convert public key to hex string for use in HTTP headers
 *
 * @param identity Device identity containing the public key
 * @return 64-character hex string of the 32-byte public key
 */
String publicKeyToHex(const DeviceIdentity &identity);
