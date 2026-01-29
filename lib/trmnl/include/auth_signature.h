#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <device_identity.h>
#include <ed25519.h>

/**
 * Authentication signature for Ed25519 challenge-response
 */
struct AuthSignature {
  uint8_t signature[ED25519_SIGNATURE_SIZE];  // 64 bytes
  uint64_t timestamp_ms;
  bool valid;
};

/**
 * Compute authentication signature for API request
 *
 * Signs the message: timestamp_ms (8 bytes big-endian) || public_key (32 bytes)
 *
 * @param identity Device identity with keypair
 * @param timestamp_ms Server timestamp in milliseconds
 * @return AuthSignature with signature and validity flag
 */
AuthSignature computeAuthSignature(const DeviceIdentity &identity, uint64_t timestamp_ms);

/**
 * Convert signature to hex string for use in HTTP headers
 *
 * @param sig Auth signature to convert
 * @return 128-character hex string of the 64-byte signature
 */
String signatureToHex(const AuthSignature &sig);

/**
 * Convert timestamp to string for use in HTTP headers
 *
 * @param timestamp_ms Timestamp in milliseconds
 * @return String representation of the timestamp
 */
String timestampToString(uint64_t timestamp_ms);
