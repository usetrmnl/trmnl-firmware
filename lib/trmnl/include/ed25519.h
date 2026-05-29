#pragma once

#include <stdint.h>
#include <stddef.h>

#define ED25519_PUBLIC_KEY_SIZE 32
#define ED25519_PRIVATE_KEY_SIZE 64
#define ED25519_SIGNATURE_SIZE 64

/**
 * Generate a new Ed25519 keypair
 *
 * @param public_key Output buffer for 32-byte public key
 * @param private_key Output buffer for 64-byte private key
 * @return true on success, false on failure
 */
bool ed25519_generate_keypair(uint8_t public_key[ED25519_PUBLIC_KEY_SIZE],
                               uint8_t private_key[ED25519_PRIVATE_KEY_SIZE]);

/**
 * Sign a message using Ed25519
 *
 * @param signature Output buffer for 64-byte signature
 * @param msg Message to sign
 * @param len Length of message
 * @param private_key 64-byte private key
 * @return true on success, false on failure
 */
bool ed25519_sign(uint8_t signature[ED25519_SIGNATURE_SIZE],
                  const uint8_t *msg, size_t len,
                  const uint8_t private_key[ED25519_PRIVATE_KEY_SIZE]);

/**
 * Verify an Ed25519 signature
 *
 * @param signature 64-byte signature to verify
 * @param msg Message that was signed
 * @param len Length of message
 * @param public_key 32-byte public key
 * @return true if signature is valid, false otherwise
 */
bool ed25519_verify(const uint8_t signature[ED25519_SIGNATURE_SIZE],
                    const uint8_t *msg, size_t len,
                    const uint8_t public_key[ED25519_PUBLIC_KEY_SIZE]);
