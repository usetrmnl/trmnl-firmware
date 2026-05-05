#include <ed25519.h>

extern "C" {
#include "tweetnacl.h"
}

bool ed25519_generate_keypair(uint8_t public_key[ED25519_PUBLIC_KEY_SIZE],
                               uint8_t private_key[ED25519_PRIVATE_KEY_SIZE])
{
  return crypto_sign_keypair(public_key, private_key) == 0;
}

bool ed25519_sign(uint8_t signature[ED25519_SIGNATURE_SIZE],
                  const uint8_t *msg, size_t len,
                  const uint8_t private_key[ED25519_PRIVATE_KEY_SIZE])
{
  uint64_t siglen = 0;
  return crypto_sign_detached(signature, &siglen, msg, len, private_key) == 0;
}

bool ed25519_verify(const uint8_t signature[ED25519_SIGNATURE_SIZE],
                    const uint8_t *msg, size_t len,
                    const uint8_t public_key[ED25519_PUBLIC_KEY_SIZE])
{
  return crypto_sign_verify_detached(signature, msg, len, public_key) == 0;
}
