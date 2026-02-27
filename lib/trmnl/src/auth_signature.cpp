#include <auth_signature.h>
#include <trmnl_log.h>
#include <hex_utils.h>
#include <string.h>

AuthSignature computeAuthSignature(const DeviceIdentity &identity, uint64_t timestamp_ms)
{
  AuthSignature result;
  result.timestamp_ms = timestamp_ms;
  result.valid = false;
  memset(result.signature, 0, sizeof(result.signature));

  if (!identity.initialized)
  {
    Log_error("Cannot compute signature: device identity not initialized");
    return result;
  }

  // Build message: timestamp_ms (8 bytes big-endian) || public_key (32 bytes)
  uint8_t message[8 + ED25519_PUBLIC_KEY_SIZE];

  // Write timestamp as big-endian
  message[0] = (timestamp_ms >> 56) & 0xFF;
  message[1] = (timestamp_ms >> 48) & 0xFF;
  message[2] = (timestamp_ms >> 40) & 0xFF;
  message[3] = (timestamp_ms >> 32) & 0xFF;
  message[4] = (timestamp_ms >> 24) & 0xFF;
  message[5] = (timestamp_ms >> 16) & 0xFF;
  message[6] = (timestamp_ms >> 8) & 0xFF;
  message[7] = timestamp_ms & 0xFF;

  // Append public key
  memcpy(message + 8, identity.publicKey, ED25519_PUBLIC_KEY_SIZE);

  // Sign the message
  if (!ed25519_sign(result.signature, message, sizeof(message), identity.privateKey))
  {
    Log_error("Ed25519 signing failed");
    return result;
  }

  // Self-verify: catch key corruption, bad entropy, or stack issues early
  if (!ed25519_verify(result.signature, message, sizeof(message), identity.publicKey))
  {
    Log_error("Ed25519 self-verify FAILED - keypair likely corrupted");
    Log_error("  pk: %s", bytesToHex(identity.publicKey, ED25519_PUBLIC_KEY_SIZE).c_str());
    Log_error("  sk[32..64]: %s", bytesToHex(identity.privateKey + 32, 32).c_str());
    Log_error("  sig: %s", bytesToHex(result.signature, ED25519_SIGNATURE_SIZE).c_str());
    Log_error("  msg: %s", bytesToHex(message, sizeof(message)).c_str());
    memset(result.signature, 0, sizeof(result.signature));
    return result;
  }

  result.valid = true;
  Log_info("Computed auth signature for timestamp %llu (self-verify OK)", timestamp_ms);
  return result;
}

String signatureToHex(const AuthSignature &sig)
{
  return bytesToHex(sig.signature, ED25519_SIGNATURE_SIZE);
}

String timestampToString(uint64_t timestamp_ms)
{
  // Using String() constructor for uint64_t
  char buf[21];  // Max uint64 is 20 digits + null
  snprintf(buf, sizeof(buf), "%llu", (unsigned long long)timestamp_ms);
  return String(buf);
}
