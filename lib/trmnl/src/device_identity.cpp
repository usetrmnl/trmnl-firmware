#include <device_identity.h>
#include <trmnl_log.h>
#include <ed25519_config.h>
#include <hex_utils.h>

bool initDeviceIdentity(Persistence &storage, DeviceIdentity &identity)
{
  identity.initialized = false;

  // Check if keys exist in NVS
  if (storage.recordExists(PREFERENCES_ED25519_PUBLIC_KEY) &&
      storage.recordExists(PREFERENCES_ED25519_PRIVATE_KEY))
  {
    // Load existing keys
    size_t pubLen = storage.readBytes(PREFERENCES_ED25519_PUBLIC_KEY,
                                       identity.publicKey,
                                       ED25519_PUBLIC_KEY_SIZE);
    size_t privLen = storage.readBytes(PREFERENCES_ED25519_PRIVATE_KEY,
                                        identity.privateKey,
                                        ED25519_PRIVATE_KEY_SIZE);

    if (pubLen == ED25519_PUBLIC_KEY_SIZE && privLen == ED25519_PRIVATE_KEY_SIZE)
    {
      identity.initialized = true;
      Log_info("Loaded existing Ed25519 keypair");
      Log_info("Public key: %s", publicKeyToHex(identity).c_str());
      return true;
    }
    else
    {
      Log_error("Invalid Ed25519 keys in NVS (pub=%d, priv=%d), regenerating",
                pubLen, privLen);
    }
  }

  // Generate new keypair
  Log_info("Generating new Ed25519 keypair...");

  if (!ed25519_generate_keypair(identity.publicKey, identity.privateKey))
  {
    Log_error("Failed to generate Ed25519 keypair");
    return false;
  }

  // Store in NVS
  size_t pubWritten = storage.writeBytes(PREFERENCES_ED25519_PUBLIC_KEY,
                                          identity.publicKey,
                                          ED25519_PUBLIC_KEY_SIZE);
  size_t privWritten = storage.writeBytes(PREFERENCES_ED25519_PRIVATE_KEY,
                                           identity.privateKey,
                                           ED25519_PRIVATE_KEY_SIZE);

  if (pubWritten != ED25519_PUBLIC_KEY_SIZE || privWritten != ED25519_PRIVATE_KEY_SIZE)
  {
    Log_error("Failed to store Ed25519 keys (pub=%d, priv=%d)", pubWritten, privWritten);
    return false;
  }

  identity.initialized = true;
  Log_info("Generated new Ed25519 keypair");
  Log_info("Public key: %s", publicKeyToHex(identity).c_str());
  return true;
}

void clearDeviceIdentity(Persistence &storage)
{
  storage.remove(PREFERENCES_ED25519_PUBLIC_KEY);
  storage.remove(PREFERENCES_ED25519_PRIVATE_KEY);
  storage.remove(PREFERENCES_AUTH_MODE);
  Log_info("Cleared device identity (keys will regenerate on next boot)");
}

String publicKeyToHex(const DeviceIdentity &identity)
{
  return bytesToHex(identity.publicKey, ED25519_PUBLIC_KEY_SIZE);
}
