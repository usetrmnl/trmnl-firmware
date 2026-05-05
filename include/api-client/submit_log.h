#pragma once

#include <WString.h>

// Forward declaration for auth support
struct DeviceIdentity;

struct LogApiInput
{
  String api_key;
  const char *log_buffer;
  // Ed25519 authentication (optional)
  String authMode;                    // "api_key" or "ed25519"
  const DeviceIdentity *identity;     // Device identity for Ed25519 auth (can be nullptr)
};

bool submitLogToApi(LogApiInput &input, const char *api_url);