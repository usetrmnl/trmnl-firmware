#pragma once

// NVS keys for Ed25519 authentication
// These are duplicated here for lib/trmnl access
// Main definitions are in include/config.h
#define PREFERENCES_ED25519_PUBLIC_KEY "ed_pub"
#define PREFERENCES_ED25519_PRIVATE_KEY "ed_priv"
#define PREFERENCES_AUTH_MODE "auth_mode"
#define PREFERENCES_AUTH_MODE_DEFAULT "api_key"
#define PREFERENCES_AUTH_MODE_ED25519 "ed25519"
