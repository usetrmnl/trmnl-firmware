#pragma once

// Shared setup + assertion helpers for on-device integration tests.
//
// Lives under lib/ so each test suite can pick it up automatically via
// LDF when it #include's "test_helpers.h" (no per-suite duplication, no
// build_src_filter changes). Production builds never reference it, so
// it isn't linked into firmware images.

/// Bring WiFi up using TEST_WIFI_SSID / TEST_WIFI_PASSWORD from test_config.h.
/// No-op if already connected. Asserts (TEST_ASSERT_EQUAL_MESSAGE) if the
/// connection doesn't establish within 15 seconds — fails the calling test.
void connect_to_wifi();
