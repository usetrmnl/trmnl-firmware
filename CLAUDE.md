# CLAUDE.md - TRMNL Firmware Development Guide

This document provides AI assistants with comprehensive context about the TRMNL firmware codebase structure, development workflows, and key conventions.

## Project Overview

**TRMNL Firmware** is embedded firmware for ESP32-based e-ink display devices created for [TRMNL](https://usetrmnl.com). The firmware manages WiFi connectivity, API communication, image rendering, power management, and over-the-air (OTA) updates for various hardware configurations.

### Key Features
- WiFi captive portal for device setup
- REST API communication with TRMNL backend
- PNG/BMP/JPEG image decoding and display
- Battery-optimized deep sleep modes
- OTA firmware updates
- Device state persistence
- Logging and diagnostics

### Hardware Support
- **TRMNL OG**: ESP32-C3 based (default target)
- **TRMNL X**: ESP32-S3 based with PSRAM
- **Seeed XIAO ESP32-C3/S3**: Development boards
- **Waveshare ESP32 Driver**: Alternative driver board
- **Other**: Various DIY kits and reTerminal E1001

## Repository Structure

```
trmnl-firmware-arduinoesp32/
├── src/                          # Main application code
│   ├── main.cpp                  # Entry point (setup/loop)
│   ├── bl.cpp/h                  # Business logic core
│   ├── display.cpp/h             # E-paper display control
│   ├── button.cpp/h              # Button/wake detection
│   ├── filesystem.cpp/h          # SPIFFS file operations
│   ├── pins.cpp/h                # Pin definitions per board
│   ├── preferences_persistence.cpp/h # NVS storage
│   ├── api-client/               # API communication layer
│   │   ├── setup.cpp/h           # Device registration
│   │   ├── display.cpp/h         # Image/config fetching
│   │   └── submit_log.cpp/h      # Log submission
│   └── *.h                       # Image assets (logos, QR codes)
│
├── include/                      # Public headers
│   ├── config.h                  # Version, constants, board defines
│   ├── types.h                   # Common type definitions
│   └── api-client/*.h            # API client headers
│
├── lib/                          # Custom libraries
│   ├── trmnl/                    # Core utilities library
│   │   ├── include/              # Library headers
│   │   │   ├── api_types.h       # API data structures
│   │   │   ├── api_response_parsing.h
│   │   │   ├── api_request_serialization.h
│   │   │   ├── bmp.h/png.h       # Image decoders
│   │   │   ├── http_client.h     # HTTP wrapper
│   │   │   ├── stored_logs.h     # Log buffer management
│   │   │   ├── string_utils.h
│   │   │   └── trmnl_log.h       # Logging macros
│   │   └── src/                  # Library implementations
│   │
│   ├── wificaptive/              # WiFi configuration portal
│   │   ├── src/
│   │   │   ├── WifiCaptive.cpp/h # Main captive portal logic
│   │   │   ├── WebServer.cpp/h   # Async web server
│   │   │   ├── connect.cpp/h     # WiFi connection helpers
│   │   │   └── WifiCaptivePage.h # HTML content
│   │   └── portal/               # Web assets
│   │
│   └── bb_epaper/                # E-paper display driver
│       ├── src/bb_epaper.h       # Display control API
│       └── Fonts/                # Font definitions
│
├── test/                         # Unit tests
│   ├── test_parse_api_display/   # API response parsing tests
│   ├── test_parse_api_setup/
│   ├── test_serialize_log/
│   ├── test_stored_logs/
│   ├── test_bmp/                 # Image format tests
│   ├── test_png_flip/
│   └── test_string_utils/
│
├── scripts/                      # Build scripts
│   └── git_version.py            # Injects git hash into firmware version
│
├── boards/                       # Custom board definitions
│   └── esp32s3_n16r8.json        # TRMNL X board config
│
├── .github/workflows/            # CI/CD
│   ├── build.yml                 # Build & test on PR/push
│   └── publish-firmware.yml      # Publish firmware artifacts
│
└── platformio.ini                # Build configuration
```

## Build System

### PlatformIO Configuration

The project uses **PlatformIO** with the **Arduino + ESP-IDF** framework. Key build environments:

| Environment | Target | Description |
|-------------|--------|-------------|
| `trmnl` (default) | ESP32-C3 | TRMNL OG hardware |
| `TRMNL_X` | ESP32-S3 | TRMNL X with PSRAM |
| `local` | ESP32-C3 | Development build with USB CDC |
| `seeed_xiao_esp32c3` | ESP32-C3 | Seeed XIAO board |
| `seeed_xiao_esp32s3` | ESP32-S3 | Seeed XIAO S3 board |
| `waveshare-esp32-driver` | ESP32 | Waveshare driver board |
| `native` | Linux/macOS | Unit tests (g++/gcc) |
| `native-windows` | Windows | Unit tests (MinGW) |

### Build Commands

```bash
# Build default (trmnl)
pio run

# Build specific environment
pio run -e TRMNL_X
pio run -e local

# Run tests
pio test -e native          # Linux/macOS
pio test -e native-windows  # Windows

# Static analysis
pio check --skip-packages --fail-on-defect high

# Upload to device (requires boot mode)
pio run --target upload

# Clean build
pio run --target clean
```

### Dependencies

**Third-party libraries** (managed by PlatformIO):
- `ArduinoJson@7.4.2` - JSON parsing/serialization
- `PNGdec@^1.1.6` - PNG image decoding
- `JPEGDEC@^1.8.4` - JPEG image decoding
- `ESPAsyncWebServer@3.7.7` - Async HTTP server
- `Arduino-Log` - Logging framework
- `FastEPD@1.3.0` - Fast e-paper display (TRMNL X only)
- `ArduinoFake@^0.4.0` - Mocking for native tests

**Custom libraries** (in `lib/`):
- `trmnl` - Core utilities, API parsing, image processing
- `wificaptive` - WiFi configuration portal
- `bb_epaper` - E-paper display driver

## Code Organization

### Architecture Overview

1. **Entry Point** (`src/main.cpp`):
   - `setup()`: Initializes QA tests if needed, marks OTA valid, calls `bl_init()`
   - `loop()`: Continuously calls `bl_process()`

2. **Business Logic** (`src/bl.cpp`):
   - Main state machine controlling device lifecycle
   - Handles wake-up reasons (timer, button press)
   - Manages WiFi connection, API calls, display updates, sleep
   - See algorithm flowchart in README.md

3. **API Client Layer** (`src/api-client/`):
   - **setup.cpp**: Device registration with MAC address → API key + Friendly ID
   - **display.cpp**: Fetches image URL, refresh rate, firmware updates
   - **submit_log.cpp**: Sends diagnostic logs to server

4. **Display Management** (`src/display.cpp`):
   - Initializes e-paper display based on board type
   - Renders images (BMP/PNG/JPEG) and status messages
   - Handles partial/full refresh

5. **Persistence** (`src/preferences_persistence.cpp`):
   - Wraps ESP32 NVS (Non-Volatile Storage)
   - Stores API credentials, WiFi settings, logs, device state

6. **WiFi Captive Portal** (`lib/wificaptive/`):
   - Starts AP mode if no WiFi credentials saved
   - Serves HTML configuration page
   - Allows device reset

### Key Files to Understand

| File | Purpose |
|------|---------|
| `include/config.h` | Firmware version, board defines, constants |
| `src/bl.cpp` | Core device logic and state machine |
| `lib/trmnl/include/api_types.h` | API request/response structures |
| `lib/trmnl/src/parse_response_api_display.cpp` | Parses `/api/display` JSON |
| `lib/wificaptive/src/WifiCaptive.cpp` | Captive portal implementation |
| `src/pins.cpp` | Pin mappings for each board variant |

## Development Workflows

### Making Code Changes

1. **Read Before Editing**: Always read files before modifying to understand context
2. **Board-Specific Code**: Check for `#ifdef BOARD_*` directives when editing
3. **Preserve Existing Style**: Match indentation, naming conventions
4. **Test Before Committing**: Run `pio test -e native` for unit tests
5. **Static Analysis**: Run `pio check` to catch issues

### Adding Support for New Hardware

1. Add board definition in `boards/` (if custom) or use PlatformIO built-in
2. Define new environment in `platformio.ini` extending `env:esp32_base`
3. Add board-specific `#define BOARD_*` in build flags
4. Update `include/config.h` with pin mappings for new board
5. Update `src/pins.cpp` with GPIO initialization
6. Test all features: WiFi, display, button, sleep, battery reading

### API Changes

1. **Request/Response Types**: Update `lib/trmnl/include/api_types.h`
2. **Parsing**: Modify `lib/trmnl/src/parse_response_api_*.cpp`
3. **Serialization**: Update `lib/trmnl/include/api_request_serialization.h`
4. **Tests**: Add tests in `test/test_parse_api_*/` or `test/test_serialize_*/`
5. **Documentation**: Update README.md API examples

### Adding New Features

1. **Check Existing Patterns**: Review similar features first
2. **Memory Constraints**: ESP32-C3 has limited RAM (~400KB), avoid large buffers
3. **Power Impact**: Minimize WiFi time, use light/deep sleep
4. **Error Handling**: Always handle HTTP errors, timeouts, parsing failures
5. **Logging**: Use `Log_info()`, `Log_error()` for diagnostics
6. **Persistence**: Store critical state in NVS (see `preferences_persistence.h`)

## Key Conventions

### Naming Conventions

- **Macros/Defines**: `UPPER_SNAKE_CASE` (e.g., `FW_VERSION_STRING`)
- **Functions**: `camelCase` (e.g., `readBatteryVoltage()`)
- **Variables**: `snake_case` (e.g., `wakeup_reason`, `api_key`)
- **Types/Structs**: `PascalCase` (e.g., `ApiDisplayResponse`)
- **Enums**: `UPPER_SNAKE_CASE` values (e.g., `MSG_WIFI_CONNECT_ERROR`)
- **Board Defines**: `BOARD_*` (e.g., `BOARD_TRMNL`, `BOARD_TRMNL_X`)

### Code Style

- **Indentation**: Spaces (typically 2 or 4, match existing file)
- **Braces**: Opening brace on same line for functions/conditionals
- **Includes**: System headers first, then local headers
- **Comments**: Use `//` for single-line, `/* */` for multi-line blocks
- **Header Guards**: `#ifndef FILENAME_H` / `#define FILENAME_H` / `#endif`

### Logging Practices

Use the Arduino-Log library macros:
```cpp
Log_info("Message: %s", str);
Log_error("Error code: %d", code);
Log_verbose("Debug details: %d, %s", val, name);
```

For device diagnostics, use the stored logs system:
```cpp
storedLogs.add(TRMNLLogLevel::INFO, LogCode::WIFI_CONNECTED, "Connected to AP");
```

### Memory Management

- **Stack vs Heap**: Prefer stack allocation for small buffers
- **Dynamic Allocation**: Use carefully, always check for `nullptr`
- **Image Buffers**: Allocated once in `bl.cpp`, reused across operations
- **PSRAM**: TRMNL X uses PSRAM for large images (`MAX_IMAGE_SIZE = 750KB`)
- **Free Memory**: Monitor with `ESP.getFreeHeap()` during development

### Error Handling Patterns

```cpp
// HTTP requests
https_request_err_e result = makeHttpRequest();
if (result != HTTP_REQUEST_SUCCESS) {
    Log_error("Request failed: %d", result);
    // Handle error (retry, show message, sleep)
}

// JSON parsing
if (!doc.containsKey("image_url")) {
    Log_error("Missing required field");
    return PARSE_ERROR;
}

// Resource allocation
uint8_t *buffer = (uint8_t*)malloc(size);
if (!buffer) {
    Log_error("Memory allocation failed");
    return ERR_OUT_OF_MEMORY;
}
```

## Testing

### Unit Tests

Located in `test/`, run with `pio test -e native` (or `native-windows` on Windows).

Tests use the **Unity** framework and **ArduinoFake** for mocking Arduino APIs.

**Test Structure**:
```cpp
#include <unity.h>

void test_feature() {
    int result = myFunction();
    TEST_ASSERT_EQUAL(expected, result);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_feature);
    return UNITY_END();
}
```

**Coverage Areas**:
- API response parsing (`test_parse_api_*`)
- API request serialization (`test_serialize_api_log`)
- Image processing (`test_bmp`, `test_png_flip`)
- Logging system (`test_serialize_log`, `test_stored_logs`)
- String utilities (`test_string_utils`)

### Hardware Testing

For device testing:
1. Build with `pio run -e local` (enables USB serial, disables light sleep)
2. Enter boot mode: Hold BOOT button, toggle power ON
3. Upload: `pio run -e local --target upload`
4. Monitor: `pio device monitor` (115200 baud)
5. Check serial output for logs

### QA Mode

On first boot after OTA update, device runs QA tests (`src/qa.cpp`):
- Display test pattern
- Button test
- Battery voltage reading
- Marks device as tested to skip on subsequent boots

## CI/CD Pipeline

### GitHub Actions Workflows

**`.github/workflows/build.yml`**:
- Triggers: Push to any branch, pull requests, manual dispatch
- Matrix: Ubuntu, macOS, Windows
- Steps:
  1. Checkout code
  2. Cache PlatformIO packages and build artifacts
  3. Build `trmnl` and `TRMNL_X` environments
  4. Run unit tests (`env:native` or `env:native-windows`)
  5. Run static analysis (clangtidy)
  6. Publish firmware (main branch or tags only)

**`.github/workflows/publish-firmware.yml`**:
- Called by build workflow on success
- Builds firmware with git hash (main) or version tag
- Uploads firmware binaries as artifacts

### Version Management

Firmware version defined in `include/config.h`:
```cpp
#define FW_MAJOR_VERSION 1
#define FW_MINOR_VERSION 7
#define FW_PATCH_VERSION 0
```

Git hash injected by `scripts/git_version.py` when `custom_include_git_hash = true`:
- Example: `1.7.0+abc1234`

## Common Tasks for AI Assistants

### Reading Codebase
- Start with `README.md` for overview and API documentation
- Read `include/config.h` to understand board variants and constants
- Trace execution from `main.cpp` → `bl_init()` → `bl_process()`
- Check relevant headers in `include/` and `lib/trmnl/include/`

### Debugging Issues
1. Check `src/bl.cpp` state machine logic
2. Review API parsing in `lib/trmnl/src/parse_response_api_*.cpp`
3. Verify board-specific defines in `include/config.h` and `platformio.ini`
4. Check error logs from serial monitor or submitted logs
5. Test in `env:local` with serial debugging enabled

### Modifying Features
1. Locate relevant code (e.g., display logic in `src/display.cpp`)
2. Read existing implementation thoroughly
3. Make minimal, focused changes
4. Update tests if behavior changes
5. Test with `pio run` and `pio test -e native`
6. Run static analysis with `pio check`

### Adding Configuration
1. Define new constant in `include/config.h`
2. Add NVS key in config section (e.g., `#define PREFERENCES_MY_KEY "my_key"`)
3. Use `preferencesPersistence.getString()` / `putString()` to access
4. Update API types if needed (`lib/trmnl/include/api_types.h`)
5. Parse from API response if server-provided

### Fixing Build Errors
- **Missing includes**: Add to source file, check `lib_deps` in `platformio.ini`
- **Undefined references**: Verify function declaration in header
- **Board-specific errors**: Check `#ifdef BOARD_*` conditions
- **Memory overflow**: Reduce buffer sizes, check `MAX_IMAGE_SIZE`
- **Link errors**: Ensure source file is in `src/` or `lib/*/src/`

## Board-Specific Notes

### TRMNL OG (`BOARD_TRMNL`)
- ESP32-C3 (single-core RISC-V, 4MB flash)
- 800x480 e-paper display (1-bit)
- Battery voltage on GPIO3
- Button wake on GPIO2
- Max compressed image: 90KB

### TRMNL X (`BOARD_TRMNL_X`)
- ESP32-S3 (dual-core Xtensa, 16MB flash, 8MB PSRAM)
- Larger display with color support (via FastEPD)
- USB CDC enabled by default
- Max compressed image: 750KB (uses PSRAM)
- Button wake on GPIO0

### Seeed XIAO ESP32-C3 (`BOARD_SEEED_XIAO_ESP32C3`)
- Boot button on GPIO9 (not RTC GPIO, can't wake from deep sleep)
- WiFi reset requires reset button + boot button sequence
- Fake battery voltage (no ADC)
- Use reset button for wake

### Seeed XIAO ESP32-S3 (`BOARD_SEEED_XIAO_ESP32S3`)
- Boot button on GPIO0 (works as wake button)
- USB mode configurable
- Fake battery voltage
- PSRAM available on some variants

## Security Considerations

- **WiFi Credentials**: Stored in NVS, cleared with long button press
- **API Key**: Stored in NVS, cleared with device reset
- **HTTPS**: All API calls use `WiFiClientSecure` with TLS
- **OTA Updates**: Firmware URL from trusted server only
- **Captive Portal**: No authentication (local AP only)
- **Logging**: Avoid logging sensitive data (API keys, WiFi passwords)

## Performance Guidelines

### Battery Optimization
- Minimize WiFi connection time (target: <6 seconds active)
- Use deep sleep between updates (100uA idle)
- Disable light sleep in development builds (`DO_NOT_LIGHT_SLEEP`)
- Read battery voltage BEFORE WiFi init (reduces noise)
- Implement sleep mode schedule (reduce updates during sleep hours)

### Memory Optimization
- Reuse buffers across API calls and image decoding
- Free allocated memory promptly
- Use `const` for string literals to save RAM
- Avoid `String` class for large data (use `char[]` or `std::string`)
- Monitor heap fragmentation with `Log_info("Free heap: %d", ESP.getFreeHeap())`

### Network Optimization
- Use persistent HTTP connections when possible
- Implement exponential backoff for retries
- Cache unchanged images (check filename before download)
- Compress logs before transmission
- Set appropriate timeouts (avoid long hangs)

## Troubleshooting

### Build Issues
- **"Tool not found"**: Run `pio pkg install`
- **Cache corruption**: Delete `.pio/` and rebuild
- **Wrong board**: Check `default_envs` in `platformio.ini`
- **Missing dependencies**: Run `pio pkg update`

### Flash/Upload Issues
- **Upload fails**: Ensure device in boot mode (hold BOOT, toggle power)
- **Port not found**: Check USB cable, drivers, permissions
- **Verification failed**: Try lower upload_speed (230400 or 115200)
- **Partition error**: Check `board_build.partitions` in platformio.ini

### Runtime Issues
- **No WiFi**: Check captive portal, verify credentials
- **No display update**: Check API response, image format, file size
- **Fast battery drain**: Verify sleep current (<200uA), check WiFi disconnect
- **OTA fails**: Check firmware_url, network stability, partition size
- **Crashes/reboots**: Monitor serial output, check stack overflow, memory leaks

## Resources

- **PlatformIO Docs**: https://docs.platformio.org
- **ESP32 Arduino Core**: https://github.com/espressif/arduino-esp32
- **ArduinoJson**: https://arduinojson.org
- **Unity Testing**: https://github.com/ThrowTheSwitch/Unity
- **TRMNL Web Flash**: https://usetrmnl.com/flash
- **TRMNL Website**: https://usetrmnl.com

## Getting Help

- **GitHub Issues**: Report bugs and feature requests
- **README.md**: API documentation, compilation guide
- **Code Comments**: Inline documentation for complex logic
- **Serial Logs**: Enable verbose logging with `LOG_LEVEL_VERBOSE`

---

**Last Updated**: 2025-12-07 (auto-generated for firmware v1.7.0)
