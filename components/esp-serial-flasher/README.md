# ESP Serial Flasher

[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/espressif/esp-serial-flasher/master.svg)](https://results.pre-commit.ci/latest/github/espressif/esp-serial-flasher/master)
[![Component Registry](https://components.espressif.com/components/espressif/esp-serial-flasher/badge.svg)](https://components.espressif.com/components/espressif/esp-serial-flasher)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ESP Serial Flasher is a portable C library for programming and interacting with Espressif SoCs from other host microcontrollers.

## Overview

This library enables you to program Espressif SoCs from various host platforms using different communication interfaces. It provides a unified API that abstracts the underlying communication protocol, making it easy to integrate ESP device programming into your projects. In this context, the host (flashing/programming device running this library) controls the target (the ESP-series SoC being programmed). It serves a similar purpose to [esptool](https://github.com/espressif/esptool), but is designed for embedded hosts without a PC or Python runtime or on less powerful single board computers.

- **Connection and identification**: Connect to targets, autodetect chip family, read MAC address, retrieve security info.
- **Flash operations**: Write, read, erase, detect flash size, and optionally verify data via MD5.
- **RAM download and execution**: Load binaries to RAM and run them.
- **Registers and control**: Read/write registers, change transmission rate, reset the target.

### Supported Communication Interfaces

- **UART** - Universal asynchronous communication
- **USB CDC ACM** - USB virtual serial port
- **SPI** - Serial Peripheral Interface (RAM download only)
- **SDIO** - Secure Digital Input/Output (experimental)

> [!NOTE]
> SDIO interface is experimental and currently supports only ESP32-C5 and ESP32-C6 as target. The implementation uses a custom stub that will be made available as part of the migration to [esp-flasher-stub](https://github.com/espressif/esp-flasher-stub).

### Supported Host Platforms (device running this library and performing flashing)

- **STM32** microcontrollers
- **Raspberry Pi** SBC
- **ESP32 series** microcontrollers
- **Zephyr OS** compatible devices
- **Raspberry Pi Pico** (RP2040)

### Supported Target Devices (ESP device being flashed)

|  Target  | UART | SPI | SDIO | USB CDC ACM |
| :------: | :--: | :-: | :--: | :---------: |
| ESP8266  |  ✅  | ❌  |  ❌  |     ❌      |
|  ESP32   |  ✅  | ❌  |  🚧  |     ❌      |
| ESP32-S2 |  ✅  | ❌  |  ❌  |     ❌      |
| ESP32-S3 |  ✅  | ✅  |  ❌  |     ✅      |
| ESP32-C2 |  ✅  | ✅  |  ❌  |     ❌      |
| ESP32-C3 |  ✅  | ✅  |  ❌  |     ✅      |
| ESP32-H2 |  ✅  | ✅  |  ❌  |     ✅      |
| ESP32-C6 |  ✅  | ❌  |  ✅  |     ✅      |
| ESP32-C5 |  ✅  | ❌  |  ✅  |     ✅      |
| ESP32-P4 |  ✅  | 🚧  |  ❌  |     ✅      |

**Legend**: ✅ Supported | ❌ Not supported | 🚧 Under development

> [!NOTE]
> **Stub support**: ESP8266, ESP32-C5, and ESP32-P4 stub support is under development

### Public API

- Public headers: [include/esp_loader.h](include/esp_loader.h) and [include/esp_loader_io.h](include/esp_loader_io.h) define the stable public API of this library.
- Examples and helpers: [examples/common/](examples/common/) contains helper utilities used by the examples; not part of the library API, but can be used as a reference.

## Getting Started

### Prerequisites

To use ESP Serial Flasher, you need:

- **[CMake](https://cmake.org/) 3.5 or later** - Build system
- **[Git](https://git-scm.com/)** - For cloning the repository with submodules

#### Platform Setup

Different host platforms require specific setup procedures:

- **[ESP32 series](docs/platform-setup.md#esp-idf-support)**: Works with [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) v4.3 or later
- **[STM32](docs/platform-setup.md#stm32-setup)**: Requires [STM32 HAL libraries](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html) and [ARM toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)
- **[Zephyr](docs/platform-setup.md#zephyr-setup)**: Integrates as [Zephyr](https://zephyrproject.org/) module with specific Kconfig options
- **[Raspberry Pi Pico](docs/platform-setup.md#raspberry-pi-pico-setup)**: Uses [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- **[Raspberry Pi](docs/platform-setup.md#raspberry-pi-setup)**: Requires [pigpio library](https://abyz.me.uk/rpi/pigpio/)
- **[Custom platforms](docs/supporting-new-platform.md#implementation-steps)**: ESP Serial Flasher can be used as a git submodule with your own custom platform implementation

For detailed setup instructions, see [Platform Setup Guide](docs/platform-setup.md).

For implementing custom platform support, see [Supporting New Platforms Guide](docs/supporting-new-platform.md), particularly the sections on [using ESP Serial Flasher as an external library](docs/supporting-new-platform.md#option-b-using-esp-serial-flasher-as-external-library) and [implementation steps](docs/supporting-new-platform.md#implementation-steps).

### Basic Usage

```c
#include "esp_loader.h"

esp_loader_error_t err;

// Initialize and connect
esp_loader_connect_args_t config = ESP_LOADER_CONNECT_DEFAULT();
err = esp_loader_connect(&config);
if (err != ESP_LOADER_SUCCESS) {
    printf("Connection failed: %s\n", esp_loader_error_string(err));
    return err;
}

// Flash binary (example: 64KB at 0x10000)
const uint32_t addr = 0x10000;
const size_t size = 65536;
const size_t block_size = 4096;
// Variable holding your binary image. Typical sources:
// - Read from storage (SD card, filesystem, flash)
// - Received over a link (UART/SPI/USB/Wi‑Fi) into a RAM buffer
// - Compiled-in C array generated from a .bin
const uint8_t *data = /* pointer to your firmware image buffer */;

err = esp_loader_flash_start(addr, size, block_size);
if (err != ESP_LOADER_SUCCESS) return err;

// Write data in chunks
size_t offset = 0;
while (offset < size) {
   size_t chunk = MIN(block_size, size - offset);
   err = esp_loader_flash_write(data + offset, chunk);
   if (err != ESP_LOADER_SUCCESS) return err;
   offset += chunk;
}

esp_loader_reset_target();
return err
```

### Examples

For complete implementation examples, see the [examples](examples/) directory:

- [ESP32 Example](examples/esp32_example/) - ESP32 family as host
- [STM32 Example](examples/stm32_example/) - STM32 as host
- [Raspberry Pi Example](examples/raspberry_example/) - Raspberry Pi as host
- [Zephyr Example](examples/zephyr_example/) - Zephyr OS integration
- [Raspberry Pi Pico Example](examples/pi_pico_example/) - RP2040 as host
- [ESF Demo](https://github.com/Dzarda7/esf-demo) - End-to-end demo flashing ESP targets from an embedded host (M5Stack Dial) over USB CDC ACM; includes SD card image selection and on-device progress UI

### Educational Resources

- [esptool documentation](https://docs.espressif.com/projects/esptool/en/latest/esp32/) - Contains most of the information on how the communication with the chip works, what is and is not possible etc.
- [YouTube Tutorial](https://www.youtube.com/watch?v=hYqkOew8y8w) published 9th September 2024 - Comprehensive guide covering library usage, internals, and custom port implementation

## Configuration

ESP Serial Flasher provides several configuration options to customize its behavior. These options are set as **CMake cache variables**.

### Basic Configuration

The most common configuration options:

```bash
# Enable SPI interface instead of UART
cmake -DSERIAL_FLASHER_INTERFACE_SPI=1 ..

# Disable MD5 verification
cmake -DMD5_ENABLED=0 ..

# Set custom retry count
cmake -DSERIAL_FLASHER_WRITE_BLOCK_RETRIES=5 ..
```

### Interface Selection

Choose one interface (UART is default):

- `SERIAL_FLASHER_INTERFACE_UART` - UART communication (default)
- `SERIAL_FLASHER_INTERFACE_SPI` - SPI communication
- `SERIAL_FLASHER_INTERFACE_USB` - USB CDC ACM
- `SERIAL_FLASHER_INTERFACE_SDIO` - SDIO (experimental)

For complete configuration reference, see [Configuration Documentation](docs/configuration.md).

## Hardware Connections

Each communication interface has specific hardware connection requirements and pin configurations. For complete wiring diagrams, pin assignments, and interface-specific setup instructions, see [Hardware Connections Guide](docs/hardware-connections.md).

## Contributing

We welcome contributions! Before starting work on new features or significant changes, please [open an issue](https://github.com/espressif/esp-serial-flasher/issues) to discuss your proposal.

For detailed contribution guidelines, see [CONTRIBUTING.md](CONTRIBUTING.md).

### Adding New Platform Support

If you want to add support for a new host platform, see [Supporting New Host Platforms Guide](docs/supporting-new-platforms.md).

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Known Limitations

The following limitations are currently known:

- Binary image size must be known before flashing
- ESP8266 targets require `MD5_ENABLED=0` due to ROM bootloader limitations
- SPI interface only supports RAM download operations
- SDIO interface is experimental with limited platform support
- Only one target can be flashed at a time (library holds state in static variables)
- Communication interface must be selected at compile time (no runtime switching)

For additional limitations and current issues, see the [GitHub Issues](https://github.com/espressif/esp-serial-flasher/issues) page.
