# Configuration Reference

This document provides a comprehensive reference for all configuration options available in ESP Serial Flasher.

## Configuration Types

ESP Serial Flasher supports various configuration options that can be set as **CMake cache variables**. These options control the behavior and features of the library.

## Configuration Options

### Debug and Diagnostics

#### `SERIAL_FLASHER_DEBUG_TRACE`

- **Type**: CMake cache variable / Kconfig (`CONFIG_SERIAL_FLASHER_DEBUG_TRACE`)
- **Default**: Disabled
- **Description**: Enables debug tracing output (transfer data tracing).

### Interface Selection

> [!IMPORTANT]
> Only one interface can be enabled at a time. These options are mutually exclusive. There is plan to add runtime interface selection in the future.

#### `SERIAL_FLASHER_INTERFACE_UART` (Default)

- **Type**: CMake cache variable
- **Default**: Enabled (when no other interface is specified)
- **Description**: Use UART interface for communication with the target device.

#### `SERIAL_FLASHER_INTERFACE_SPI`

- **Type**: CMake cache variable
- **Default**: Disabled
- **Description**: Use SPI interface for communication with the target device.
- **Note**: Only supported for RAM download operations.

#### `SERIAL_FLASHER_INTERFACE_USB`

- **Type**: CMake cache variable
- **Default**: Disabled
- **Description**: Use USB CDC ACM interface for communication with the target device.

#### `SERIAL_FLASHER_INTERFACE_SDIO`

- **Type**: CMake cache variable
- **Default**: Disabled
- **Description**: Use SDIO interface for communication with the target device.
- **Note**: Experimental feature. Currently supported only with ESP32-P4 as host and ESP32-C6 as target.

### Flash Verification

#### `MD5_ENABLED`

- **Type**: CMake cache variable / Kconfig (see mapping below)
- **Default**:
  - Plain CMake: Disabled (only enabled if you pass `-DMD5_ENABLED=1`)
  - ESP-IDF/Zephyr (Kconfig): Enabled by default (`CONFIG_SERIAL_FLASHER_MD5_ENABLED=y`)
- **Description**: Enable MD5 checksum verification of flash content after writing.
- **Warning**: Must be disabled for ESP8266 targets as the ROM bootloader does not support MD5 verification.

### Retry and Timing Configuration

#### `SERIAL_FLASHER_WRITE_BLOCK_RETRIES`

- **Type**: CMake cache variable
- **Default**: 3
- **Description**: Number of retry attempts for writing blocks to target flash or RAM.

#### `SERIAL_FLASHER_RESET_HOLD_TIME_MS`

- **Type**: CMake cache variable
- **Default**: 100 (milliseconds)
- **Description**: Duration for which the reset pin is asserted during a hard reset.

#### `SERIAL_FLASHER_BOOT_HOLD_TIME_MS`

- **Type**: CMake cache variable
- **Default**: 50 (milliseconds)
- **Description**: Duration for which the boot pin is asserted during a hard reset.

### GPIO Pin Inversion

#### `SERIAL_FLASHER_RESET_INVERT`

- **Type**: CMake cache variable
- **Default**: Disabled
- **Description**: Invert the output of the reset GPIO pin.
- **Use case**: Useful when hardware has an inverting connection between host and target reset pin.
- **Availability**: UART interface only.

#### `SERIAL_FLASHER_BOOT_INVERT`

- **Type**: CMake cache variable
- **Default**: Disabled
- **Description**: Invert the output of the boot GPIO pin.
- **Use case**: Useful when hardware has an inverting connection between host and target boot pin.
- **Availability**: UART interface only.

## Setting Configuration Options

Configuration options can be set when running CMake using the `-D` flag:

```bash
cmake -DMD5_ENABLED=1 -DSERIAL_FLASHER_WRITE_BLOCK_RETRIES=5 .. && cmake --build .
```

### Multiple Options Example

```bash
cmake \
  -DSERIAL_FLASHER_INTERFACE_SPI=1 \
  -DMD5_ENABLED=0 \
  -DSERIAL_FLASHER_WRITE_BLOCK_RETRIES=5 \
  -DSERIAL_FLASHER_RESET_HOLD_TIME_MS=200 \
  .. && cmake --build .
```

### ESP-IDF and Zephyr Configuration

When using ESP-IDF or Zephyr, configuration options can be set using **Kconfig** instead of CMake flags. Many options have corresponding Kconfig names with the `CONFIG_` prefix:

- `SERIAL_FLASHER_DEBUG_TRACE` → `CONFIG_SERIAL_FLASHER_DEBUG_TRACE`
- `MD5_ENABLED` → `CONFIG_SERIAL_FLASHER_MD5_ENABLED`

You can configure these options using:

**ESP-IDF:**

```bash
idf.py menuconfig
```

**Zephyr:**

```bash
west build -t menuconfig
```

Alternatively, you can set them directly in your `sdkconfig` (ESP-IDF) or `prj.conf` (Zephyr) files:

```ini
# ESP-IDF sdkconfig or Zephyr prj.conf
CONFIG_SERIAL_FLASHER_DEBUG_TRACE=y
CONFIG_SERIAL_FLASHER_MD5_ENABLED=n
```

## Platform-Specific Configuration

Some platforms may have additional configuration requirements. See the platform-specific setup guides:

- [STM32 Setup](platform-setup.md#stm32-setup)
- [Zephyr Setup](platform-setup.md#zephyr-setup)
- [Raspberry Pi Pico Setup](platform-setup.md#raspberry-pi-pico-setup)
