# Supporting New Host Platforms

This guide explains how to add support for new host microcontrollers or platforms to ESP Serial Flasher.

## Overview

ESP Serial Flasher uses a port layer abstraction to support different host platforms. To add support for a new host, implement the required port functions for your hardware and selected interface.

## Required Port Functions

All host platforms must implement these core functions. The exact signatures depend on whether you're using the SDIO interface or others (see `include/esp_loader_io.h` for complete definitions).

### Core Functions

#### `loader_port_read()`

- **Purpose**: Read bytes from the transport into a buffer.

**For UART/SPI/USB CDC ACM interfaces:**

```c
esp_loader_error_t loader_port_read(uint8_t *data, uint16_t size, uint32_t timeout);
```

**Parameters**:

- `data` destination buffer
- `size` requested bytes
- `timeout` maximum time to wait [ms]

**For SDIO interface:**

```c
esp_loader_error_t loader_port_read(uint32_t function, uint32_t addr, uint8_t *data,
                                   uint16_t size, uint32_t timeout);
```

**Parameters**:

- `function` SDIO function number

- `addr` address within function

- `data` destination buffer

- `size` requested bytes

- `timeout` maximum time to wait [ms]

- **Returns**: `ESP_LOADER_SUCCESS` on success, or other error codes.

> [!NOTE]
> Should block until at least 1 byte is read or `timeout` elapses. Can be called repeatedly by higher layers until `size` is satisfied. Must be thread-safe for concurrent calls within the same thread.

#### `loader_port_write()`

- **Purpose**: Write bytes to the transport.

**For UART/SPI/USB CDC ACM interfaces:**

```c
esp_loader_error_t loader_port_write(const uint8_t *data, uint16_t size, uint32_t timeout);
```

**Parameters**:

- `data` source buffer
- `size` number of bytes to send
- `timeout` timeout in milliseconds

**For SDIO interface:**

```c
esp_loader_error_t loader_port_write(uint32_t function, uint32_t addr, const uint8_t *data,
                                    uint16_t size, uint32_t timeout);
```

**Parameters**:

- `function` SDIO function number

- `addr` address within function

- `data` source buffer

- `size` number of bytes to send

- `timeout` timeout in milliseconds

- **Returns**: `ESP_LOADER_SUCCESS` on success, or error codes on failure.

> [!NOTE]
> Should block until all bytes are queued/transmitted or an error occurs.

#### `loader_port_enter_bootloader()`

```c
void loader_port_enter_bootloader(void);
```

- **Purpose**: Put the target device into ROM bootloader mode.

> [!NOTE]
> Typical UART sequence: assert BOOT low, hold RESET low for `SERIAL_FLASHER_RESET_HOLD_TIME_MS`, keep BOOT asserted for `SERIAL_FLASHER_BOOT_HOLD_TIME_MS`, then release. Respect `SERIAL_FLASHER_RESET_INVERT` and `SERIAL_FLASHER_BOOT_INVERT` settings when controlling GPIOs.

#### `loader_port_delay_ms()`

```c
void loader_port_delay_ms(uint32_t ms);
```

- **Purpose**: Millisecond delay helper.

**Parameters**:

- `ms` delay time [ms]

> [!NOTE]
> Used by the library for timing between steps.

#### `loader_port_start_timer()`

```c
void loader_port_start_timer(uint32_t ms);
```

- **Purpose**: Start a deadline/timeout timer.

**Parameters**:

- `ms` timeout duration [ms]

> [!NOTE]
> Subsequent calls to `loader_port_remaining_time()` should reflect remaining time relative to this start.

#### `loader_port_remaining_time()`

```c
uint32_t loader_port_remaining_time(void);
```

- **Purpose**: Query remaining time on the current deadline.
- **Returns**: Remaining time in milliseconds (≥0), or 0 if expired.

> [!NOTE]
> Should never return a negative value; clamp to 0 when expired.

## Interface-Specific Functions

Only required if the respective interface is enabled.

### SPI Interface

#### `loader_port_spi_set_cs()`

```c
void loader_port_spi_set_cs(uint32_t level);
```

- **When required**: If building with `SERIAL_FLASHER_INTERFACE_SPI`.
- **Purpose**: Control the SPI chip-select signal.

**Parameters**:

- `level` boolean-like state (active/inactive)

> [!NOTE]
> Ensure CS timing meets target requirements.

### SDIO Interface

#### `loader_port_sdio_card_init()`

```c
esp_loader_error_t loader_port_sdio_card_init(void);
```

- **When required**: If building with `SERIAL_FLASHER_INTERFACE_SDIO`.
- **Purpose**: Initialize SDIO communication to the target.
- **Returns**: `ESP_LOADER_SUCCESS` on success, negative error code on failure.

> [!NOTE]
> Configure bus width, speed, and any interrupts required by the host SoC.

## Optional Convenience Functions

These helpers are declared in `include/esp_loader_io.h` but are not called by the core library unless your port uses them. Implement as needed for your platform.

#### `loader_port_change_transmission_rate()` (Non-SDIO only)

```c
esp_loader_error_t loader_port_change_transmission_rate(uint32_t transmission_rate);
```

- **Purpose**: Change communication baud rate/speed.

> [!NOTE]
> Useful after initial sync to speed up transfers. Not available for SDIO interface.

#### `loader_port_reset_target()`

```c
void loader_port_reset_target(void);
```

- **Purpose**: Hardware reset of the target.

> [!NOTE]
> Can be used for recovery after errors.

#### `loader_port_debug_print()`

```c
void loader_port_debug_print(const char *str);
```

- **Purpose**: Print debug messages from the library.

> [!NOTE]
> Route to your platform's logging/console facility.

## Implementation Steps

There are two main approaches to adding support for your platform:

### Option A: Contributing to ESP Serial Flasher Repository

This approach involves adding your port directly to the ESP Serial Flasher repository.

#### 1. Add a Port File

Place a new C file under `port/` implementing the required functions for your platform:

```text
port/your_platform_port.c
port/your_platform_port.h  (if needed)
```

#### 2. Update Build System

Add your port to the main `CMakeLists.txt`:

```cmake
elseif(PORT STREQUAL "YOUR_PLATFORM")
    list(APPEND flasher_srcs "port/your_platform_port.c")
    # Add any platform-specific dependencies here
```

### Option B: Using ESP Serial Flasher as External Library

This approach uses ESP Serial Flasher as a submodule or external dependency in your own project. This is often the preferred approach as it keeps your port implementation within your own project and lets you simply update the submodule.

#### 1. Add ESP Serial Flasher to Your Project

**As Git Submodule:**

```bash
git submodule add https://github.com/espressif/esp-serial-flasher.git external/esp-serial-flasher
git submodule update --init --recursive
```

#### 2. Implement Port Functions in Your Project

Create your port implementation in your own source tree:

```text
your_project/
├── main/
│   ├── main.c                # Your application code
│   ├── your_port.c           # Your port implementation
│   ├── your_port.h           # Your port declarations (optional)
│   └── CMakeLists.txt        # Main application build
├── external/
│   └── esp-serial-flasher/   # Git submodule
├── CMakeLists.txt            # Root project configuration
└── .gitmodules               # Git submodule configuration
```

#### 3. Configure Build System

**Main CMakeLists.txt:**

```cmake
cmake_minimum_required(VERSION 3.16)
project(your_project C)

# Configure ESP Serial Flasher
set(SERIAL_FLASHER_INTERFACE_UART true)  # or SPI/USB/SDIO
set(PORT USER_DEFINED)

# Add ESP Serial Flasher submodule
add_subdirectory(external/esp-serial-flasher)

# Add your application
add_subdirectory(main)
```

**main/CMakeLists.txt:**

```cmake
# Create your executable
add_executable(your_app main.c)

# Add port implementation directly to flasher target
target_sources(flasher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/your_port.c
)

# Set up include directories
target_include_directories(your_app PRIVATE
    ${CMAKE_SOURCE_DIR}/external/esp-serial-flasher/include
)

target_include_directories(flasher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/external/esp-serial-flasher/include
)

# Link ESP Serial Flasher
target_link_libraries(your_app PRIVATE flasher)
```

**Key Points:**

- Set `PORT=USER_DEFINED` to tell ESP Serial Flasher to expect external port implementation
- Use `target_sources(flasher PRIVATE ...)` to add your port files to the flasher library target
- Set up include directories for both your app and the flasher target
- Interface selection is done via CMake variables (e.g., `SERIAL_FLASHER_INTERFACE_UART=true`)

## Submitting Your Port

### Before Submitting

1. Test thoroughly on your platform
2. Follow coding standards (see [Contributing Guide](contributing.md))
3. Add documentation for your platform
4. Provide or reference a minimal example demonstrating usage

### Pull Request

1. Open an issue first to discuss the new platform support
2. Create a pull request with your implementation
3. Include example and documentation
4. Provide testing results

## Getting Help

### Resources

- Study existing port implementations in the `port/` directory
- Check `include/esp_loader_io.h` for function specifications
- Review `examples/` for usage patterns

### Support

- Open a [GitHub issue](https://github.com/espressif/esp-serial-flasher/issues) for questions
