# Platform Setup Guide

This document provides setup instructions for using ESP Serial Flasher on different host platforms to program and interact with ESP devices.

## General Prerequisites

Before setting up any platform, ensure you have the following:

- **[CMake](https://cmake.org/) 3.5 or later** - Build system generator
- **Platform-specific toolchain** - See individual platform sections below

## ESP-IDF Support

**Testing Status**: Regularly tested with all major and minor versions of ESP-IDF from v4.3 to v5.5

### Prerequisites

- **[ESP-IDF](https://docs.espressif.com/projects/esp-idf/) v4.3+** - ESP-IDF development framework

### Setup

ESP Serial Flasher is available as a managed component through the Espressif Component Registry. This is the recommended installation method for ESP-IDF projects.

**Installation:**

1. **Navigate to your ESP-IDF project directory**
2. **Add the component dependency**:
   ```bash
   idf.py add-dependency "espressif/esp-serial-flasher"
   ```
3. **Build your project**:
   ```bash
   idf.py build
   ```

The component will be automatically downloaded and integrated into your project's build system.

**Component Registry**: [espressif/esp-serial-flasher](https://components.espressif.com/components/espressif/esp-serial-flasher/)

### Example Code

See [examples/esp32_example](../examples/esp32_example) for a complete implementation with build instructions.

## STM32 Setup

**Testing Status**: Regularly tested with STM32CubeH7 v1.11.1 and arm-gnu-toolchain-13.2.

### Prerequisites

- **[Git](https://git-scm.com/)** - For cloning with submodules
- **[STM32 HAL libraries](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)** - For your specific target chip family
- **[ARM GCC toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)** - Cross-compilation toolchain
- **[stm32-cmake](https://github.com/ObKo/stm32-cmake) support package** - Included as submodule

### Setup

1. **Clone the repository with submodules**:

   ```bash
   git clone --recursive https://github.com/espressif/esp-serial-flasher.git
   ```

2. **If already cloned without `--recursive`**, initialize the submodule:

   ```bash
   git submodule update --init --recursive
   ```

### Required CMake Variables

Define these variables when configuring the build:

- `STM32_TOOLCHAIN_PATH`: Path to ARM toolchain
- `STM32_CUBE_<CHIP_FAMILY>_PATH`: Path to STM32 Cube libraries
- `STM32_CHIP`: Target STM32 chip name
- `CORE_USED`: Core used on multicore devices (e.g., M7 or M4)
- `PORT`: Must be set to `STM32`

### Build Example

```bash
cmake \
  -DSTM32_TOOLCHAIN_PATH="/path/to/gcc-arm-none-eabi" \
  -DSTM32_CUBE_H7_PATH="/path/to/STM32Cube_FW_H7" \
  -DSTM32_CHIP="STM32H743VI" \
  -DCORE_USED="M7" \
  -DPORT="STM32" \
  .. && cmake --build .
```

### Alternative: CMake Variables File

Set variables in your top-level `CMakeLists.txt`:

```cmake
set(STM32_TOOLCHAIN_PATH /path/to/gcc-arm-none-eabi)
set(STM32_CUBE_H7_PATH /path/to/STM32Cube_FW_H7)
set(STM32_CHIP STM32H743VI)
set(CORE_USED M7)
set(PORT STM32)
```

### Example Code

See [examples/stm32_example](../examples/stm32_example) for a complete implementation with build instructions.

## Zephyr Setup

**Testing Status**: Regularly tested with Zephyr RTOS v4.0.0 and Zephyr SDK v0.17.0.

### Prerequisites

- **[Zephyr RTOS](https://zephyrproject.org/)** - Real-time operating system
- **[Zephyr SDK](https://github.com/zephyrproject-rtos/sdk-ng)** - Development toolkit
- **[West](https://docs.zephyrproject.org/latest/develop/west/index.html)** - Zephyr's meta-tool for managing repositories

### Setup

ESP Serial Flasher can be integrated as a Zephyr module. Add the following to your West manifest file (`west.yml`):

```yaml
manifest:
  projects:
    - name: esp-flasher
      url: https://github.com/espressif/esp-serial-flasher
      revision: master
      path: modules/lib/esp_flasher
```

After updating the manifest, fetch the module:

```bash
west update
```

### Project Configuration

Add these configuration options to your `prj.conf`:

```text
CONFIG_ESP_SERIAL_FLASHER=y
CONFIG_CONSOLE_GETCHAR=y
CONFIG_SERIAL_FLASHER_MD5_ENABLED=y
```

### Example Code

See [examples/zephyr_example](../examples/zephyr_example) for a complete implementation with build instructions.

## Raspberry Pi Pico Setup

**Testing Status**: Regularly tested with Raspberry Pi Pico SDK v1.5.1 and arm-gnu-toolchain-13.2.

### Prerequisites

- **[Git](https://git-scm.com/)** - For cloning repositories
- **[Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)** - Development framework
- **[ARM GCC toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)** - Cross-compilation toolchain

### Setup

1. **Clone the repository**:

   ```bash
   git clone https://github.com/espressif/esp-serial-flasher.git
   cd esp-serial-flasher
   ```

2. **Ensure Pico SDK is available**:

   - Either set `PICO_SDK_PATH` environment variable
   - Or place the Pico SDK in a standard location

### Example Code

See [examples/pi_pico_example](../examples/pi_pico_example) for a complete implementation with build instructions.

## Raspberry Pi Setup

**Testing Status**: Regularly tested with the latest Raspberry Pi OS.

### Prerequisites

- **[Git](https://git-scm.com/)** - For cloning repositories
- **[Raspberry Pi OS](https://www.raspberrypi.org/software/)** - Latest version recommended
- **[pigpio library](https://abyz.me.uk/rpi/pigpio/)** - For GPIO control and hardware interfacing

### Setup

1. **Clone the repository**:

   ```bash
   git clone https://github.com/espressif/esp-serial-flasher.git
   ```

2. **Install pigpio library** (if not already installed):

   ```bash
   sudo apt update
   sudo apt install pigpio
   ```

### Example Code

See [examples/raspberry_example](../examples/raspberry_example) for a complete implementation with build instructions.

## Next Steps

After successful platform setup:

1. **Review the [Configuration Guide](configuration.md)** for library configuration options
2. **Check platform-specific examples** in the `examples/` directory
3. **Read the [API documentation](../README.md)** for usage details
