# Changelog for USB Host Library

All notable changes to this component will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.4.0] - 2026-04-02

### Added

- Add ESP32-S31 to supported targets

### Fixed

- Fixed possible out-of-bounds read in `usb_parse_next_descriptor()` in case of malicious Configuration descriptor

## [1.3.0] - 2026-03-04

### Added

- USB Dual Host support on ESP32-P4

### Fixed

- Various security fixes

## [1.2.0] - 2026-02-05

### Added

- Device initiated remote wakeup (https://github.com/espressif/esp-usb/pull/298)

### Fixed

- Fixed deadlock that prevented closing devices from high priority tasks (https://github.com/espressif/esp-idf/issues/17707)

## [1.1.0] - 2025-12-04

### Added

- Global suspend/resume (https://github.com/espressif/esp-usb/pull/275)

### Fixed

- Hardware FIFO size biasing Kconfig now shows allocation formulas instead of chip-specific examples

## [1.0.1] - 2025-10-16

### Fixed

- Compatibility with esp-idf 5.5 and 5.4 releases (https://github.com/espressif/esp-usb/pull/273)
- Printing of High Speed periodic endpoint descriptor now also shows additional transactions in microframe (https://github.com/espressif/esp-usb/pull/282)

## [1.0.0] - 2025-09-18

- First release: taken from usb component present in esp-idf at commit [`166269f`](https://github.com/espressif/esp-idf/commit/166269fb9338607aa9726ecc4ea2d1763de31f0e)
