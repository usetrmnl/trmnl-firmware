# USB Host Library

[![Component Registry](https://components.espressif.com/components/espressif/usb/badge.svg)](https://components.espressif.com/components/espressif/usb) ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) ![changelog](https://img.shields.io/badge/Keep_a_Changelog-blue?logo=keepachangelog&logoColor=E05735)

This component contains an implementation of a [USB Host Library](https://docs.espressif.com/projects/esp-usb/en/latest/esp32p4/usb_host.html).

In a typical USB application, the USB Host Library works underneath the USB Host Class drivers:

- [CDC](../class/cdc/) Communication Device Class (ACM)
- [HID](../class/hid/usb_host_hid/) Human Interface Device
- [MSC](../class/msc/usb_host_msc/) Mass Storage Class
- [UAC](../class/uac/usb_host_uac/) USB Audio Class
- [UVC](../class/uvc/usb_host_uvc/) USB Video Class

## esp-idf compatibility

The USB Host Library is included in [esp-idf](https://github.com/espressif/esp-idf/tree/release/v5.5/components/usb) versions 5.x. Starting with version 6.0, it is no longer part of esp-idf and can only be used as a managed component.

### Bugfixes and new features

**Bugfixes** are backported to all active esp-idf 5.x releases branches. You can find the currently supported release branches [here](https://github.com/espressif/esp-idf?tab=readme-ov-file#esp-idf-release-support-schedule).

**New features** are added only to this component. However, backward compatibility is maintained for all esp-idf service-period branches. This means that users of active 5.x releases can override the version of the library bundled with esp-idf if they need features introduced after esp-idf version 6.0.

## Usage

To include this component in your project:

1. Add the USB Component to your project using the [Espressif Component Registry](https://components.espressif.com/):

Using `idf.py` command:

```bash
idf.py add-dependency espressif/usb
```

Or modifying you `idf_component.yml` file:

```yaml
## IDF Component Manager Manifest File
dependencies:
  ...
  espressif/usb: "*"
```

2. Include the USB Host header file in your code:

```c
#include "usb/usb_host.h"
```

3. Initialize the USB Host Library. The library is a singleton, it must be initialized only once for all class drivers:

```c
usb_host_config_t host_config = {/* configuration */};
usb_host_install(&host_config);
```

4. Use class drivers (CDC, HID, ets.) on top of the USB Host Library, to interact with USB devices

## Examples

Refer to following examples, using USB Host library from esp-idf:

- [CDC Host](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/cdc)
- [HID Host](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/hid)
- [MSC Host](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/msc)
- [USB Host Library](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/usb_host_lib)
- [UVC Host](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/uvc)
