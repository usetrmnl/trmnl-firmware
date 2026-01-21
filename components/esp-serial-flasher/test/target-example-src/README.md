# Firmware Example Sources

This directory contains standalone ESP-IDF example projects for producing target firmware binaries used by ESP Serial Flasher demos/tests.

## Building ESP32 Hello World

Flash build (default):

```bash
cd test/target-example-src/hello-world-ESP32-src
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.defaults.flash build
```

RAM build:

```bash
cd test/target-example-src/hello-world-ESP32-src
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.defaults.ram build
```
