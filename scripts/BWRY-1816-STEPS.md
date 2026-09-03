# BWRY 1.8.16 blank screen on factory-fresh flash

## The bug

A BWRY flashed from the flash page with **1.8.16** draws the loading screen,
goes blank, and never draws the WiFi portal. Roughly **2 boots in 3**.

Reported by Niklas (factory flashing station) on 1 Sep. It started when the
1.8.16 flash image went live on **31 Aug 20:54 UTC**; before that the station
was serving 1.8.15 and had no trouble.

**Only virgin (factory-fresh) devices are affected.** The first boot writes the
QA `testPassed` key, and every later boot takes a different path. Devices that
reached 1.8.16 by OTA already have that key, which is why 364 BWRY units are
running 1.8.16 in the field with no problem.

## Reproducing it

You need a BWRY, a USB cable, and `pip install esptool`.

```
./bwry-1816-repro.sh bad     # flash the failing S3 release image, wipe + verify NVS
# unplug USB, cycle the battery, watch ~60s  -> expect blank about 2 times in 3

./bwry-1816-repro.sh wipe    # before EVERY subsequent trial
# unplug USB, cycle the battery, watch ~60s
```

Two rules, both learned the hard way:

1. **Power-on reset only.** Unplug USB *and* cycle the battery. A USB-initiated
   boot does not reproduce this: the known-bad image passed 2 of 2 that way
   while failing 2 of 3 on battery.
2. **Verify the wipe every time.** The script reads all 20,480 NVS bytes back
   and refuses to continue unless every one is `0xff`. An unverified wipe cost
   us one wasted trial early on.

Comparison images: `./bwry-1816-repro.sh v1815` (known good) and
`./bwry-1816-repro.sh candidate` (see below).

## What is proven

- The failing artifact is **only** the release build. Nine builds of identical
  source all pass a virgin boot: three CI dev builds (`fb26d87`, `6b7183f`,
  `22dd41d`), a plain local `pio run -e trmnl_4clr`, a local run of
  `scripts/prepare_release.sh`, and four diagnostic builds.
- 1.8.15 release image: passes.
- The three code changes in the tag are cleared individually. `fb26d87` (#587,
  WiFi event handlers) and `6b7183f` (all three changes) both pass as CI builds.

## What is ruled out

| Hypothesis | Evidence against |
|---|---|
| Heap exhaustion | Largest free block was 180 KB right before the 96,000-byte framebuffer `malloc`, 86 KB after. Not marginal. |
| Library or toolchain drift | CI logs pin identical versions: espressif32 6.12.0, arduino 3.20017.241212, esp-idf 4.4.7, bb_epaper 2.1.9, AsyncTCP 3.5.0. Same 1,383 objects compiled. |
| Bootloader or partition table | Differs from 1.8.15 by 38 bytes, all compile timestamp. |
| Build mode | Both jobs log "Building in debug mode". |
| Corrupt S3 object | md5 matches the S3 ETag; image checksum and SHA256 both validate. |
| `display_update_epaper` failure | It always returns true on this board. |

## The one difference found

Link order. In the release binary the DROM segment carries about 12 KB of WiFi
driver rodata **before** the app's own data; in the dev binary the app comes
first. Application translation-unit order is otherwise identical, a constant
`+0x3044` shift. Segment sizes are identical.

That makes it look like a **latent, layout-sensitive bug** rather than anything
in the three commits. It has not been proven to be the cause.

## Getting logs out (the part that blocked us)

- `CONFIG_ESP_CONSOLE_UART_DEFAULT` with USB-serial-JTAG as **secondary**. So
  ESP-IDF `I (123) ...` lines reach USB, but the app's own ArduinoLog output
  goes to **UART0 only** (C3 default GPIO21 TX / GPIO20 RX, 115200).
- The USB console dies about **7 seconds** into the boot, when the first panel
  refresh enters light sleep. Expected, not the failure.
- On macOS, opening the port with pyserial resets the C3 into ROM download mode
  on every DTR/RTS combination tried. Booting the app needs either a battery
  cycle or `esp.run()` from an already-open esptool session.
- `esp_rom_printf` does reach USB, so a diagnostic build can be instrumented
  that way. `Log.*` will not show up.
- A blank boot leaves **no stored error** in NVS, and the softAP still comes
  up. Everything except the panel believes the boot succeeded.

**Next diagnostic step:** a 3.3 V UART adapter on UART0, capturing one blank
boot. That should name the failing step directly.

## The candidate for shipping 1.8.16

Commit `22dd41d` ("Bump to v1.8.16 to match release") has **byte-identical
source** to tag `v1.8.16` and is on main. Both report `1.8.16` to the server;
`FW_COMMIT` only changes the string on the setup screen. Its CI-built flash
image is on S3 and its app region is byte-identical to a build that already
passed a virgin boot.

```
https://trmnl-fw.s3.us-east-2.amazonaws.com/trmnl_bwry/dev/FW1.8.16-trmnl_bwry-flash-22dd41d.bin
sha256 91ecd1cb40aba661e36fbd467f46e1e83db4a740baf5b51293ec2a79f7606828
```

Validating it means battery-cycle trials, and the statistics are unforgiving.
Zero failures in *n* boots bounds the true failure rate at about 3/*n*:

| Clean boots | Failure rate still consistent |
|---|---|
| 5 | 45% |
| 10 | 30% |
| 20 | 15% |

Ten clean boots rules out anything resembling the current broken image. Nothing
short of root cause gets to zero.

## Reference

| Item | Value |
|---|---|
| Bad release image | `trmnl_bwry/flash/FW1.8.16.bin` sha256 `916f631c…92f4` |
| Candidate | `trmnl_bwry/dev/FW1.8.16-trmnl_bwry-flash-22dd41d.bin` sha256 `91ecd1cb…6828` |
| Known good | `trmnl_bwry/flash/FW1.8.15.bin` sha256 `906ef60d…1b5e` |
| Tag | `v1.8.16` = `76d8134`; release CI run `33093873325`, 27 Aug 16:33 UTC |
| Release workflow | `publish-release-firmware.yml` → `scripts/prepare_release.sh trmnl` |
| Dev workflow | `publish-dev-firmware.yml` → `pio run -e trmnl_4clr` |
| Core rows | `Firmware` id 399 (OTA, production), `FlashFirmware` id 88 (flash page) |
| NVS partition | offset `0x9000`, size `0x5000` |
| Test device | ESP32-C3 rev v0.4, 4 MB flash |

## Two unrelated bugs found on the way

1. `src/display.cpp`: `dpList[4]` is declared with four slots but the
   `BOARD_TRMNL_4CLR` block fills three. A server-sent `temp_profile` of 3
   indexes a zero entry. `iTempProfile` is never bounds-checked against the
   table.
2. `src/app_logger.cpp`: the `LOG_SERIAL_ONLY` branch of `log_impl` returns
   without freeing `serial_buffer` or `user_message`. Leaks on every
   serial-only log call.
