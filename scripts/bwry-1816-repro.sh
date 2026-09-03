#!/usr/bin/env bash
#
# bwry-1816-repro.sh — reproduce the TRMNL BWRY 1.8.16 blank-screen bug.
#
# Symptom: a factory-fresh (virgin NVS) BWRY flashed with the 1.8.16 RELEASE
# image draws the loading screen, goes blank, and never draws the WiFi portal.
# Roughly 2 boots in 3. Devices that reach 1.8.16 by OTA are unaffected.
#
# The bug only shows on the FIRST boot after a wipe, because the first boot
# writes the factory QA "testPassed" key and every later boot takes a different
# path. So every trial needs a fresh NVS wipe, which is what this script does.
#
# Requires: esptool (pip install esptool), curl, python3.
# Usage:    ./bwry-1816-repro.sh help
#
set -euo pipefail

CHIP=esp32c3
NVS_OFFSET=0x9000
NVS_SIZE=0x5000
BASE=https://trmnl-fw.s3.us-east-2.amazonaws.com/trmnl_bwry
CACHE="${TMPDIR:-/tmp}/bwry-1816-repro"

# name|url|sha256
IMG_BAD="bad|$BASE/flash/FW1.8.16.bin|916f631c7d8813e284048ea0fdaeaef8895e09f4e89407d7bd808546fa7092f4"
IMG_CAND="candidate|$BASE/dev/FW1.8.16-trmnl_bwry-flash-22dd41d.bin|91ecd1cb40aba661e36fbd467f46e1e83db4a740baf5b51293ec2a79f7606828"
IMG_1815="v1815|$BASE/flash/FW1.8.15.bin|906ef60d0a0b4de618b2e0831af7c72a9833428be1502d6d4ffe55db12a21b5e"

die() { echo "error: $*" >&2; exit 1; }

find_port() {
  # Deliberately NOT called PORT: that name is already taken in most dev shells.
  if [ -n "${BWRY_PORT:-}" ]; then
    [ -e "$BWRY_PORT" ] || die "BWRY_PORT=$BWRY_PORT does not exist"
    echo "$BWRY_PORT"; return
  fi
  local p
  for p in /dev/cu.usbmodem* /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$p" ] && { echo "$p"; return; }
  done
  die "no serial port found. Plug in USB and press the device button to wake it, or set BWRY_PORT=/dev/..."
}

esptool_cmd() {
  if command -v esptool >/dev/null 2>&1; then echo "esptool";
  elif command -v esptool.py >/dev/null 2>&1; then echo "esptool.py";
  else die "esptool not found. Install with: pip install esptool"; fi
}

# esptool 4.x needs underscore subcommands; 5.x accepts them with a deprecation
# warning. Underscores therefore work on both.
et() {
  local port; port=$(find_port)
  $(esptool_cmd) --port "$port" --chip "$CHIP" --before default_reset --after no_reset "$@"
}

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | awk '{print $1}';
  else shasum -a 256 "$1" | awk '{print $1}'; fi
}

fetch() { # fetch <spec> -> prints local path
  local spec=$1 name url want file got
  name=${spec%%|*}; spec=${spec#*|}; url=${spec%%|*}; want=${spec##*|}
  file="$CACHE/$name.bin"
  mkdir -p "$CACHE"
  if [ ! -f "$file" ]; then
    echo "downloading $name ..." >&2
    curl -fsS -o "$file" "$url" || die "download failed: $url"
  fi
  got=$(sha256_of "$file")
  [ "$got" = "$want" ] || die "checksum mismatch for $name (got $got, want $want)"
  echo "$file"
}

wipe_nvs() {
  echo "port: $(find_port)"
  echo "wiping NVS ($NVS_SIZE bytes at $NVS_OFFSET) ..."
  et --baud 460800 erase_region "$NVS_OFFSET" "$NVS_SIZE" >/dev/null 2>&1 \
    || die "NVS erase failed"
  local out="$CACHE/nvs_verify.bin"
  et read_flash "$NVS_OFFSET" "$NVS_SIZE" "$out" >/dev/null 2>&1 \
    || die "NVS read-back failed"
  python3 - "$out" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read()
bad = sum(1 for b in d if b != 0xff)
if bad:
    print(f"WIPE FAILED: {bad} of {len(d)} bytes are not blank. DO NOT COUNT THIS TRIAL.")
    sys.exit(1)
print(f"NVS verified blank ({len(d)}/{len(d)} bytes 0xff) — this is a virgin first boot")
PY
}

manual_steps() {
  cat <<'TXT'

  Next, by hand:
    1. Unplug the USB cable.
    2. Disconnect and reconnect the battery. A power-on reset is required;
       a USB-initiated reset does NOT reproduce this bug (see NOTES below).
    3. Watch the panel for about 60 seconds and record the outcome:
         PASS  loading screen -> brief blank -> WiFi portal screen
         FAIL  loading screen -> blank, and it stays blank
    4. For the next trial: plug USB back in (press the button if the port does
       not appear) and run:  ./bwry-1816-repro.sh wipe

TXT
}

flash_image() { # flash_image <spec> <label>
  local file port out
  port=$(find_port)   # dies here if no device, before we claim to be writing
  echo "port: $port"
  file=$(fetch "$1")
  echo "writing $2 image at 0x0 ($(wc -c <"$file" | tr -d ' ') bytes) ..."
  out=$(et --baud 460800 write_flash 0x0 "$file" 2>&1) \
    || { echo "$out" | tail -5 >&2; die "flash write failed"; }
  echo "$out" | tr -d '\r' | grep -Ei 'wrote|hash of data' || true
  echo "$out" | grep -qi 'hash of data verified' \
    || die "esptool did not confirm 'Hash of data verified' — do not trust this write"
  wipe_nvs
  manual_steps
}

case "${1:-help}" in
  bad)       flash_image "$IMG_BAD"  "KNOWN-BAD 1.8.16 release" ;;
  candidate) flash_image "$IMG_CAND" "CANDIDATE 1.8.16 (CI dev build of 22dd41d)" ;;
  v1815)     flash_image "$IMG_1815" "KNOWN-GOOD 1.8.15 release" ;;
  wipe)      wipe_nvs; manual_steps ;;
  info)
    echo "port:    $(find_port)"
    echo "esptool: $($(esptool_cmd) version 2>/dev/null | head -1)"
    et flash_id 2>&1 | grep -Ei 'chip is|features|MAC|flash size' || true
    ;;
  help|*)
    awk 'NR>1 { if (/^#/) { sub(/^# ?/, ""); print } else exit }' "$0"
    cat <<'TXT'
Commands:
  bad         flash the S3 1.8.16 release image that fails  (expect ~2/3 FAIL)
  candidate   flash the CI dev build of 22dd41d             (expect PASS)
  v1815       flash the 1.8.15 release image                (expect PASS)
  wipe        wipe + verify NVS between trials              (run before EVERY trial)
  info        show port, esptool version, chip id

Environment:
  BWRY_PORT=/dev/ttyACM0   override serial port autodetection

NOTES
  - Commit 22dd41d has byte-identical source to tag v1.8.16 and is on main.
    Both report "1.8.16" to the server; the commit hash only appears on the
    setup screen. So the candidate is the same firmware, a different build.
  - A USB-initiated boot (esptool run, or opening the port) does NOT reproduce
    the failure: the known-bad image passed 2/2 that way but failed 2/3 on
    battery power-on. Only power-on resets count.
  - Console: CONFIG_ESP_CONSOLE_UART_DEFAULT with USB-serial-JTAG as SECONDARY.
    ESP-IDF "I (123) ..." lines reach USB, but the app's own ArduinoLog output
    goes to UART0 only (ESP32-C3 default GPIO21 TX / GPIO20 RX, 115200).
    Getting the failing boot's app log needs a 3.3V UART adapter on UART0.
  - The USB console dies about 7s into the boot when the first panel refresh
    enters light sleep. That is expected and is not the failure.
  - Ruled out: heap exhaustion (180 KB largest free block before the 96,000
    byte framebuffer malloc), library/toolchain drift, bootloader and partition
    table differences, and all three code changes in the tag. Nine builds of
    identical source pass; only the release artifact fails. The one difference
    found is link order: the release build places ~12 KB of WiFi driver rodata
    ahead of the app's data.
TXT
    ;;
esac
