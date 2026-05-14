#!/bin/bash

set -e

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    cat <<'EOF'
Usage: scripts/flash_merged.sh <merged_firmware.bin> [chip]
  merged_firmware.bin: path to merged firmware file
  chip:                (optional) target chip, e.g. esp32c3 (trmnl) or esp32s3 (TRMNL_X).
                       Defaults to 'auto' so esptool detects the connected chip.

Env vars:
  NO_STUB=1   Flash via the ROM loader instead of the stub flasher (slower, but
              works around "chip stopped responding" on large writes over the
              ESP32-S3 USB-Serial/JTAG link). Implies --no-compress.

Examples:
  scripts/flash_merged.sh my-image.bin                 # auto-detect chip
  scripts/flash_merged.sh my-image.bin esp32s3         # force ESP32-S3 (TRMNL_X)
  NO_STUB=1 scripts/flash_merged.sh my-image.bin esp32s3
EOF
    exit 1
fi

FIRMWARE_FILE="$1"
CHIP="${2:-auto}"
BAUD_RATE=$((115200 * 4))
ESPTOOL="pio pkg exec -p tool-esptoolpy esptool.py -- "

WRITE_OPTS=()
EXTRA_OPTS=()
MODE_LABEL="stub"
if [ "${NO_STUB:-0}" != "0" ]; then
    EXTRA_OPTS+=(--no-stub)
    WRITE_OPTS+=(--no-compress)
    MODE_LABEL="no-stub"
fi

if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "Error: Firmware file not found: $FIRMWARE_FILE"
    exit 1
fi

echo ""
echo "⚡ Flashing firmware (chip: $CHIP, mode: $MODE_LABEL)..."

if PYTHONIOENCODING=utf-8 $ESPTOOL --chip "$CHIP" --baud "$BAUD_RATE" "${EXTRA_OPTS[@]}" write_flash "${WRITE_OPTS[@]}" 0x0000 "$FIRMWARE_FILE"; then
    echo ""
    echo "✅ Flash completed successfully!"
    echo "🔄 Device will reboot automatically"
    echo ""
else
    echo ""
    echo "❌ Flash failed!"
    if [ "${NO_STUB:-0}" = "0" ]; then
        echo "💡 If it died mid-write with \"chip stopped responding\" (common on ESP32-S3"
        echo "   USB-Serial/JTAG with large images), retry with the ROM loader:"
        echo "     NO_STUB=1 $0 $FIRMWARE_FILE $CHIP"
    fi
    echo ""
    exit 1
fi