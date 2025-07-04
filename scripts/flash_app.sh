#!/bin/bash

set -e

if [ $# -eq 0 ]; then
    echo "Usage: $0 <firmware.bin>"
    echo "  firmware.bin: path to application firmware file"
    echo ""
    echo "Example:"
    echo "  $0 .pio/build/local/firmware.bin"
    exit 1
fi

FIRMWARE_FILE="$1"
BAUD_RATE=$((115200 * 4))
ESPTOOL="pio pkg exec -p tool-esptoolpy esptool.py -- "
ESPTOOL_CMD="$ESPTOOL --chip esp32c3 --baud $BAUD_RATE"

if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "Error: Firmware file not found: $FIRMWARE_FILE"
    exit 1
fi

echo ""
echo "⚡ Flashing application firmware..."

$ESPTOOL_CMD write_flash 0x10000 "$FIRMWARE_FILE"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Application flash completed successfully!"
    echo "🔄 Device will reboot automatically"
    echo ""
else
    echo ""
    echo "❌ Application flash failed!"
    echo ""
    exit 1
fi