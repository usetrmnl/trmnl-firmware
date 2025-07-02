#!/bin/bash

set -e

if [ $# -eq 2 ]; then
    BUILD_DIR=$1
    OUTPUT_NAME=$2
else
    echo "Usage: $0 [build_directory] [output_name]"
    echo "  build_directory: relative path to directory containing bootloader.bin, partitions.bin, firmware.bin"
    echo "  output_name: name for output file"
    echo ""
    echo "Example:"
    echo "  $0 .pio/build/trmnl my-image.bin"
    exit 1
fi

# Check if build files exist
if [ ! -f "$BUILD_DIR/bootloader.bin" ]; then
    echo "Error: bootloader.bin not found. Run 'pio run -e trmnl' first."
    exit 1
fi

if [ ! -f "$BUILD_DIR/partitions.bin" ]; then
    echo "Error: partitions.bin not found. Run 'pio run -e trmnl' first."
    exit 1
fi

if [ ! -f "$BUILD_DIR/firmware.bin" ]; then
    echo "Error: firmware.bin not found. Run 'pio run -e trmnl' first."
    exit 1
fi

ESPTOOL="pio pkg exec -p tool-esptoolpy esptool.py -- "

echo "Using esptool: $ESPTOOL"

$ESPTOOL --chip esp32c3 merge_bin \
    -o "$OUTPUT_NAME" \
    --flash_mode dio \
    --flash_freq 40m \
    --flash_size 4MB \
    0x0000 "$BUILD_DIR/bootloader.bin" \
    0x8000 "$BUILD_DIR/partitions.bin" \
    0x10000 "$BUILD_DIR/firmware.bin"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Merged firmware created successfully!"
    echo "📁 Location: $OUTPUT_NAME"
    echo ""
    echo "To flash this merged firmware:"
    echo "./scripts/flash_merged.sh $OUTPUT_NAME"
    echo ""
else
    echo "❌ Failed to create merged firmware"
    exit 1
fi
