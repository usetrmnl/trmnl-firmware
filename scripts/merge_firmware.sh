#!/bin/bash

set -e

usage() {
    cat <<'EOF'
Usage: scripts/merge_firmware.sh <build_directory> <output_name> [littlefs.bin] [fs_offset]
  build_directory: path to directory containing bootloader.bin, partitions.bin, firmware.bin
  output_name:     name for the merged output file
  littlefs.bin:    (optional) path to a filesystem image to append
  fs_offset:       (optional) flash offset for the filesystem image (defaults to the board profile)

Board profile (chip / flash settings / partition offsets) is auto-detected from the
build directory name. Override with the BOARD env var. Supported:
  trmnl   -> esp32c3, dio/40m/4MB,  app @ 0x10000, fs @ 0x3B0000   (default)
  TRMNL_X -> esp32s3, qio/80m/16MB, app @ 0x20000, fs @ 0x620000

Examples:
  scripts/merge_firmware.sh .pio/build/trmnl my-image.bin
  scripts/merge_firmware.sh .pio/build/trmnl my-image.bin .pio/build/trmnl/littlefs.bin
  scripts/merge_firmware.sh .pio/build/TRMNL_X trmnl_x.bin .pio/build/littlefs.bin
  BOARD=TRMNL_X scripts/merge_firmware.sh some/dir trmnl_x.bin some/dir/littlefs.bin
EOF
}

if [ $# -lt 2 ] || [ $# -gt 4 ]; then
    usage
    exit 1
fi

BUILD_DIR=$1
OUTPUT_NAME=$2
LITTLEFS_BIN=${3:-}
FS_OFFSET_OVERRIDE=${4:-}

BOARD=${BOARD:-$(basename "$BUILD_DIR")}

case "$BOARD" in
    TRMNL_X*)
        CHIP=esp32s3
        FLASH_MODE=qio
        FLASH_FREQ=80m
        FLASH_SIZE=16MB
        APP_OFFSET=0x20000
        FS_OFFSET_DEFAULT=0x620000
        ;;
    *)  # trmnl, trmnl_4clr, xteink_x4, local, ... (ESP32-C3, min_spiffs.csv)
        CHIP=esp32c3
        FLASH_MODE=dio
        FLASH_FREQ=40m
        FLASH_SIZE=4MB
        APP_OFFSET=0x10000
        FS_OFFSET_DEFAULT=0x3B0000
        ;;
esac

FS_OFFSET=${FS_OFFSET_OVERRIDE:-$FS_OFFSET_DEFAULT}

# Check if build files exist
for f in bootloader.bin partitions.bin firmware.bin; do
    if [ ! -f "$BUILD_DIR/$f" ]; then
        echo "Error: $f not found in $BUILD_DIR. Run 'pio run -e $BOARD' first."
        exit 1
    fi
done

if [ -n "$LITTLEFS_BIN" ] && [ ! -f "$LITTLEFS_BIN" ]; then
    echo "Error: filesystem image not found: $LITTLEFS_BIN"
    echo "Run 'pio run -e $BOARD -t buildfs' first."
    exit 1
fi

ESPTOOL="pio pkg exec -p tool-esptoolpy esptool.py -- "

echo "Board profile: $BOARD  (chip=$CHIP mode=$FLASH_MODE freq=$FLASH_FREQ size=$FLASH_SIZE app@$APP_OFFSET)"
echo "Using esptool: $ESPTOOL"

MERGE_ARGS=(
    --chip "$CHIP" merge_bin
    -o "$OUTPUT_NAME"
    --flash_mode "$FLASH_MODE"
    --flash_freq "$FLASH_FREQ"
    --flash_size "$FLASH_SIZE"
    0x0000 "$BUILD_DIR/bootloader.bin"
    0x8000 "$BUILD_DIR/partitions.bin"
    "$APP_OFFSET" "$BUILD_DIR/firmware.bin"
)

if [ -n "$LITTLEFS_BIN" ]; then
    echo "Including filesystem image $LITTLEFS_BIN at $FS_OFFSET"
    MERGE_ARGS+=("$FS_OFFSET" "$LITTLEFS_BIN")
fi

$ESPTOOL "${MERGE_ARGS[@]}"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Merged firmware created successfully!"
    echo "📁 Location: $OUTPUT_NAME"
    echo ""
    echo "To flash this merged firmware:"
    echo "./scripts/flash_merged.sh $OUTPUT_NAME $CHIP"
    if [ "$CHIP" = "esp32s3" ]; then
        echo "  (if it dies mid-write with \"chip stopped responding\", use: NO_STUB=1 ./scripts/flash_merged.sh $OUTPUT_NAME $CHIP)"
    fi
    echo ""
else
    echo "❌ Failed to create merged firmware"
    exit 1
fi