#!/bin/bash

set -e

ENVS=(trmnl trmnl_4clr TRMNL_X)
CONFIG="include/config.h"

if [ ! -f "$CONFIG" ]; then
    echo "Error: $CONFIG not found. Run from the firmware repo root."
    exit 1
fi

MAJOR=$(awk '/#define FW_MAJOR_VERSION/ {print $3}' "$CONFIG")
MINOR=$(awk '/#define FW_MINOR_VERSION/ {print $3}' "$CONFIG")
PATCH=$(awk '/#define FW_PATCH_VERSION/ {print $3}' "$CONFIG")
VERSION="$MAJOR.$MINOR.$PATCH"
EXPECTED_MSG="v$VERSION"

echo "Firmware version:        $VERSION"
echo "Expected commit message: $EXPECTED_MSG"
echo "Environments:            ${ENVS[*]}"
echo

HEAD_MSG=$(git log -1 --pretty=%s)
if [ "$HEAD_MSG" != "$EXPECTED_MSG" ]; then
    echo "Error: HEAD commit message is \"$HEAD_MSG\", expected \"$EXPECTED_MSG\""
    exit 1
fi

read -r -p "Build release for $EXPECTED_MSG? [y/N] " reply
case "$reply" in
    [yY]|[yY][eE][sS]) ;;
    *) echo "Aborted."; exit 1 ;;
esac

for env in "${ENVS[@]}"; do
    echo
    echo "=== Building $env ==="
    pio run -e "$env"

    BUILD_DIR=".pio/build/$env"
    OTA_DIR=".pio/release/$env/ota"
    FLASH_DIR=".pio/release/$env/flash"
    OUT_NAME="FW$VERSION.bin"

    mkdir -p "$OTA_DIR" "$FLASH_DIR"

    cp "$BUILD_DIR/firmware.bin" "$OTA_DIR/$OUT_NAME"
    cp "$BUILD_DIR/merged_firmware.bin" "$FLASH_DIR/$OUT_NAME"

    echo "  ota:   $OTA_DIR/$OUT_NAME"
    echo "  flash: $FLASH_DIR/$OUT_NAME"
done

echo
echo "Release $TAG ready in .pio/release/"
