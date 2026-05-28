#!/bin/bash

set -e

usage() {
    cat <<'EOF'
Usage: scripts/copy_to_core.sh <device_env> <core_root_dir>
  device_env:    PlatformIO environment name (e.g. trmnl, TRMNL_X, seeed_xiao_esp32s3)
  core_root_dir: path to the core repo root

Copies .pio/build/<device_env>/merged_firmware.bin to
<core_root_dir>/public/firmware/<device_env>/FW<major>.<minor>.<patch>.bin
EOF
}

if [ $# -ne 2 ]; then
    usage
    exit 1
fi

DEVICE=$1
CORE_DIR=$2

SRC=".pio/build/$DEVICE/merged_firmware.bin"
CONFIG="include/config.h"

if [ ! -f "$SRC" ]; then
    echo "Error: $SRC not found. Run 'pio run -e $DEVICE' first."
    exit 1
fi

if [ ! -f "$CONFIG" ]; then
    echo "Error: $CONFIG not found. Run this script from the firmware repo root."
    exit 1
fi

MAJOR=$(awk '/#define FW_MAJOR_VERSION/ {print $3}' "$CONFIG")
MINOR=$(awk '/#define FW_MINOR_VERSION/ {print $3}' "$CONFIG")
PATCH=$(awk '/#define FW_PATCH_VERSION/ {print $3}' "$CONFIG")
VERSION="$MAJOR.$MINOR.$PATCH"

DEST_DIR="$CORE_DIR/public/firmware/$DEVICE"
DEST="$DEST_DIR/FW$VERSION.bin"

mkdir -p "$DEST_DIR"
cp "$SRC" "$DEST"

echo "Copied $SRC -> $DEST"
