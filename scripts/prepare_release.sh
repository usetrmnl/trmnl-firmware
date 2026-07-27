#!/bin/bash

set -e

TRMNL_ENVS=(
    trmnl
    trmnl_4clr
    TRMNL_X
)

BYOD_ENVS=(
    seeed_xiao_esp32c3
    xteink_x4
    TRMNL_7inch5_OG_DIY_Kit
    seeed_reTerminal_E1001
    seeed_reTerminal_E1002
    TRMNL_X_E1003
)

CONFIG="include/config.h"
FORCE=0
ENV_INPUT=()
ENVS=()

for arg in "$@"; do
    case "$arg" in
        --force|-f) FORCE=1 ;;
        --help) echo "Usage: $0 [--force] [trmnl] [byod] [env1] [env2] [...]"; exit 0 ;;
        *) ENV_INPUT+=("$arg") ;;
    esac
done

# expand "trmnl" and "byod" to multiple environments
for i in "${!ENV_INPUT[@]}"; do
    case "${ENV_INPUT[$i]}" in
        trmnl) ENVS+=("${TRMNL_ENVS[@]}") ;;
        byod) ENVS+=("${BYOD_ENVS[@]}") ;;
        *) ENVS+=("${ENV_INPUT[$i]}") ;;
    esac
done

# default to TRMNL_ENVS if blank
if [ ${#ENVS[@]} -eq 0 ]; then
    ENVS=("${TRMNL_ENVS[@]}")
fi

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
    if [ "$FORCE" -eq 1 ]; then
        echo "Warning: HEAD commit message is \"$HEAD_MSG\", expected \"$EXPECTED_MSG\" (bypassed with --force)"
    else
        echo "Error: HEAD commit message is \"$HEAD_MSG\", expected \"$EXPECTED_MSG\""
        echo "Use --force to bypass this check."
        exit 1
    fi
fi

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
