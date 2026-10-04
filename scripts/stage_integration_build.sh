#!/usr/bin/env bash
# Copy what the integration tests (spec/) need from .pio/build/<env> to integration-builds/<env>:
# the merged image the simulator runs and its ELF, and the files the specs read (firmware.bin
# for OTA, partitions.bin).
set -euo pipefail

env="$1"
src=".pio/build/$env"
dst="integration-builds/$env"

mkdir -p "$dst"
cp "$src"/*.bin "$src/merged_firmware.elf" "$dst/"
ls -l "$dst"
