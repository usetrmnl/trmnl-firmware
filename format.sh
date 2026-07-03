#!/usr/bin/env bash
# Run clang-format on tracked C/C++ sources. Requires clang-format 22+ and repo .clang-format.

cd "$(dirname "$0")"

CF="${CLANG_FORMAT:-/opt/homebrew/opt/llvm/bin/clang-format}"

# Modified files with large diffs in open PRs (>200 LOC) — skip to reduce merge conflicts.
EXCLUDE=(
  src/bl.cpp
  src/display.cpp
)

VENDOR_PREFIXES=(
  components/
  lib/BMA530_SensorAPI/
  lib/BQ27427/
  lib/IQS323/
  lib/trmnl_x/
)

CONST_FILES=(
  include/Inter_18.h
  include/Roboto_Black_24.h
  include/nicoclean_8.h
  src/battery_small.h
  src/loading.h
  src/logo_big.h
  src/logo_medium.h
  src/logo_small.h
  src/wifi_connect_qr.h
  src/wifi_failed_qr.h
)

is_excluded() {
  local f="$1"
  for e in "${EXCLUDE[@]}"; do [[ "$f" == "$e" ]] && return 0; done
  for p in "${VENDOR_PREFIXES[@]}"; do [[ "$f" == "$p"* ]] && return 0; done
  for v in "${CONST_FILES[@]}"; do [[ "$f" == "$v" ]] && return 0; done
  return 1
}

formatted=0
skipped=0
while IFS= read -r f; do
  if is_excluded "$f"; then
    ((skipped++)) || true
  else
    "$CF" -style=file -i "$f"
    ((formatted++)) || true
  fi
done < <(git ls-files '*.cpp' '*.h' '*.c' '*.hpp' '*.ino')

echo "Formatted: $formatted files"
echo "Skipped:   $skipped files"
echo ""
git diff --stat
