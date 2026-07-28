#!/usr/bin/env bash
# Run clang-format on tracked C/C++ sources. Requires clang-format 22+ and repo .clang-format.
# Exclusions (vendor dirs, large open-PR files, generated constants) live in .clang-format-ignore.

set -euo pipefail

cd "$(dirname "$0")/.."

CF="${CLANG_FORMAT:-clang-format}"

if ! command -v "$CF" &> /dev/null; then
  echo "Error: clang-format not found. Please install clang-format 22+ or set the CLANG_FORMAT environment variable."
  exit 1
fi

# Find files needing changes (clang-format honors .clang-format-ignore); dry-run
# emits a "code should be clang-formatted" warning per offending file.
NEED=$(
  git ls-files -z '*.cpp' '*.h' '*.c' '*.hpp' '*.ino' \
    | xargs -0 "$CF" -style=file --dry-run 2>&1 \
    | sed -n 's/:[0-9]*:[0-9]*: warning: code should be clang-formatted.*//p' \
    | sort -u
)

if [ -z "$NEED" ]; then
  echo "All files already formatted."
  exit 0
fi

printf '%s\n' "$NEED" | xargs "$CF" -style=file -i --verbose
echo "Formatted $(printf '%s\n' "$NEED" | wc -l | tr -d ' ') file(s)."
