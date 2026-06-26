#!/usr/bin/env bash
# Regenerate committed per-target IDF component lockfiles.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

mkdir -p lockfiles

update_lock() {
  local env="$1"
  local target="$2"
  echo "==> Updating lockfiles/dependencies.${target}.lock (${env})"
  pio pkg install -e "$env"
  pio run -e "$env"
  python3 scripts/ci/normalize_lock.py dependencies.lock > "lockfiles/dependencies.${target}.lock"
}

update_lock TRMNL_X_dev esp32s3
update_lock trmnl esp32c3

echo "Done. Review lockfiles/ and commit if changes are intentional."
