#!/usr/bin/env bash
# Regenerate committed per-target IDF component lockfiles.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

LOCKFILES_DIR="scripts/ci/lockfiles"
mkdir -p "$LOCKFILES_DIR"

update_lock() {
  local env="$1"
  local target="$2"
  echo "==> Updating ${LOCKFILES_DIR}/dependencies.${target}.lock (${env})"
  pio pkg install -e "$env"
  pio run -e "$env"
  python3 scripts/ci/normalize_lock.py dependencies.lock > "${LOCKFILES_DIR}/dependencies.${target}.lock"
}

update_lock TRMNL_X_dev esp32s3
update_lock trmnl esp32c3

echo "Done. Review ${LOCKFILES_DIR}/ and commit if changes are intentional."
