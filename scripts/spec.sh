#!/usr/bin/env bash
# Build one PlatformIO env and run the integration specs (spec/) on it in trmnl-sim, all of them
# (ENVS=<env>:full).
#
#   scripts/spec.sh <env> [rspec args...]
#
#   scripts/spec.sh trmnl                                      # every spec, in parallel
#   scripts/spec.sh trmnl spec/spec/general/images_spec.rb:252  # one example
#   scripts/spec.sh TRMNL_X -e "jpeg"                           # examples matching a name
#
# With no arguments the whole suite runs in parallel (rake spec); with any, a single rspec runs
# them (parallel_rspec takes neither file:line nor -e). Paths can be relative to the current
# directory or to spec/. See spec/README.md for SIM_BIN and the other options.

set -euo pipefail

if [ $# -lt 1 ] || [ -z "$1" ]; then
  echo "Usage: $0 <env> [rspec args...]" >&2
  exit 2
fi

ENV_NAME="$1"
shift

# Paths that exist from the caller's directory are made absolute (rspec runs in spec/).
ARGS=()
for arg in "$@"; do
  path="${arg%%:*}"
  if [[ "$arg" != -* && -e "$path" ]]; then
    arg="$(cd "$(dirname "$path")" && pwd)/$(basename "$arg")"
  fi
  ARGS+=("$arg")
done

cd "$(dirname "$0")/.."

pio run -e "$ENV_NAME"

cd spec
bundle check > /dev/null || bundle install
export ENVS="$ENV_NAME:full"
if [ ${#ARGS[@]} -eq 0 ]; then
  exec bundle exec rake spec
fi
bundle exec rake sim:build
exec bundle exec rspec "${ARGS[@]}"
