#!/usr/bin/env bash
# Build a PlatformIO env, then run its merged image in the simulator (trmnl-sim). To debug it with
# GDB, use debug_sim.sh instead.
#
#   scripts/sim.sh trmnl                 # pio run -e trmnl, then the simulator
#   scripts/sim.sh TRMNL_X --erase       # more arguments go to the simulator (trmnl-sim --help)
#   scripts/sim.sh --no-build TRMNL_X    # run the last build as it is
#
# The simulator is the latest trmnl-sim release (or $TRMNL_SIM_VERSION's), from fetch_sim.sh.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
  awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
  exit 0
fi

say() { printf '\033[1m==> %s\033[0m\n' "$*"; }
die() { printf 'sim.sh: %s\n' "$*" >&2; exit 1; }

build=1
if [[ "${1:-}" == --no-build ]]; then
  build=0
  shift
fi
env=${1:-}
[[ -n "$env" && "$env" != -* ]] || die "which PlatformIO env? (scripts/sim.sh --help)"
shift

sim=$(scripts/fetch_sim.sh)
out=.pio/build/$env

if ((build)); then
  pio=$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")
  say "building $env"
  "$pio" run -e "$env"
fi

# The newest merged image the post-build step wrote (not the OTA app image).
image=$(ls -t "$out"/FW*-"$env".bin 2>/dev/null | head -1) || true
[[ -n "$image" ]] || die "no merged image FW…-$env.bin in $out (this env has no merge step?)"
[[ -f "${image%.bin}.elf" ]] || die "no ELF next to $image"

say "running $(basename "$image")"
exec "$sim" "$PWD/$image" "$@"
