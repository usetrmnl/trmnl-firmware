#!/usr/bin/env bash
# Build a PlatformIO env, then run its merged image in the simulator (trmnl-sim) held at reset
# until a debugger attaches on 127.0.0.1:3333. VS Code's "Debug in simulator"
# (trmnl.code-workspace) runs this for PlatformIO's selected env, then attaches.
#
#   scripts/debug_sim.sh trmnl                 # pio run -e trmnl, then the simulator, waiting for GDB
#   scripts/debug_sim.sh TRMNL_X --erase       # more arguments go to the simulator
#   scripts/debug_sim.sh --no-build trmnl      # run the last build as it is
#   scripts/debug_sim.sh --stop                # stop the simulator it started
#
# For the debugger it links .pio/sim-debug/firmware.elf to the image's ELF and makes
# .pio/sim-debug/gdb run the toolchain GDB for its chip. The simulator checkout is ../trmnl-sim
# unless $TRMNL_SIM says otherwise; the GDB port is 3333 unless $TRMNL_SIM_GDB_PORT does.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
  awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
  exit 0
fi

say() { printf '\033[1m==> %s\033[0m\n' "$*"; }
die() { printf 'debug_sim.sh: %s\n' "$*" >&2; exit 1; }

state=.pio/sim-debug
pidfile=$state/sim.pid
addr=127.0.0.1:${TRMNL_SIM_GDB_PORT:-3333}

# Stop the simulator a previous run started (it holds the GDB port).
stop() {
  local pid
  pid=$(cat "$pidfile" 2>/dev/null) || return 0
  if ps -p "$pid" -o command= 2>/dev/null | grep -q trmnl-sim; then
    kill "$pid" 2>/dev/null || true
    for _ in $(seq 50); do ps -p "$pid" >/dev/null 2>&1 || break; sleep 0.1; done
  fi
  rm -f "$pidfile"
}

if [[ "${1:-}" == --stop ]]; then
  stop
  exit 0
fi

build=1
if [[ "${1:-}" == --no-build ]]; then
  build=0
  shift
fi
env=${1:-}
[[ -n "$env" && "$env" != -* ]] || die "which PlatformIO env? (scripts/debug_sim.sh --help)"
shift

sim=${TRMNL_SIM:-../trmnl-sim}
[[ -x "$sim/bin/sim" ]] || die "no trmnl-sim checkout at $sim (set TRMNL_SIM)"
out=.pio/build/$env

stop
if ((build)); then
  pio=$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")
  say "building $env"
  "$pio" run -e "$env"
fi

# The newest merged image the post-build step wrote (not the OTA app image).
image=$(ls -t "$out"/FW*-"$env".bin 2>/dev/null | head -1) || true
[[ -n "$image" ]] || die "no merged image FW…-$env.bin in $out (this env has no merge step?)"
elf=${image%.bin}.elf
[[ -f "$elf" ]] || die "no ELF next to $image"

# The GDB for the ELF's machine: RISC-V (C3, C5) or Xtensa (S3).
case $(od -An -tu2 -j18 -N2 "$elf" | tr -d ' ') in
  243) gdb=$HOME/.platformio/packages/tool-riscv32-esp-elf-gdb/bin/riscv32-esp-elf-gdb ;;
  94) gdb=$HOME/.platformio/packages/tool-xtensa-esp-elf-gdb/bin/xtensa-esp32s3-elf-gdb ;;
  *) die "$elf is neither RISC-V nor Xtensa" ;;
esac
[[ -x "$gdb" ]] || die "no GDB at $gdb (PlatformIO installs it with the platform's debug tools)"

mkdir -p "$state"
ln -sf "$PWD/$elf" "$state/firmware.elf"
# A script, not a link: Espressif's GDB launcher picks its binary by the name it runs as.
rm -f "$state/gdb"
printf '#!/bin/sh\nexec "%s" "$@"\n' "$gdb" >"$state/gdb"
chmod +x "$state/gdb"
image=$PWD/$image
say "running $(basename "$image")"
echo $$ >"$pidfile"
# bin/sim builds the simulator, then execs it: it keeps this pid.
cd "$sim"
exec bin/sim "$image" --gdb "$addr" --gdb-wait "$@"
