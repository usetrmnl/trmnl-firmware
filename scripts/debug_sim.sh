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
# .pio/sim-debug/gdb run the toolchain GDB for its chip. The simulator is the latest trmnl-sim
# release from GitHub (or the one $TRMNL_SIM_VERSION names, e.g. v0.3.1), downloaded once into
# .pio/sim-debug/trmnl-sim; the GDB port is 3333 unless $TRMNL_SIM_GDB_PORT says otherwise.
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

# The simulator binary for this machine from a trmnl-sim release, downloading it if it isn't cached.
sim_repo=https://github.com/usetrmnl/trmnl-sim
sim_cache=$state/trmnl-sim
fetch_sim() {
  local tag platform ext dir bin tmp sums
  case $(uname -s)-$(uname -m) in
    Darwin-arm64) platform=macos-arm64 ext=zip ;;
    Linux-x86_64) platform=linux-x86_64 ext=tar.gz ;;
    *) die "no trmnl-sim release for $(uname -s) $(uname -m)" ;;
  esac
  tag=${TRMNL_SIM_VERSION:-}
  if [[ -z "$tag" ]]; then
    # releases/latest redirects to releases/tag/<tag>; offline, use the newest one cached.
    tag=$(curl -fsSI "$sim_repo/releases/latest" 2>/dev/null |
      sed -n 's|^[Ll]ocation: .*/releases/tag/\([^[:space:]]*\).*|\1|p') || true
    [[ -n "$tag" ]] || tag=$(ls -t "$sim_cache" 2>/dev/null | head -1)
    [[ -n "$tag" ]] || die "can't find the latest trmnl-sim release (offline?)"
  fi
  dir=$sim_cache/$tag
  case $platform in
    macos-*) bin="$dir/TRMNL Simulator.app/Contents/MacOS/trmnl-sim" ;;
    *) bin=$dir/trmnl-sim ;;
  esac
  if [[ ! -x "$bin" ]]; then
    local name=trmnl-sim-$tag-$platform
    say "downloading trmnl-sim $tag" >&2
    tmp=$(mktemp -d "${TMPDIR:-/tmp}/trmnl-sim.XXXXXX")
    trap "rm -rf '$tmp'" EXIT
    curl -fL# -o "$tmp/$name.$ext" "$sim_repo/releases/download/$tag/$name.$ext" >&2 ||
      die "can't download $name.$ext"
    curl -fsSL -o "$tmp/SHA256SUMS.txt" "$sim_repo/releases/download/$tag/SHA256SUMS.txt" ||
      die "can't download SHA256SUMS.txt for $tag"
    sums=$(command -v sha256sum || echo "shasum -a 256")
    (cd "$tmp" && grep " $name.$ext\$" SHA256SUMS.txt | $sums -c - >/dev/null) ||
      die "$name.$ext doesn't match its SHA256SUMS.txt"
    case $ext in
      zip) unzip -q "$tmp/$name.$ext" -d "$tmp" ;;
      tar.gz) tar xzf "$tmp/$name.$ext" -C "$tmp" ;;
    esac
    rm -rf "$dir"
    mkdir -p "$sim_cache"
    mv "$tmp/$name" "$dir"
    rm -rf "$tmp"
    trap - EXIT
    [[ -x "$bin" ]] || die "no simulator binary at $bin in the $tag release"
  fi
  printf '%s\n' "$bin"
}

sim=$(fetch_sim)
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
# exec: the simulator keeps this pid.
exec "$sim" "$image" --gdb "$addr" --gdb-wait "$@"
