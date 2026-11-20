#!/usr/bin/env bash
# Print the path of the simulator (trmnl-sim) binary for this machine: the latest release from
# GitHub (or the one $TRMNL_SIM_VERSION names, e.g. v0.3.1), downloaded once into
# .pio/sim-debug/trmnl-sim. Offline, the newest one cached. sim.sh and debug_sim.sh run it.
set -euo pipefail
cd "$(dirname "$0")/.."

say() { printf '\033[1m==> %s\033[0m\n' "$*" >&2; }
die() { printf 'fetch_sim.sh: %s\n' "$*" >&2; exit 1; }

sim_repo=https://github.com/usetrmnl/trmnl-sim
sim_cache=.pio/sim-debug/trmnl-sim

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
  name=trmnl-sim-$tag-$platform
  say "downloading trmnl-sim $tag"
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
  [[ -x "$bin" ]] || die "no simulator binary at $bin in the $tag release"
fi
printf '%s\n' "$PWD/$bin"
