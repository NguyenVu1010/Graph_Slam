#!/usr/bin/env bash
# Configure, build and test the repo inside WSL.
# Usage: build.sh [--no-gtsam] [--asan] [--build-dir DIR] [--target T]
set -euo pipefail

usage() { echo "Usage: $0 [--no-gtsam] [--asan] [--build-dir DIR] [--target T]" >&2; }

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
with_gtsam=ON
asan=0
build_dir=""
target=""
jobs=3 # WSL has ~10 GB RAM

while [ $# -gt 0 ]; do
  case "$1" in
    --no-gtsam) with_gtsam=OFF ;;
    --asan) asan=1 ;;
    --build-dir)
      [ $# -ge 2 ] || { usage; exit 2; }
      build_dir="$2"
      shift
      ;;
    --target)
      [ $# -ge 2 ] || { usage; exit 2; }
      target="$2"
      shift
      ;;
    -h | --help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if [ -z "$build_dir" ]; then
  branch="$(git -C "$repo" rev-parse --abbrev-ref HEAD 2>/dev/null || echo detached)"
  build_dir="$HOME/mss-build/${branch//\//-}"
  if [ "$with_gtsam" = OFF ]; then build_dir="${build_dir}-nogtsam"; fi
  if [ "$asan" = 1 ]; then build_dir="${build_dir}-asan"; fi
fi

args=(-S "$repo" -B "$build_dir" "-DMSS_WITH_GTSAM=$with_gtsam" -DMSS_WERROR=ON
  "-DCMAKE_PREFIX_PATH=$HOME/opt/gtsam;$HOME/opt/sophus")
if [ "$asan" = 1 ]; then
  args+=(-DCMAKE_BUILD_TYPE=Debug "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined")
else
  args+=(-DCMAKE_BUILD_TYPE=Release)
fi
if command -v ninja >/dev/null 2>&1; then args+=(-G Ninja); fi
if command -v ccache >/dev/null 2>&1; then args+=(-DCMAKE_CXX_COMPILER_LAUNCHER=ccache); fi

cmake "${args[@]}"
if [ -n "$target" ]; then
  cmake --build "$build_dir" --parallel "$jobs" --target "$target"
  ctest --test-dir "$build_dir" --output-on-failure -R "$target"
else
  cmake --build "$build_dir" --parallel "$jobs"
  ctest --test-dir "$build_dir" --output-on-failure
fi
echo "BUILD OK: $build_dir"
