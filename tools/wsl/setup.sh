#!/usr/bin/env bash
# One-time dev setup inside WSL Ubuntu 24.04. Idempotent, needs no sudo.
# Checks apt packages (prints the install line), builds Sophus and GTSAM into $HOME/opt.
set -euo pipefail

SOPHUS_VERSION=1.22.10
GTSAM_VERSION=4.2.0
OPT="${HOME}/opt"
JOBS=3 # WSL has ~10 GB RAM

required_pkgs=(libeigen3-dev libgtest-dev libyaml-cpp-dev libtbb-dev libboost-all-dev)
optional_pkgs=(ninja-build ccache clang-format-18 clang-tidy-18)

missing_req=()
missing_opt=()
for p in "${required_pkgs[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || missing_req+=("$p")
done
for p in "${optional_pkgs[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || missing_opt+=("$p")
done

if [ ${#missing_req[@]} -gt 0 ]; then
  echo "ERROR: required apt packages missing: ${missing_req[*]}" >&2
  echo "Install with:" >&2
  echo "  sudo apt-get install -y ${missing_req[*]} ${missing_opt[*]}" >&2
  exit 1
fi
if [ ${#missing_opt[@]} -gt 0 ]; then
  echo "Optional apt packages missing (build still works): ${missing_opt[*]}"
  echo "Suggested: sudo apt-get install -y ${missing_opt[*]}"
fi

mkdir -p "$OPT"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

if [ -f "$OPT/sophus/share/sophus/cmake/SophusConfig.cmake" ]; then
  echo "Sophus already installed: $OPT/sophus"
else
  git clone --depth 1 --branch "$SOPHUS_VERSION" https://github.com/strasdat/Sophus.git "$work/sophus"
  cmake -S "$work/sophus" -B "$work/sophus/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$OPT/sophus" \
    -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DSOPHUS_USE_BASIC_LOGGING=ON
  cmake --build "$work/sophus/build" --parallel "$JOBS"
  cmake --install "$work/sophus/build"
fi

if [ -f "$OPT/gtsam/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
  echo "GTSAM already installed: $OPT/gtsam"
else
  git clone --depth 1 --branch "$GTSAM_VERSION" https://github.com/borglab/gtsam.git "$work/gtsam"
  # Same flags as docker/deps.Dockerfile.
  cmake -S "$work/gtsam" -B "$work/gtsam/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$OPT/gtsam" \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
  cmake --build "$work/gtsam/build" --parallel "$JOBS"
  cmake --install "$work/gtsam/build"
fi

echo "SETUP OK"
