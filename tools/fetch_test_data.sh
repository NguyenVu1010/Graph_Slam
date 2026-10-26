#!/usr/bin/env bash
# Fetch the rosbags used by tests/integration into tests/data/ (git-ignored).
# Usage: tools/fetch_test_data.sh [dest-dir]
set -euo pipefail

DEST="${1:-$(cd "$(dirname "$0")/.." && pwd)/tests/data}"
mkdir -p "$DEST"

# Datasets are published as multi-file sets (a bag split by size into several files) described by
# a manifest; the manifest and its handling arrive with the Gazebo plan. For now the single-file
# entries below are a placeholder, no functional change.
# name  url  sha256 -- fill in when the first dataset is published.
DATASETS=(
  # "E-01_sim_loop.mcap https://example.invalid/E-01.mcap 0000"
)

if [ "${#DATASETS[@]}" -eq 0 ]; then
  echo "no datasets registered yet; nothing to fetch (dest: $DEST)"
  exit 0
fi

for entry in "${DATASETS[@]}"; do
  read -r name url sha <<<"$entry"
  if [ -f "$DEST/$name" ] && echo "$sha  $DEST/$name" | sha256sum -c --quiet; then
    echo "ok   $name"
    continue
  fi
  echo "get  $name"
  curl -fsSL "$url" -o "$DEST/$name"
  echo "$sha  $DEST/$name" | sha256sum -c --quiet
done
