#!/usr/bin/env bash
# Calibrate gpu_lidar's own range error along a scenario's trajectory (ADR-011 note, deviation 1).
#
# Usage: tools/calibrate_lidar_render.sh <scenario.yaml> <out_dir> [--render-engine ENGINE]
#   e.g. tools/calibrate_lidar_render.sh sim/scenarios/E-01.yaml results/<run-id>/
#
# Records the scenario once with the Gazebo LiDAR noise at 0 (record_scenario.sh
# --lidar-gz-sigma 0; same scenario, seed, robot and driver), then measures the render table on
# the RAW bag (tools/calibrate_lidar_render.py) into <out_dir>/render_<world>.yaml. The noise
# report of that recording is expected to fail (no declared noise in Gazebo) and is ignored.
# Install the table by copying it to sim/noise/render/<world>.yaml (the printed command): the
# script never writes into the source tree.
# Exit: 0 ok, 2 recording or measurement failed. Env as record_scenario.sh.
set -euo pipefail

[ $# -ge 2 ] || { echo "usage: $0 <scenario.yaml> <out_dir> [--render-engine ENGINE]" >&2; exit 2; }
scenario="$1"
out="${2%/}"
shift 2
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"

set +e
"$repo/tools/record_scenario.sh" "$scenario" "$out" --lidar-gz-sigma 0 "$@"
status=$?
set -e
if [ "$status" -eq 2 ]; then
  echo "calibrate_lidar_render: recording failed" >&2
  exit 2
fi

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
set +u
# shellcheck disable=SC1090
. "$ROS_SETUP"
set -u
export PYTHONPATH="$repo/sim:$repo/eval:$repo/tools${PYTHONPATH:+:$PYTHONPATH}"
world="$("$PYTHON" -c "import sys; from pathlib import Path; from mss_sim.scenario import \
load_scenario; print(load_scenario(Path(sys.argv[1])).world)" "$scenario")"
gz_version="$(gz sim --version 2>/dev/null | head -1 || echo unknown)"
render_engine=ogre2
if [ "${1:-}" = "--render-engine" ] && [ -n "${2:-}" ]; then render_engine="$2"; fi
"$PYTHON" "$repo/tools/calibrate_lidar_render.py" "$out/raw_bag" "$scenario" \
  --out "$out/render_$world.yaml" --gazebo "$gz_version" --render-engine "$render_engine" \
  || { echo "calibrate_lidar_render: measurement failed" >&2; exit 2; }
echo "install: cp $out/render_$world.yaml $repo/sim/noise/render/$world.yaml"
