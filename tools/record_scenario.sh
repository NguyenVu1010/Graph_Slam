#!/usr/bin/env bash
# Record one scenario in Gazebo Harmonic and turn it into the bag the pipeline reads
# (ADR-011 sections 1, 6 and 7). Desktop only (needs ROS 2 Jazzy + Gazebo; never in CI).
#
# Usage: tools/record_scenario.sh <scenario.yaml> <out_dir> [--render-engine ogre2|ogre] [--gui]
#          [--no-driver] [--lidar-gz-sigma S]
#   --no-driver: the robot stays at its spawn pose (static calibration recordings).
#   --lidar-gz-sigma S: Gazebo LiDAR range noise S m instead of the profile's (render
#   calibration: 0; sim/README.md "LiDAR render error").
#   e.g. tools/record_scenario.sh sim/scenarios/E-01.yaml results/<run-id>/
#
# Steps: gen_sim_assets.py -> ros2 bag record (raw topics, MCAP, zstd file compression,
# 100 MB splits, sim time) -> ros2 launch sim/launch/sim.launch.py paused -> unpause once the
# recorder has subscribed and the robot is spawned -> stop at duration_s + SIM_MARGIN_S of sim
# time -> offline noise bridge -> gt-to-tum -> noise report with the world's planes sidecar.
#
# Writes only under <out_dir>: assets/, raw_bag/ (raw, clean), bag/ + gt.tum +
# bridge_report.json (bridged), raw_gt.tum, noise_report/, record_summary.json, logs/.
# Exit: 0 ok, 1 noise report failed, 2 setup, launch, recording or bridge failed.
# Env: PYTHON (default python3, needs rosbags, shapely, scipy, jinja2, allantools, matplotlib),
#      ROS_SETUP (default /opt/ros/jazzy/setup.bash), SIM_MARGIN_S (default 2),
#      WALL_TIMEOUT_FACTOR (default 20: give up after this x duration_s of wall time).
set -euo pipefail

usage() {
  echo "usage: $0 <scenario.yaml> <out_dir> [--render-engine ENGINE] [--gui] [--no-driver]" \
    "[--lidar-gz-sigma S]" >&2
}
[ $# -ge 2 ] || { usage; exit 2; }
scenario="$1"
out="${2%/}"
shift 2
render_engine=ogre2
headless=true
driver=true
gen_extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --render-engine) render_engine="$2"; shift 2 ;;
    --gui) headless=false; shift ;;
    --no-driver) driver=false; shift ;;
    --lidar-gz-sigma) gen_extra=(--lidar-gz-sigma "$2"); shift 2 ;;
    *) usage; exit 2 ;;
  esac
done

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
SIM_MARGIN_S="${SIM_MARGIN_S:-2}"
WALL_TIMEOUT_FACTOR="${WALL_TIMEOUT_FACTOR:-20}"
STARTUP_TIMEOUT_S=90   # wall time for the recorder to subscribe and the robot to spawn
STOP_TIMEOUT_S=120     # wall time for the recorder to flush and compress its last file
POLL_S=2               # sim-clock poll period, wall s
# Raw topics (ADR-011 section 6 raw part, plus /joint_states for the encoders).
RAW_TOPICS=(/lidar/points /imu/data /gnss/fix /ground_truth /tf_static /clock /joint_states)

fail() { echo "record_scenario: $*" >&2; exit 2; }

[ -f "$scenario" ] || fail "scenario not found: $scenario"
[ -e "$out/raw_bag" ] && fail "$out/raw_bag exists; use a new run directory"
mkdir -p "$out/logs"
out="$(cd "$out" && pwd)"
scenario="$(cd "$(dirname "$scenario")" && pwd)/$(basename "$scenario")"
logs="$out/logs"

# ROS setup scripts read unset variables.
set +u
# shellcheck disable=SC1090
. "$ROS_SETUP"
set -u
export PYTHONPATH="$repo/sim:$repo/eval:$repo/tools${PYTHONPATH:+:$PYTHONPATH}"
export RCUTILS_COLORIZED_OUTPUT=0

# Scenario fields (validated by mss_sim.scenario).
eval "$("$PYTHON" - "$scenario" <<'EOF'
import shlex, sys
from pathlib import Path
from mss_sim.scenario import load_scenario
sc = load_scenario(Path(sys.argv[1]))
for k, v in {"world": sc.world, "profile": sc.profile, "pipeline_config": sc.pipeline_config,
             "duration_s": sc.duration_s, "seed": sc.seed}.items():
    print(f"sc_{k}={shlex.quote(str(v))}")
EOF
)" || fail "cannot load scenario $scenario"
world_sdf="$repo/sim/worlds/$sc_world.sdf"
planes="$repo/sim/worlds/$sc_world.planes.yaml"
[ -f "$world_sdf" ] && [ -f "$planes" ] || fail "world $sc_world: missing $world_sdf or $planes"
stop_sim_s="$("$PYTHON" -c "import math; print(math.ceil($sc_duration_s + $SIM_MARGIN_S))")"
wall_timeout_s="$("$PYTHON" -c "import math; print(math.ceil($sc_duration_s * $WALL_TIMEOUT_FACTOR))")"

echo "record_scenario: $(basename "$scenario") world=$sc_world duration=${sc_duration_s}s seed=$sc_seed engine=$render_engine"
"$PYTHON" "$repo/tools/gen_sim_assets.py" --profile "$sc_profile" \
  --pipeline-config "$sc_pipeline_config" --out "$out/assets" "${gen_extra[@]}" \
  > "$logs/gen_assets.log" 2>&1 \
  || fail "gen_sim_assets.py failed, see $logs/gen_assets.log"

rec_pid=""
launch_pid=""
# stop_group PID TIMEOUT_S: SIGTERM the process group, SIGKILL after TIMEOUT_S. Not SIGINT:
# background jobs of a non-interactive shell start with SIGINT ignored.
stop_group() {
  local pid="$1" timeout_s="$2" i=0
  [ -n "$pid" ] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || return 0
  while kill -0 -- "-$pid" 2>/dev/null && [ "$i" -lt "$timeout_s" ]; do sleep 1; i=$((i + 1)); done
  kill -KILL -- "-$pid" 2>/dev/null || true
}
cleanup() {
  stop_group "$rec_pid" "$STOP_TIMEOUT_S"
  stop_group "$launch_pid" 20
  # Backstop: a gz server or bridge that left the launch process group.
  pkill -KILL -f "gz sim.*$world_sdf" 2>/dev/null || true
  pkill -KILL -f "parameter_bridge.*$out/assets" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 2' INT TERM

# 1. Recorder first, so nothing published after the unpause is lost.
setsid ros2 bag record -s mcap --compression-mode file --compression-format zstd \
  --max-bag-size 100000000 --use-sim-time -o "$out/raw_bag" "${RAW_TOPICS[@]}" \
  > "$logs/record.log" 2>&1 &
rec_pid=$!

# 2. Gazebo paused, robot, bridge, /tf_static and driver.
setsid ros2 launch "$repo/sim/launch/sim.launch.py" "scenario:=$scenario" \
  "assets_dir:=$out/assets" "render_engine:=$render_engine" "headless:=$headless" \
  paused:=true "driver:=$driver" > "$logs/launch.log" 2>&1 &
launch_pid=$!

# 3. Wait for the spawned robot and every raw topic subscription, then unpause. The recorder
# (sim time) starts only on the first /clock, which a paused world does not publish: once the
# robot exists, advance the world by single 1 ms physics steps until the recorder subscribed.
t_start=$(date +%s)
n_steps=0
while :; do
  n_sub=$(grep -c "Subscribed to topic" "$logs/record.log" 2>/dev/null || true)
  spawned=$(grep -c "Entity creation successful" "$logs/launch.log" 2>/dev/null || true)
  n_sub="${n_sub:-0}"
  spawned="${spawned:-0}"
  [ "$n_sub" -ge "${#RAW_TOPICS[@]}" ] && [ "$spawned" -ge 1 ] && break
  if [ "$spawned" -ge 1 ]; then
    gz service -s "/world/$sc_world/control" --reqtype gz.msgs.WorldControl \
      --reptype gz.msgs.Boolean --timeout 2000 --req 'multi_step: 1' >> "$logs/unpause.log" 2>&1 \
      || true
    n_steps=$((n_steps + 1))
  fi
  kill -0 "$rec_pid" 2>/dev/null || fail "recorder exited early, see $logs/record.log"
  kill -0 "$launch_pid" 2>/dev/null || fail "launch exited early, see $logs/launch.log"
  [ $(($(date +%s) - t_start)) -lt "$STARTUP_TIMEOUT_S" ] \
    || fail "startup timeout: $n_sub/${#RAW_TOPICS[@]} topics subscribed, spawned=$spawned"
  sleep 1
done
gz service -s "/world/$sc_world/control" --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' >> "$logs/unpause.log" 2>&1 \
  || fail "unpause failed, see $logs/unpause.log"
wall_t0=$(date +%s.%N)
echo "record_scenario: ${n_steps} start-up steps; running until sim t = ${stop_sim_s} s"

# 4. Poll the sim clock (gz.msgs.Clock: the first 'sec' after 'sim {'; absent while 0).
sim_s=0
while [ "$sim_s" -lt "$stop_sim_s" ]; do
  sleep "$POLL_S"
  sim_s=$(timeout 10 gz topic -e -t /clock -n 1 2>/dev/null \
    | awk '/^sim/{f=1} f && /sec:/{print $2; exit}' || true)
  sim_s="${sim_s:-0}"
  kill -0 "$launch_pid" 2>/dev/null || fail "launch exited during the run, see $logs/launch.log"
  kill -0 "$rec_pid" 2>/dev/null || fail "recorder exited during the run, see $logs/record.log"
  wall_s=$("$PYTHON" -c "import time; print(int(time.time() - $wall_t0))")
  [ "$wall_s" -lt "$wall_timeout_s" ] || fail "wall timeout ${wall_timeout_s}s at sim t = ${sim_s}s"
done
wall_s=$("$PYTHON" -c "import time; print(round(time.time() - $wall_t0, 1))")

# 5. Pause the world first (the recorder may take a while to flush and compress, and the sim
# would keep running meanwhile), then stop the recorder, then the simulation.
gz service -s "/world/$sc_world/control" --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: true' >> "$logs/unpause.log" 2>&1 \
  || fail "pause failed, see $logs/unpause.log"
stop_group "$rec_pid" "$STOP_TIMEOUT_S"
rec_pid=""
stop_group "$launch_pid" 20
launch_pid=""
ros2 bag info "$out/raw_bag" > "$logs/bag_info.txt" 2>&1 || fail "unreadable raw bag, see $logs/bag_info.txt"
echo "record_scenario: recorded ${sim_s}s of sim time in ${wall_s}s of wall time"

# 6. Offline noise bridge, ground truth, noise report.
"$PYTHON" -m mss_sim bridge "$out/raw_bag" "$scenario" "$out" > "$logs/bridge.log" 2>&1 \
  || fail "bridge failed, see $logs/bridge.log"
"$PYTHON" -m mss_sim gt-to-tum "$out/raw_bag" "$out/raw_gt.tum" > "$logs/gt_to_tum.log" 2>&1 \
  || fail "gt-to-tum failed, see $logs/gt_to_tum.log"
"$PYTHON" - "$out" "$sim_s" "$wall_s" "$render_engine" <<'EOF' || fail "summary failed"
import hashlib, json, sys
from pathlib import Path
out, sim_s, wall_s, engine = Path(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
files = []
for f in sorted((out / "raw_bag").iterdir()):
    files.append({"name": f.name, "bytes": f.stat().st_size,
                  "sha256": hashlib.sha256(f.read_bytes()).hexdigest()})
summary = {"sim_s": sim_s, "wall_s": wall_s, "real_time_factor": round(sim_s / wall_s, 3),
           "render_engine": engine, "raw_bag_files": files,
           "raw_bag_bytes": sum(f["bytes"] for f in files)}
(out / "record_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps({k: v for k, v in summary.items() if k != "raw_bag_files"}))
EOF

# The world's measured LiDAR render table (tools/calibrate_lidar_render.sh), when there is one.
render_table="$repo/sim/noise/render/$sc_world.yaml"
render_args=()
[ -f "$render_table" ] && render_args=(--render-table "$render_table")
set +e
"$PYTHON" -m mss_eval.noise_report "$out" "$sc_profile" --out "$out/noise_report" \
  --planes "$planes" --pipeline-config "$sc_pipeline_config" "${render_args[@]}" \
  > "$logs/noise_report.log" 2>&1
report_status=$?
set -e
cat "$logs/noise_report.log"
if [ "$report_status" -ne 0 ]; then
  [ -f "$out/noise_report/noise_report.json" ] || fail "noise report crashed, see $logs/noise_report.log"
  echo "record_scenario: NOISE REPORT FAILED ($out/noise_report/noise_report.html)" >&2
  exit 1
fi
echo "record_scenario: OK ($out)"
