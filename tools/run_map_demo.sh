#!/usr/bin/env bash
# One command from a scenario to a top-down map picture with the estimated and true
# trajectories (sim/README.md "Map demo"). Desktop only when it records (ROS 2 Jazzy + Gazebo).
#
# Usage: tools/run_map_demo.sh <scenario.yaml> <out_dir> --bench-cmd "<cmd>" [--bag <dir>]
#          [--z-min-m Z] [--z-max-m Z] [--res-m R] [--render-engine ENGINE]
#   --bench-cmd: command that runs mss_bench (e.g. "env LD_LIBRARY_PATH=$HOME/opt/gtsam/lib
#     ~/mss-build/<dir>/bench/mss_bench"); default $MSS_BENCH_CMD.
#   --bag <dir>: an existing record_scenario.sh output (bag/ + gt.tum) of this scenario; skips
#     the recording.
#   --z-min-m / --z-max-m / --res-m: height cut (w frame, m) and cell size of the picture
#     (defaults: mss_eval.map_png).
#   e.g. tools/run_map_demo.sh sim/scenarios/E-01.yaml results/<run-id>/ --bench-cmd "..."
#
# Steps, each an existing tool:
#   1. tools/record_scenario.sh <scenario> <out>/record   (Gazebo, then the noise bridge)
#   2-3. tools/run_integration.py --bridged <bag dir> --tests E-01 --map-out
#        (bag_to_mssdata.py, then mss_bench ... --map-out <out>/run/E-01/map.pcd, then ATE)
#   4. python -m mss_eval map-png <map.pcd> --est est.tum --gt gt.tum --out <out>/map.png
#
# Writes only under <out_dir>: record/ (unless --bag), run/ (run_integration output), map.png,
# map_demo.json (scenario id, E-01 gate verdict, wall time per step, map-png and E-01 results)
# and logs/.
# The E-01 gate of run_integration compares against eval/baselines.json (synthetic data), so a
# Gazebo bag usually fails it on the ATE; that exit 1 is accepted and recorded in map_demo.json
# ("gate"). An exit 1 from a bench failure or timeout (bench_exit != 0) is an error here, as the
# map would be partial (tools/map_demo_verdict.py tells the two apart from integration.json).
# Exit: 0 picture written; 2 a step failed (see the log it names).
# Env: PYTHON (default python3; needs the pyproject.toml deps and matplotlib), MSS_BENCH_CMD,
#      and record_scenario.sh's own variables.
set -euo pipefail

usage() {
  echo "usage: $0 <scenario.yaml> <out_dir> --bench-cmd CMD [--bag DIR] [--z-min-m Z]" \
    "[--z-max-m Z] [--res-m R] [--render-engine ENGINE]" >&2
}
fail() { echo "run_map_demo: $*" >&2; exit 2; }

[ $# -ge 2 ] || { usage; exit 2; }
scenario="$1"
out="${2%/}"
shift 2
bench_cmd="${MSS_BENCH_CMD:-}"
bag=""
png_args=()
rec_args=()
while [ $# -gt 0 ]; do
  [ $# -ge 2 ] || { usage; exit 2; }
  case "$1" in
    --bench-cmd) bench_cmd="$2" ;;
    --bag) bag="${2%/}" ;;
    --z-min-m | --z-max-m | --res-m) png_args+=("$1" "$2") ;;
    --render-engine) rec_args+=("$1" "$2") ;;
    *) usage; exit 2 ;;
  esac
  shift 2
done

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONPATH="$repo/sim:$repo/eval:$repo/tools${PYTHONPATH:+:$PYTHONPATH}"
export MPLBACKEND=Agg

[ -f "$scenario" ] || fail "scenario not found: $scenario"
[ -n "$bench_cmd" ] || fail "no bench command: pass --bench-cmd or set MSS_BENCH_CMD"
if [ -n "$bag" ]; then
  [ -d "$bag/bag" ] && [ -f "$bag/gt.tum" ] || fail "--bag $bag: needs bag/ and gt.tum"
  bag="$(cd "$bag" && pwd)"
fi
[ -e "$out/run" ] && fail "$out/run exists; use a new output directory"
mkdir -p "$out/logs"
out="$(cd "$out" && pwd)"
scenario="$(cd "$(dirname "$scenario")" && pwd)/$(basename "$scenario")"
sc_id="$(basename "$scenario" .yaml)"

now() { date +%s.%N; }
since() { "$PYTHON" -c "import sys; print(round($(now) - float(sys.argv[1]), 1))" "$1"; }

# 1. Record (Gazebo + noise bridge), unless a recording was given.
record_s=null
record_status=null
if [ -z "$bag" ]; then
  t0=$(now)
  set +e
  "$repo/tools/record_scenario.sh" "$scenario" "$out/record" "${rec_args[@]}" \
    2>&1 | tee "$out/logs/record.log"
  record_status=${PIPESTATUS[0]}
  set -e
  record_s=$(since "$t0")
  # 1 = the noise report failed: the bag exists, the map can still be drawn (reported below).
  [ "$record_status" -le 1 ] || fail "recording failed (exit $record_status), see $out/logs/record.log"
  [ "$record_status" -eq 0 ] || echo "run_map_demo: WARNING: noise report failed" >&2
  bag="$out/record"
fi

# 2-3. Convert and replay with the map export (the --bridged chain of run_integration.py).
t0=$(now)
set +e
"$PYTHON" "$repo/tools/run_integration.py" --bridged "$bag" --scenario "$scenario" \
  --tests E-01 --map-out --bench-cmd "$bench_cmd" --out "$out/run" > "$out/logs/run.log" 2>&1
run_status=$?
set -e
run_s=$(since "$t0")
cat "$out/logs/run.log"
# run_integration.py exits 1 for a failed gate, which on a Gazebo bag is expected (synthetic
# baselines), but also for a failed or timed-out bench (partial map): tell them apart.
verdict_json="$("$PYTHON" "$repo/tools/map_demo_verdict.py" "$out/run/integration.json" \
  "$run_status" "$sc_id" 2> "$out/logs/verdict.err")" \
  || fail "run_integration.py result not usable: $(cat "$out/logs/verdict.err"), see $out/logs/run.log"
[ -f "$out/run/E-01/map.pcd" ] || fail "no map.pcd, see $out/run/E-01/bench.log"

# 4. Picture.
t0=$(now)
"$PYTHON" -m mss_eval map-png "$out/run/E-01/map.pcd" --est "$out/run/E-01/est.tum" \
  --gt "$bag/gt.tum" --out "$out/map.png" --title "$sc_id" "${png_args[@]}" \
  > "$out/logs/map_png.json" 2> "$out/logs/map_png.err" \
  || fail "map-png failed: $(cat "$out/logs/map_png.err")"
png_s=$(since "$t0")

"$PYTHON" - "$out" "$bag" "$record_s" "$record_status" "$run_s" "$run_status" "$png_s" \
  "$verdict_json" <<'EOF'
import json, re, sys
from pathlib import Path
out, bag = Path(sys.argv[1]), sys.argv[2]
record_s, record_status, run_s, run_status, png_s = (json.loads(a) for a in sys.argv[3:8])
verdict = json.loads(sys.argv[8])
e01 = json.loads((out / "run/integration.json").read_text())["E-01"]
log = (out / "run/E-01/bench.log").read_text(errors="replace")
m = re.search(r"^map: .*$", log, re.M)
doc = {
    "scenario": verdict["scenario"],
    "bag": bag,
    "gate": {k: verdict[k] for k in ("gate_passed", "gate_note", "reason")},
    "wall_s": {"record": record_s, "convert_and_bench": run_s, "map_png": png_s},
    "record_exit": record_status,
    "run_integration_exit": run_status,
    "e01": {k: e01.get(k) for k in ("ate_rmse_m", "n_pairs", "p50_latency_ms",
                                    "p95_latency_ms", "bench_exit", "bench")},
    "bench_map_line": m.group(0) if m else None,
    "map_png": json.loads((out / "logs/map_png.json").read_text()),
}
(out / "map_demo.json").write_text(json.dumps(doc, indent=2) + "\n")
print(json.dumps(doc["map_png"]))
EOF
echo "run_map_demo: OK ($out/map.png)"
