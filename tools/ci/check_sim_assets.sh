#!/usr/bin/env bash
# CI (python job, no Gazebo; ADR-011 section 9): the Gazebo assets generate and parse.
# gen_sim_assets.py for every scenario, an XML parse of every world and generated robot, the
# world vs planes-sidecar check, and ruff. Writes only under results/ci-sim-assets/.
# Usage: tools/ci/check_sim_assets.sh   (from anywhere; needs the repo's Python deps)
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo"
PYTHON="${PYTHON:-python3}"
export PYTHONPATH="$repo/sim:$repo/eval:$repo/tools${PYTHONPATH:+:$PYTHONPATH}"
out="results/ci-sim-assets"
rm -rf "$out"

for scenario in sim/scenarios/*.yaml; do
  id="$(basename "$scenario" .yaml)"
  read -r profile overlay < <("$PYTHON" -c "
import sys
from pathlib import Path
from mss_sim.scenario import load_scenario
sc = load_scenario(Path(sys.argv[1]))
print(sc.profile, sc.pipeline_config)" "$scenario")
  "$PYTHON" tools/gen_sim_assets.py --profile "$profile" --pipeline-config "$overlay" \
    --out "$out/$id" > /dev/null
  echo "assets OK: $id"
done

for sdf in sim/worlds/*.sdf "$out"/*/robot.sdf; do
  "$PYTHON" -c "import sys, xml.etree.ElementTree as ET; ET.parse(sys.argv[1])" "$sdf"
  echo "XML OK: $sdf"
done
"$PYTHON" tools/gen_sim_assets.py --check-worlds
ruff check .
ruff format --check .
echo "check_sim_assets: OK"
