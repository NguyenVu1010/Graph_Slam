"""Decides whether a run_integration.py result may be used for the map demo (A1, A2).

tools/run_map_demo.sh draws a map from the E-01 bench run. run_integration.py exits 1 both when
the E-01 gate fails on the ATE (expected on a Gazebo bag: eval/baselines.json holds synthetic
baselines) and when the bench failed or timed out (the map is then partial). Only the first is
acceptable. This module tells them apart from ``integration.json`` and writes the verdict that
run_map_demo.sh stores in ``map_demo.json``.

Usage: map_demo_verdict.py <integration.json> <run_integration exit code> <scenario id>
Prints one JSON object (``scenario``, ``usable``, ``gate_passed`` = the E-01 gate only,
``gate_note``, ``pf01_passed``, ``pf01_note``, ``reason``).
Exit: 0 usable; 1 not usable (the reason is in the JSON and on stderr).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

_BENCH_OK = 0  # mss_bench exit code of a complete run


def judge_run(integration: dict[str, Any] | None, run_status: int, scenario: str) -> dict[str, Any]:
    """Verdict for the E-01 result of one run_integration.py run.

    ``integration`` is the parsed integration.json (None when missing or unreadable) and
    ``run_status`` the exit code of run_integration.py. Usable: exit 0 (gate passed), or exit 1
    with a complete bench run (exit 0) and an ATE, so only the baseline comparison failed.
    """
    e01 = None if integration is None else integration.get("E-01")
    verdict: dict[str, Any] = {
        "scenario": scenario,
        "usable": False,
        "gate_passed": None,
        "gate_note": None if e01 is None else e01.get("note"),
        "pf01_passed": None,
        "pf01_note": None,
        "reason": "",
    }
    if run_status not in (0, 1):
        verdict["reason"] = f"run_integration.py failed (exit {run_status})"
        return verdict
    if e01 is None:
        verdict["reason"] = "integration.json has no E-01 result"
        return verdict
    # gate_passed is the E-01 ATE gate only; other checks (PF-01 latency) are recorded apart.
    verdict["gate_passed"] = bool(e01.get("passed"))
    pf01 = integration.get("PF-01") if integration is not None else None
    verdict["pf01_passed"] = None if pf01 is None else bool(pf01.get("passed"))
    verdict["pf01_note"] = None if pf01 is None else pf01.get("note")
    bench_exit = e01.get("bench_exit")
    if bench_exit != _BENCH_OK:
        verdict["reason"] = f"E-01 bench did not finish cleanly (bench_exit {bench_exit})"
        return verdict
    if e01.get("ate_rmse_m") is None:
        verdict["reason"] = "E-01 has no ATE (no usable trajectory)"
        return verdict
    verdict["usable"] = True
    if run_status == 1 and not verdict["gate_passed"]:
        verdict["reason"] = "E-01 gate failed on the ATE comparison (bench ran to the end)"
    elif run_status == 1:
        verdict["reason"] = "E-01 gate passed; another check failed (PF-01 or a later test)"
    return verdict


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    path, status, scenario = Path(argv[0]), int(argv[1]), argv[2]
    try:
        integration = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        integration = None
    verdict = judge_run(integration, status, scenario)
    print(json.dumps(verdict))
    if not verdict["usable"]:
        print(f"map_demo_verdict: {verdict['reason']}", file=sys.stderr)
    return 0 if verdict["usable"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
