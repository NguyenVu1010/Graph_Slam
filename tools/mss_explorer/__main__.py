"""CLI: ``python -m mss_explorer run <spec.yaml> [--bench-cmd CMD] [--root DIR] [--timeout-s S]``
prints the result as one JSON line (exit 0 ok or lost, 1 failed or running, 2 spec error);
``python -m mss_explorer list [--root DIR]`` prints one JSON line per run.
The bench command defaults to the ``MSS_BENCH_CMD`` environment variable."""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict
from pathlib import Path

from mss_explorer.runner import RunResult, list_runs, run_experiment
from mss_explorer.spec import REPO, SpecError, load_spec


def _line(r: RunResult, label: str = "") -> str:
    return json.dumps({**asdict(r), "run_dir": str(r.run_dir), "label": label})


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="mss_explorer", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run")
    run.add_argument("spec", type=Path)
    run.add_argument("--bench-cmd", default=os.environ.get("MSS_BENCH_CMD", ""))
    run.add_argument("--timeout-s", type=float, default=None)
    lst = sub.add_parser("list")
    for p in (run, lst):
        p.add_argument("--root", type=Path, default=REPO / "results/explorer")
    a = ap.parse_args(argv)
    if a.cmd == "list":
        for spec, r in list_runs(a.root):
            print(_line(r, spec.label))
        return 0
    if not a.bench_cmd:
        print("mss_explorer: no bench command (--bench-cmd or MSS_BENCH_CMD)", file=sys.stderr)
        return 2
    try:
        spec = load_spec(a.spec)
        r = run_experiment(spec, bench_cmd=a.bench_cmd, root=a.root, timeout_s=a.timeout_s)
    except SpecError as e:
        print(f"mss_explorer: {e}", file=sys.stderr)
        return 2
    print(_line(r, spec.label))
    return 0 if r.status in ("ok", "lost") else 1


if __name__ == "__main__":
    sys.exit(main())
