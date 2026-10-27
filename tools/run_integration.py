"""Integration tests E-01..E-03 and PF-01 against eval/baselines.json (Test_CI_CD, ADR-009).

Usage:
  python tools/run_integration.py --bench-cmd "<cmd>" --out <dir>
      [--scenario sim/scenarios/E-01.yaml] [--baselines eval/baselines.json]
      [--tests E-01,E-02,E-03] [--duration-s S] [--plots] [--path-style native|wsl]

Chain: synthetic warehouse-lap bag (``mss_sim.synth``: 5 s still, 2 s speed ramp, ADR-011
extrinsics) -> noise bridge with the scenario -> ``bag_to_mssdata.py``; E-03 adds
``mutate_bag.py`` (5 % drops, 2 % reordering, seed 42) before the conversion. Each test replays
its dataset with ``<bench-cmd> config/default.yaml <mssdata> --overlay <overlay> --out <dir>``
and scores ``est.tum`` against the bridge's ``gt.tum`` with ``mss_eval.ate``.

The bench is an external command (it may be ``docker run ... mss_bench`` or ``wsl -- ...``), so
every path handed to it is absolute; ``--path-style wsl`` maps ``C:\\x`` to ``/mnt/c/x``.
Writes only under ``--out``: ``data/`` (bags, datasets), ``E-01/``, ``E-02/``, ``E-03/`` (bench
outputs, ``ate.json``, optional ``traj_xy.png``), ``integration.json`` and ``summary.md``
(also printed). Exit codes: 0 every gated check passes; 1 a gated check fails; 2 setup error
(missing scenario, baselines or bench command, data preparation failure).

Gates: E-01 bench exit 0, ATE from >= 10 pairs, ATE <= 1.2 x the E-01 baseline. E-02 a second
run on the E-01 dataset with a byte-identical ``est.tum``. E-03 bench exit 0 or 3 (3 = ran
with unprocessed records), ATE <= 1.2 x the E-01 baseline (its own baseline is reported only).
PF-01 per-scan p95 latency above 1.2 x baseline only warns. A ``null`` baseline passes with a
note that prints the measured value for the baseline-update PR.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

from bag_to_mssdata import convert
from mss_eval.ate import ate, plot_xy
from mss_sim.bridge import run_bridge
from mss_sim.config_overlay import get, load_pipeline_config
from mss_sim.profile import load_profile
from mss_sim.scenario import load_scenario
from mss_sim.synth import write_synthetic_raw_bag
from mutate_bag import mutate

_REPO = Path(__file__).resolve().parents[1]
_DEFAULT_YAML = _REPO / "config" / "default.yaml"
_TESTS = ("E-01", "E-02", "E-03")

# Data preparation (ADR-011 section 5; planner's values where the sources are silent).
_STATIC_S = 5.0  # every scenario starts with 5 s still (IMU initialisation)
_RAMP_S = 2.0  # smooth speed ramp after the still phase
_BOX_HALF_M = (8.0, 5.0, 2.0)  # floor and ceiling inside the +-15 deg beams
_E03_DROP, _E03_REORDER, _E03_WINDOW_NS, _E03_SEED = 0.05, 0.02, 20_000_000, 42

# Judging (ADR-009, CLAUDE.md): more than 20 % worse than the baseline fails.
_REGRESSION_FACTOR = 1.2
_MIN_PAIRS = 10
_EXIT_OK, _EXIT_INCOMPLETE = 0, 3  # mss_bench exit codes
_E03_BENCH_OK = (_EXIT_OK, _EXIT_INCOMPLETE)
_SHELL_NOT_FOUND = (126, 127)  # shell / wsl / docker: command not executable / not found
REORDER_NOTE = "reordering is re-sorted by bag_to_mssdata.py; only drops reach the bench"
EXIT_PASS, EXIT_FAIL, EXIT_SETUP = 0, 1, 2


class SetupError(RuntimeError):
    """The harness cannot run (not a test failure): exit code 2."""


@dataclass(frozen=True)
class Prepared:
    mssdata: Path
    mssdata_e03: Path
    gt_tum: Path
    overlay: Path


@dataclass(frozen=True)
class Verdict:
    passed: bool
    note: str


def prepare(scenario: Path, out: Path, duration_s: float | None) -> Prepared:
    """Synthesise, bridge and convert the scenario's bag under ``out / "data"``."""
    sc = load_scenario(scenario)
    if duration_s is not None:
        sc = replace(sc, duration_s=duration_s)
    cfg = load_pipeline_config(_DEFAULT_YAML, sc.pipeline_config)
    tr = [float(v) for v in get(cfg, "frames.T_b_l.translation_m")]
    q = [float(v) for v in get(cfg, "frames.T_b_l.quaternion_wxyz")]  # Hamilton (w, x, y, z)
    data = out / "data"
    data.mkdir(parents=True, exist_ok=True)
    write_synthetic_raw_bag(
        data,
        duration_s=sc.duration_s,
        seed=sc.seed,
        box_half_m=_BOX_HALF_M,
        origin_lla=sc.origin_lla,
        static_s=_STATIC_S,
        ramp_s=_RAMP_S,
        t_b_l=((tr[0], tr[1], tr[2]), (q[0], q[1], q[2], q[3])),
    )
    bridged = data / "bridged"
    run_bridge(data / "raw", sc, load_profile(sc.profile), bridged, seed=sc.seed)
    convert(bridged / "bag", data / "mssdata", pipeline_config=sc.pipeline_config)
    report = mutate(
        bridged / "bag",
        data / "bag_e03",
        drop_fraction=_E03_DROP,
        reorder_fraction=_E03_REORDER,
        window_ns=_E03_WINDOW_NS,
        seed=_E03_SEED,
    )
    (data / "mutate_report.json").write_text(
        json.dumps(asdict(report), indent=2) + "\n", encoding="utf-8"
    )
    convert(data / "bag_e03", data / "mssdata_e03", pipeline_config=sc.pipeline_config)
    return Prepared(data / "mssdata", data / "mssdata_e03", bridged / "gt.tum", sc.pipeline_config)


_WIN_ABS = re.compile(r"^([A-Za-z]):[\\/](.*)$")


def to_bench_path(path: Path | str, path_style: str) -> str:
    """Absolute path as the bench sees it; ``wsl`` maps ``C:\\a\\b`` to ``/mnt/c/a/b``."""
    text = str(path)
    if path_style == "wsl":
        m = _WIN_ABS.match(text) or _WIN_ABS.match(str(Path(text).resolve()))
        if m is None:
            return Path(text).resolve().as_posix()
        return f"/mnt/{m.group(1).lower()}/" + m.group(2).replace("\\", "/")
    return str(Path(text).resolve())


def run_bench(
    bench_cmd: list[str],
    default_yaml: Path,
    data: Path,
    overlay: Path,
    out: Path,
    path_style: str,
) -> int:
    """Run the bench on ``data`` into ``out``; stdout/stderr go to ``out / "bench.log"``.

    Raises SetupError when the command cannot be started or the shell reports it missing.
    """
    out.mkdir(parents=True, exist_ok=True)
    args = [
        to_bench_path(default_yaml, path_style),
        to_bench_path(data, path_style),
        "--overlay",
        to_bench_path(overlay, path_style),
        "--out",
        to_bench_path(out, path_style),
    ]
    try:
        proc = subprocess.run(
            [*bench_cmd, *args], capture_output=True, text=True, check=False, errors="replace"
        )
    except OSError as e:
        raise SetupError(f"cannot run bench command {bench_cmd!r}: {e}") from e
    (out / "bench.log").write_text(
        f"$ {shlex.join([*bench_cmd, *args])}\n--- stdout\n{proc.stdout}--- stderr\n{proc.stderr}",
        encoding="utf-8",
    )
    if proc.returncode in _SHELL_NOT_FOUND:
        raise SetupError(
            f"bench command not found or not executable (exit {proc.returncode}): "
            f"{proc.stderr.strip()[-300:]}"
        )
    return proc.returncode


def _baseline(baselines: dict, test_id: str, key: str) -> float | None:
    value = baselines.get(test_id, {}).get(key)
    return None if value is None else float(value)


def _ate_gate(ate_m: float | None, limit_from: float | None) -> tuple[bool, str]:
    if ate_m is None or not math.isfinite(ate_m):
        return False, f"no ATE (fewer than {_MIN_PAIRS} matched poses or no est.tum)"
    if limit_from is None:
        return True, f"no baseline: record ate_rmse_m={ate_m:.6f} in a baseline-update PR"
    limit = _REGRESSION_FACTOR * limit_from
    ok = ate_m <= limit
    word = "ok" if ok else "REGRESSION"
    return ok, f"{word}: ATE {ate_m:.6f} m vs E-01 baseline {limit_from:.6f} m (limit {limit:.6f})"


def judge(
    test_id: str,
    ate_m: float | None,
    bench_exit: int,
    baselines: dict,
    identical: bool | None,
) -> Verdict:
    """Gate one integration test (see the module docstring)."""
    e01 = _baseline(baselines, "E-01", "ate_rmse_m")
    if test_id == "E-01":
        if bench_exit != _EXIT_OK:
            return Verdict(False, f"bench exit {bench_exit} (need 0)")
        return Verdict(*_ate_gate(ate_m, e01))
    if test_id == "E-02":
        if bench_exit != _EXIT_OK:
            return Verdict(False, f"bench exit {bench_exit} (need 0)")
        if identical is not True:
            return Verdict(False, "est.tum differs from the E-01 run (not deterministic)")
        return Verdict(True, "est.tum byte-identical to the E-01 run")
    if test_id == "E-03":
        if bench_exit not in _E03_BENCH_OK:
            return Verdict(False, f"bench exit {bench_exit} (need 0, or 3 = ran with warnings)")
        ok, note = _ate_gate(ate_m, e01)
        own = _baseline(baselines, "E-03", "ate_rmse_m")
        warn = "; bench warned: records not processed" if bench_exit == _EXIT_INCOMPLETE else ""
        own_note = "" if own is None else f"; own E-03 baseline {own:.6f} m (not gated)"
        return Verdict(ok, note + own_note + warn)
    raise ValueError(f"unknown test id {test_id!r}")


def judge_latency(p95_ms: float | None, baselines: dict) -> Verdict:
    """PF-01: per-scan p95 latency of E-01 against its baseline. Never fails (shared runner)."""
    if p95_ms is None:
        return Verdict(True, "WARN: no scan_process_ms_p95 in bench.json")
    base = _baseline(baselines, "E-01", "p95_latency_ms")
    if base is None:
        return Verdict(
            True, f"no baseline: record p95_latency_ms={p95_ms:.3f} in a baseline-update PR"
        )
    limit = _REGRESSION_FACTOR * base
    if p95_ms > limit:
        return Verdict(True, f"WARN: p95 {p95_ms:.3f} ms > {limit:.3f} ms (1.2 x {base:.3f})")
    return Verdict(True, f"ok: p95 {p95_ms:.3f} ms vs baseline {base:.3f} ms")


def _read_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


_BENCH_KEYS = (
    "n_scans",
    "n_dropped_scans",
    "n_skipped_scans",
    "n_uncovered_scans",
    "n_rejected_imu",
    "n_dropped_imu",
    "n_dropped_aux",
    "n_keyframes",
    "n_match_failures",
    "n_gnss_factors",
    "n_odom_factors",
)


def _score(test_dir: Path, gt_tum: Path, plots: bool) -> dict[str, Any]:
    """ATE of ``test_dir / "est.tum"`` (``ate.json``) plus the bench counters of interest."""
    row: dict[str, Any] = {"ate_rmse_m": None, "n_pairs": 0, "n_unmatched": None}
    est = test_dir / "est.tum"
    try:
        res = ate(est, gt_tum, min_pairs=_MIN_PAIRS)
        row.update(ate_rmse_m=res.rmse_m, n_pairs=res.n_pairs, n_unmatched=res.n_unmatched)
        if plots:
            plot_xy(est, gt_tum, test_dir / "traj_xy.png")
    except (OSError, ValueError) as e:
        row["ate_error"] = str(e)
    (test_dir / "ate.json").write_text(json.dumps(row, indent=2) + "\n", encoding="utf-8")
    bench = _read_json(test_dir / "bench.json")
    row["p95_latency_ms"] = bench.get("scan_process_ms_p95")
    row["p50_latency_ms"] = bench.get("scan_process_ms_p50")
    row["bench"] = {k: bench[k] for k in _BENCH_KEYS if k in bench}
    return row


def _fmt(value: Any, digits: int) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def _summary(results: dict[str, dict[str, Any]], mutate_report: dict[str, Any]) -> str:
    lines = [
        "# Integration E-01..E-03, PF-01",
        "",
        "| test | result | ATE RMSE [m] | pairs | bench exit | p50 / p95 [ms] | note |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for test_id, r in results.items():
        verdict = "PASS" if r["passed"] else "FAIL"
        lat = f"{_fmt(r.get('p50_latency_ms'), 3)} / {_fmt(r.get('p95_latency_ms'), 3)}"
        lines.append(
            f"| {test_id} | {verdict} | {_fmt(r.get('ate_rmse_m'), 4)} | {r.get('n_pairs', '-')}"
            f" | {r.get('bench_exit', '-')} | {lat} | {r['note']} |"
        )
    if "E-03" in results:
        lines += [
            "",
            f"E-03: {REORDER_NOTE}. mutate_bag.py: {mutate_report.get('n_dropped', '?')} "
            f"messages dropped, {mutate_report.get('n_reordered', '?')} pairs reordered "
            f"(of {mutate_report.get('n_in', '?')}); out-of-order handling is covered by C-08.",
        ]
    lines += ["", "Bench counters per test: integration.json (`bench`)."]
    return "\n".join(lines) + "\n"


def _parse(argv: list[str] | None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bench-cmd", required=True, help="command that runs mss_bench (shlex)")
    ap.add_argument("--out", required=True, type=Path, help="results dir, e.g. results/<run-id>")
    ap.add_argument("--scenario", type=Path, default=_REPO / "sim" / "scenarios" / "E-01.yaml")
    ap.add_argument("--baselines", type=Path, default=_REPO / "eval" / "baselines.json")
    ap.add_argument("--tests", default=",".join(_TESTS), help="comma-separated subset")
    ap.add_argument("--duration-s", type=float, default=None, help="override the scenario")
    ap.add_argument("--plots", action="store_true", help="write traj_xy.png per test")
    ap.add_argument("--path-style", choices=("native", "wsl"), default="native")
    return ap.parse_args(argv)


def _setup(args: argparse.Namespace) -> tuple[list[str], list[str], dict]:
    tests = [t.strip() for t in args.tests.split(",") if t.strip()]
    unknown = sorted(set(tests) - set(_TESTS))
    if unknown or not tests:
        raise SetupError(f"--tests: unknown or empty {unknown or args.tests!r}")
    if not args.scenario.is_file():
        raise SetupError(f"scenario {args.scenario} not found")
    try:
        baselines = json.loads(args.baselines.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise SetupError(f"baselines {args.baselines}: {e}") from e
    bench_cmd = shlex.split(args.bench_cmd)
    if not bench_cmd or shutil.which(bench_cmd[0]) is None:
        raise SetupError(f"bench command {args.bench_cmd!r} not found")
    if args.out.exists() and any(args.out.iterdir()):
        raise SetupError(f"--out {args.out} is not empty")
    return tests, bench_cmd, baselines


def _run(args: argparse.Namespace) -> int:
    tests, bench_cmd, baselines = _setup(args)
    out: Path = args.out.resolve()
    try:
        prep = prepare(args.scenario, out, args.duration_s)
    except (OSError, ValueError, KeyError) as e:
        raise SetupError(f"data preparation failed: {e}") from e

    def bench(test_id: str, data: Path) -> int:
        return run_bench(
            bench_cmd, _DEFAULT_YAML, data, prep.overlay, out / test_id, args.path_style
        )

    results: dict[str, dict[str, Any]] = {}
    exits: dict[str, int] = {}
    if "E-01" in tests or "E-02" in tests:  # E-02 compares against the E-01 run
        exits["E-01"] = bench("E-01", prep.mssdata)
    if "E-02" in tests:
        exits["E-02"] = bench("E-02", prep.mssdata)
    if "E-03" in tests:
        exits["E-03"] = bench("E-03", prep.mssdata_e03)

    for test_id in tests:
        row = _score(out / test_id, prep.gt_tum, args.plots)
        identical = None
        if test_id == "E-02":
            a, b = out / "E-01" / "est.tum", out / "E-02" / "est.tum"
            identical = a.is_file() and b.is_file() and a.read_bytes() == b.read_bytes()
            row["identical"] = identical
        verdict = judge(test_id, row["ate_rmse_m"], exits[test_id], baselines, identical)
        row.update(
            bench_exit=exits[test_id],
            baseline=baselines.get(test_id, {}).get("ate_rmse_m"),
            passed=verdict.passed,
            note=verdict.note,
        )
        results[test_id] = row
    if "E-01" in results:
        p95 = results["E-01"].get("p95_latency_ms")
        lat = judge_latency(p95, baselines)
        results["PF-01"] = {
            "p95_latency_ms": p95,
            "p50_latency_ms": results["E-01"].get("p50_latency_ms"),
            "baseline": baselines.get("E-01", {}).get("p95_latency_ms"),
            "passed": lat.passed,
            "note": lat.note,
        }

    (out / "integration.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    summary = _summary(results, _read_json(out / "data" / "mutate_report.json"))
    (out / "summary.md").write_text(summary, encoding="utf-8")
    print(summary)
    return EXIT_PASS if all(r["passed"] for r in results.values()) else EXIT_FAIL


def main(argv: list[str] | None = None) -> int:
    args = _parse(argv)
    try:
        return _run(args)
    except SetupError as e:
        print(f"run_integration: setup error: {e}", file=sys.stderr)
        return EXIT_SETUP


if __name__ == "__main__":
    os.environ.setdefault("MPLBACKEND", "Agg")
    sys.exit(main())
