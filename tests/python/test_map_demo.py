"""tools/run_map_demo.sh and tools/map_demo_verdict.py (backlog A1, A2) with a fake bench."""

import json
import os
import shlex
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest
from test_run_integration import _FAKE_BENCH

from map_demo_verdict import judge_run

_REPO = Path(__file__).resolve().parents[2]
_SCRIPT = _REPO / "tools" / "run_map_demo.sh"


def _e01(bench_exit: int | None = 0, ate: float | None = 0.1, passed: bool = True) -> dict:
    return {"E-01": {"bench_exit": bench_exit, "ate_rmse_m": ate, "passed": passed, "note": "n"}}


def test_gate_pass_is_usable() -> None:
    v = judge_run(_e01(), 0, "E-01")
    assert v["usable"] and v["gate_passed"] is True and v["scenario"] == "E-01"


def test_gate_fail_with_a_complete_bench_is_usable_and_recorded() -> None:
    v = judge_run(_e01(passed=False), 1, "E-04")
    assert v["usable"] and v["gate_passed"] is False
    assert v["scenario"] == "E-04" and "ATE" in v["reason"]


@pytest.mark.parametrize("bench_exit", [124, 1, 2, 3, None])
def test_bench_failure_or_timeout_is_not_usable(bench_exit: int | None) -> None:
    v = judge_run(_e01(bench_exit=bench_exit, ate=None, passed=False), 1, "E-01")
    assert not v["usable"] and "bench" in v["reason"]


def test_missing_ate_or_result_or_setup_error_is_not_usable() -> None:
    assert not judge_run(_e01(ate=None, passed=False), 1, "E-01")["usable"]
    assert not judge_run(None, 1, "E-01")["usable"]
    assert not judge_run({}, 1, "E-01")["usable"]
    assert not judge_run(_e01(), 2, "E-01")["usable"]


def _pcd(points: list[tuple[float, float, float]]) -> bytes:
    n = len(points)
    head = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    return head.encode() + b"".join(struct.pack("<3f", *p) for p in points)


def _run_demo(
    tmp: Path, bridged: Path, bench_exit: int
) -> tuple[subprocess.CompletedProcess, Path]:
    pcd = _pcd([(float(i), float(i % 3), 0.5) for i in range(30)])
    script = tmp / "fake_bench.py"
    script.write_text(_FAKE_BENCH.replace('b"fake pcd"', repr(pcd)), encoding="utf-8")
    bench = shlex.join([Path(sys.executable).as_posix(), script.as_posix()])
    out = tmp / "demo"
    env = {**os.environ, "PYTHON": sys.executable, "FAKE_BENCH_EXIT": str(bench_exit)}
    proc = subprocess.run(
        ["bash", str(_SCRIPT), str(_REPO / "sim/scenarios/E-01.yaml"), str(out),
         "--bench-cmd", bench, "--bag", str(bridged)],
        capture_output=True, text=True, env=env, check=False,
    )  # fmt: skip
    return proc, out


needs_bash = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


@needs_bash
def test_demo_records_scenario_and_gate(bridged_bag: tuple[Path, object], tmp_path: Path) -> None:
    proc, out = _run_demo(tmp_path, bridged_bag[0], 0)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    doc = json.loads((out / "map_demo.json").read_text(encoding="utf-8"))
    assert doc["scenario"] == "E-01"
    assert doc["gate"]["gate_passed"] is True
    assert (out / "map.png").is_file()


@needs_bash
@pytest.mark.parametrize("bench_exit", [1, 124])
def test_demo_refuses_a_failed_bench(
    bridged_bag: tuple[Path, object], tmp_path: Path, bench_exit: int
) -> None:
    """A bench failure makes run_integration exit 1; that must not count as a gate fail."""
    proc, out = _run_demo(tmp_path, bridged_bag[0], bench_exit)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "run_map_demo: OK" not in proc.stdout
    assert not (out / "map.png").exists() and not (out / "map_demo.json").exists()


def test_pf01_failure_does_not_make_the_e01_gate_fail() -> None:
    """run_integration exits 1 when only the PF-01 latency check fails; E-01 itself passed."""
    doc = {**_e01(passed=True), "PF-01": {"passed": False, "note": "p95 too high"}}
    v = judge_run(doc, 1, "E-01")
    assert v["usable"] and v["gate_passed"] is True
    assert v["pf01_passed"] is False and v["pf01_note"] == "p95 too high"
    assert "ATE" not in v["reason"] and "PF-01" in v["reason"]
