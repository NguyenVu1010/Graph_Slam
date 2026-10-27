"""Integration harness (tools/run_integration.py) with a fake bench: gates, report, exit codes."""

import json
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from run_integration import (
    REORDER_NOTE,
    judge,
    judge_latency,
    main,
    to_bench_path,
)

_REPO = Path(__file__).resolve().parents[2]

# Stand-in for mss_bench: copies the dataset's ground truth (gt.bin, GT_DTYPE of
# bag_to_mssdata) to <out>/est.tum and writes a bench.json; exit code from FAKE_BENCH_EXIT.
_FAKE_BENCH = """
import json, os, sys
from pathlib import Path
import numpy as np

GT = np.dtype([("t", "<i8"), ("p", "<f8", 3), ("q_wxyz", "<f8", 4)], align=False)
args = sys.argv[1:]
data, out = Path(args[1]), Path(args[args.index("--out") + 1])
assert Path(args[0]).is_absolute() and data.is_absolute() and out.is_absolute()
assert Path(args[args.index("--overlay") + 1]).is_file()
out.mkdir(parents=True, exist_ok=True)
gt = np.fromfile(data / "gt.bin", dtype=GT)
lines = []
for r in gt:
    t = int(r["t"])
    w, x, y, z = r["q_wxyz"]
    vals = [*r["p"], x, y, z, w]
    lines.append(f"{t // 10**9}.{t % 10**9:09d} " + " ".join(f"{v:.17g}" for v in vals))
(out / "est.tum").write_text("\\n".join(lines) + "\\n", encoding="utf-8")
bench = {"n_scans": 10, "scan_process_ms_p50": 0.5, "scan_process_ms_p95": 1.0}
(out / "bench.json").write_text(json.dumps(bench), encoding="utf-8")
sys.exit(int(os.environ.get("FAKE_BENCH_EXIT", "0")))
"""


def _fake_bench_cmd(tmp: Path) -> str:
    script = tmp / "fake_bench.py"
    script.write_text(_FAKE_BENCH, encoding="utf-8")
    return shlex.join([Path(sys.executable).as_posix(), script.as_posix()])


def _git_status() -> str | None:
    if shutil.which("git") is None:
        return None
    proc = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=all"],
        cwd=_REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    return proc.stdout if proc.returncode == 0 else None


@pytest.fixture(scope="module")
def e2e(tmp_path_factory: pytest.TempPathFactory) -> tuple[int, Path, str | None, str | None]:
    """One harness run (8 s scenario, fake bench): exit code, out dir, git status before/after."""
    tmp = tmp_path_factory.mktemp("integration")
    before = _git_status()
    out = tmp / "results"
    code = main(["--bench-cmd", _fake_bench_cmd(tmp), "--out", str(out), "--duration-s", "8"])
    return code, out, before, _git_status()


def test_end_to_end_with_fake_bench(e2e: tuple[int, Path, str | None, str | None]) -> None:
    code, out, _, _ = e2e
    assert code == 0, (out / "summary.md").read_text(encoding="utf-8")
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    for test_id in ("E-01", "E-02", "E-03", "PF-01"):
        assert res[test_id]["passed"], res[test_id]
    assert res["E-01"]["ate_rmse_m"] < 1e-6
    assert res["E-01"]["n_pairs"] >= 10
    assert res["E-02"]["identical"] is True
    assert "no baseline" in res["E-01"]["note"]
    assert res["PF-01"]["p95_latency_ms"] == 1.0
    for test_id in ("E-01", "E-02", "E-03"):
        assert (out / test_id / "est.tum").is_file()
        assert (out / test_id / "ate.json").is_file()
        assert (out / test_id / "bench.log").is_file()
    assert (out / "data" / "mssdata" / "index.json").is_file()
    assert (out / "data" / "mssdata_e03" / "index.json").is_file()


def test_null_baseline_passes_with_note() -> None:
    v = judge("E-01", 0.3, 0, {"E-01": {"ate_rmse_m": None}}, None)
    assert v.passed
    assert "no baseline: record ate_rmse_m=0.300000 in a baseline-update PR" in v.note


def test_regression_fails() -> None:
    base = {"E-01": {"ate_rmse_m": 0.10}}
    assert not judge("E-01", 0.121, 0, base, None).passed
    assert judge("E-01", 0.12, 0, base, None).passed


def test_e01_needs_bench_exit_0_and_ate() -> None:
    base = {"E-01": {"ate_rmse_m": None}}
    assert not judge("E-01", 0.1, 3, base, None).passed
    assert not judge("E-01", None, 0, base, None).passed
    assert not judge("E-01", float("nan"), 0, base, None).passed


def test_e02_requires_identical() -> None:
    base = {"E-01": {"ate_rmse_m": None}}
    assert not judge("E-02", 0.1, 0, base, False).passed
    assert judge("E-02", 0.1, 0, base, True).passed


def test_e03_accepts_exit_3() -> None:
    base = {"E-01": {"ate_rmse_m": None}}
    assert judge("E-03", 0.1, 3, base, None).passed
    assert judge("E-03", 0.1, 0, base, None).passed
    assert not judge("E-03", 0.1, 2, base, None).passed
    assert not judge("E-03", 0.1, -11, base, None).passed  # crash (signal)


def test_e03_uses_e01_threshold() -> None:
    base = {"E-01": {"ate_rmse_m": 0.10}, "E-03": {"ate_rmse_m": 0.20}}
    assert not judge("E-03", 0.125, 0, base, None).passed
    assert judge("E-03", 0.115, 3, base, None).passed


def test_e03_report_states_reorder_is_resorted(
    e2e: tuple[int, Path, str | None, str | None],
) -> None:
    _, out, _, _ = e2e
    summary = (out / "summary.md").read_text(encoding="utf-8")
    assert REORDER_NOTE in summary
    assert "reordering is re-sorted by bag_to_mssdata.py; only drops reach the bench" in summary


def test_pf01_only_warns() -> None:
    v = judge_latency(2.0, {"E-01": {"p95_latency_ms": 1.0}})
    assert v.passed and "WARN" in v.note
    v = judge_latency(1.1, {"E-01": {"p95_latency_ms": 1.0}})
    assert v.passed and "WARN" not in v.note
    v = judge_latency(1.1, {"E-01": {"p95_latency_ms": None}})
    assert v.passed and "no baseline" in v.note


def test_missing_bench_is_setup_error(tmp_path: Path) -> None:
    out = tmp_path / "results"
    assert main(["--bench-cmd", "/nonexistent/mss_bench", "--out", str(out)]) == 2
    assert not out.exists()  # rejected before any data is prepared


def test_bench_exit_127_is_setup_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_BENCH_EXIT", "127")  # what a shell / wsl / docker reports
    out = tmp_path / "results"
    cmd = _fake_bench_cmd(tmp_path)
    assert (
        main(["--bench-cmd", cmd, "--out", str(out), "--duration-s", "6", "--tests", "E-01"]) == 2
    )


def test_failing_bench_fails_the_run(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_BENCH_EXIT", "2")
    out = tmp_path / "results"
    cmd = _fake_bench_cmd(tmp_path)
    assert (
        main(["--bench-cmd", cmd, "--out", str(out), "--duration-s", "6", "--tests", "E-01"]) == 1
    )
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    assert res["E-01"]["bench_exit"] == 2 and not res["E-01"]["passed"]


def test_wsl_path_style() -> None:
    assert to_bench_path("C:\\a\\b", "wsl") == "/mnt/c/a/b"
    assert to_bench_path("D:/x/y.yaml", "wsl") == "/mnt/d/x/y.yaml"
    assert Path(to_bench_path("rel/dir", "native")).is_absolute()


def test_writes_only_under_out(e2e: tuple[int, Path, str | None, str | None]) -> None:
    _, _, before, after = e2e
    if before is None or after is None:
        pytest.skip("git not available")
    assert after == before
