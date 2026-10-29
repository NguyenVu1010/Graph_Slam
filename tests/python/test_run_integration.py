"""Integration harness (tools/run_integration.py) with a fake bench: gates, report, exit codes."""

import json
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from run_integration import (
    BENCH_TIMEOUT,
    REORDER_NOTE,
    judge,
    judge_e03_drops,
    judge_latency,
    main,
    run_bench,
    to_bench_path,
)

_REPO = Path(__file__).resolve().parents[2]

# Stand-in for mss_bench: copies the dataset's ground truth (gt.bin, GT_DTYPE of
# bag_to_mssdata) to <out>/est.tum and writes a bench.json (n_scans = the dataset's scan
# count) and, with --map-out FILE, a placeholder FILE; exit code from FAKE_BENCH_EXIT, optional
# delay FAKE_BENCH_SLEEP_S.
_FAKE_BENCH = """
import json, os, sys, time
from pathlib import Path
import numpy as np

GT = np.dtype([("t", "<i8"), ("p", "<f8", 3), ("q_wxyz", "<f8", 4)], align=False)
args = sys.argv[1:]
data, out = Path(args[1]), Path(args[args.index("--out") + 1])
assert Path(args[0]).is_absolute() and data.is_absolute() and out.is_absolute()
assert Path(args[args.index("--overlay") + 1]).is_file()
out.mkdir(parents=True, exist_ok=True)
if "--map-out" in args:
    Path(args[args.index("--map-out") + 1]).write_bytes(b"fake pcd")
gt = np.fromfile(data / "gt.bin", dtype=GT)
lines = []
for r in gt:
    t = int(r["t"])
    w, x, y, z = r["q_wxyz"]
    vals = [*r["p"], x, y, z, w]
    lines.append(f"{t // 10**9}.{t % 10**9:09d} " + " ".join(f"{v:.17g}" for v in vals))
(out / "est.tum").write_text("\\n".join(lines) + "\\n", encoding="utf-8")
time.sleep(float(os.environ.get("FAKE_BENCH_SLEEP_S", "0")))
n_scans = json.loads((data / "index.json").read_text())["topics"]["lidar"]["count"]
bench = {"n_scans": n_scans, "scan_process_ms_p50": 0.5, "scan_process_ms_p95": 1.0}
(out / "bench.json").write_text(json.dumps(bench), encoding="utf-8")
sys.exit(int(os.environ.get("FAKE_BENCH_EXIT", "0")))
"""


def _fake_bench_cmd(tmp: Path) -> str:
    script = tmp / "fake_bench.py"
    script.write_text(_FAKE_BENCH, encoding="utf-8")
    return shlex.join([Path(sys.executable).as_posix(), script.as_posix()])


def _scenario(tmp: Path, prep: bool = True, mutate: bool = True) -> Path:
    """Copy of sim/scenarios/E-01.yaml in ``tmp``, optionally without prep or prep.mutate."""
    data = yaml.safe_load((_REPO / "sim/scenarios/E-01.yaml").read_text(encoding="utf-8"))
    name = "scenario"
    if not prep:
        data.pop("prep")
        name += "_no_prep"
    elif not mutate:
        data["prep"].pop("mutate")
        name += "_no_mutate"
    path = tmp / f"{name}.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


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
    # Own null baselines: the test pins the "no baseline" path independently of the repo's
    # eval/baselines.json, which holds real values since the first baseline-update.
    baselines = tmp / "baselines.json"
    null = {"ate_rmse_m": None, "p95_latency_ms": None}
    baselines.write_text(json.dumps({t: null for t in ("E-01", "E-02", "E-03")}), "utf-8")
    argv = ["--bench-cmd", _fake_bench_cmd(tmp), "--out", str(out), "--duration-s", "8"]
    code = main([*argv, "--scenario", str(_scenario(tmp)), "--baselines", str(baselines)])
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
    src = res["data_source"]
    assert src["speed_mps"] == 0.5 and src["omega_radps"] == 0.3
    assert src["box_half_m"] == [8.0, 5.0, 2.0] and src["duration_s"] == 8.0
    assert (src["static_s"], src["ramp_s"], src["seed"]) == (5.0, 2.0, 42)
    assert src["scenario_world_trajectory_used"] is False
    assert src["prep_source"].endswith("scenario.yaml")
    assert src["mutate"] == {
        "drop_fraction": 0.05,
        "reorder_fraction": 0.02,
        "window_ns": 20_000_000,
        "seed": 42,
    }
    assert "E-03 processed" in res["E-03"]["note"]
    for test_id in ("E-01", "E-02", "E-03"):
        assert (out / test_id / "est.tum").is_file()
        assert (out / test_id / "ate.json").is_file()
        assert (out / test_id / "bench.log").is_file()
    assert (out / "data" / "mssdata" / "index.json").is_file()
    assert (out / "data" / "mssdata_e03" / "index.json").is_file()


def test_missing_prep_is_setup_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "results"
    scenario = _scenario(tmp_path, prep=False)
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out)]
    assert main([*argv, "--scenario", str(scenario)]) == 2
    assert "scenario E-01: missing prep block" in capsys.readouterr().err
    assert not out.exists()  # rejected before any data is prepared


def test_missing_mutate_needs_no_e03(tmp_path: Path) -> None:
    out = tmp_path / "results"
    scenario = _scenario(tmp_path, mutate=False)
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out), "--duration-s", "6"]
    assert main([*argv, "--scenario", str(scenario), "--tests", "E-01"]) == 0
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    assert res["data_source"]["mutate"] is None
    assert not (out / "data" / "mssdata_e03").exists()


def test_missing_mutate_with_e03_is_setup_error(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "results"
    scenario = _scenario(tmp_path, mutate=False)
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out)]
    assert main([*argv, "--scenario", str(scenario), "--tests", "E-01,E-03"]) == 2
    assert "scenario E-01: prep.mutate is required for E-03" in capsys.readouterr().err
    assert not out.exists()


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


def test_e03_drops_must_reach_the_bench() -> None:
    v = judge_e03_drops(80, 80, 20)  # as many scans as E-01: the drops never reached the bench
    assert not v.passed
    assert "E-03 processed 80 of 80 E-01 scans; 20 records dropped upstream" in v.note
    assert not judge_e03_drops(75, 80, 0).passed  # mutate_bag.py dropped nothing
    assert not judge_e03_drops(None, 80, 20).passed  # no bench.json
    assert judge_e03_drops(75, 80, 20).passed


def test_e03_summary_carries_scan_counts(e2e: tuple[int, Path, str | None, str | None]) -> None:
    _, out, _, _ = e2e
    summary = (out / "summary.md").read_text(encoding="utf-8")
    assert "E-03 processed " in summary and " of 80 E-01 scans; " in summary
    assert "synthetic box circle, R = V/W = 0.5/0.3 = 1.67 m" in summary


def test_e02_needs_a_trajectory() -> None:
    base = {"E-01": {"ate_rmse_m": None}}
    v = judge("E-02", None, 0, base, True, ate_error="only 0 pairs")
    assert not v.passed and "only 0 pairs" in v.note


def test_ate_error_is_in_the_note() -> None:
    v = judge("E-01", None, 0, {"E-01": {}}, None, ate_error="est.tum: line 3: non-finite pose")
    assert not v.passed and "line 3: non-finite pose" in v.note


def test_bench_timeout_fails_the_test(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_BENCH_SLEEP_S", "30")
    out = tmp_path / "results"
    cmd = _fake_bench_cmd(tmp_path)
    argv = ["--bench-cmd", cmd, "--out", str(out), "--duration-s", "6", "--tests", "E-01"]
    assert main([*argv, "--timeout-s", "1"]) == 1
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    assert res["E-01"]["bench_exit"] == 124 and "timed out" in res["E-01"]["note"]


def test_unexpected_exception_is_setup_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import run_integration

    def boom(*_args: object, **_kwargs: object) -> None:
        raise RuntimeError("rosbags exploded")

    monkeypatch.setattr(run_integration, "prepare", boom)
    out = tmp_path / "results"
    assert main(["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out)]) == 2


_SLEEPER = "import time; time.sleep(5)"


def _kill_recorder(marker: Path) -> list[str]:
    """Stand-in for ``docker kill``: writes the container name it is given into ``marker``."""
    code = "import pathlib, sys; pathlib.Path(sys.argv[1]).write_text(sys.argv[2])"
    return [sys.executable, "-c", code, str(marker)]


def test_timeout_without_container_kills_nothing(tmp_path: Path) -> None:
    marker = tmp_path / "killed.txt"
    cmd = [sys.executable, "-c", _SLEEPER]
    data = tmp_path / "data"
    data.mkdir()
    code = run_bench(
        cmd,
        _REPO / "config/default.yaml",
        data,
        _REPO / "config/sim/warehouse.yaml",
        tmp_path / "E-01",
        "native",
        0.5,
        kill_cmd=_kill_recorder(marker),
    )
    assert code == BENCH_TIMEOUT
    assert not marker.exists()


def test_timeout_kills_the_container(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    import run_integration

    marker = tmp_path / "killed.txt"
    monkeypatch.setattr(run_integration, "DOCKER_KILL", tuple(_kill_recorder(marker)))
    cmd = shlex.join([Path(sys.executable).as_posix(), "-c", _SLEEPER, "--name", "{container}"])
    out = tmp_path / "run 1"
    argv = ["--bench-cmd", cmd, "--out", str(out), "--duration-s", "6", "--tests", "E-01"]
    assert main([*argv, "--scenario", str(_scenario(tmp_path)), "--timeout-s", "0.5"]) == 1
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    assert res["E-01"]["bench_exit"] == BENCH_TIMEOUT and "timed out" in res["E-01"]["note"]
    name = "mss-bench-run-1-E-01"  # sanitised: the space becomes '-'
    assert marker.read_text() == name
    assert f"docker kill {name} attempted (see bench.log)" in res["E-01"]["note"]
    log = (out / "E-01" / "bench.log").read_text(encoding="utf-8")
    assert f"--name {name}" in log and "kill" in log


def test_bridged_bag_source(bridged_bag: tuple[Path, object], tmp_path: Path) -> None:
    """--bridged replaces the synthetic data by a recorded bridge output (bag/ + gt.tum)."""
    bridged, _ = bridged_bag
    out = tmp_path / "results"
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out), "--bridged", str(bridged)]
    code = main([*argv, "--scenario", str(_scenario(tmp_path))])
    summary = (out / "summary.md").read_text(encoding="utf-8")
    assert code == 0, summary
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    src = res["data_source"]
    assert src["generator"] == "recorded bag (bridge output)"
    assert src["scenario_world_trajectory_used"] is True
    assert src["scenario"] == "E-01" and src["mutate"]["drop_fraction"] == 0.05
    assert res["E-01"]["ate_rmse_m"] < 1e-6 and res["E-02"]["identical"] is True
    assert "E-03 processed" in res["E-03"]["note"]
    assert not (out / "data" / "raw").exists()  # nothing synthesised
    assert "Data: recorded bag" in summary


def test_bridged_dir_without_bag_is_setup_error(tmp_path: Path) -> None:
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(tmp_path / "r")]
    argv += ["--bridged", str(tmp_path), "--scenario", str(_scenario(tmp_path))]
    assert main(argv) == 2


def test_bridged_refuses_duration_override(
    bridged_bag: tuple[Path, object], tmp_path: Path
) -> None:
    bridged, _ = bridged_bag
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(tmp_path / "r")]
    argv += ["--bridged", str(bridged), "--duration-s", "5", "--scenario", str(_scenario(tmp_path))]
    assert main(argv) == 2


def test_bridged_recording_must_match_the_scenario(
    bridged_bag: tuple[Path, object], tmp_path: Path
) -> None:
    bridged, _ = bridged_bag
    rec = tmp_path / "rec"
    shutil.copytree(bridged, rec)
    (rec / "assets").mkdir()
    meta = {"profile": "sim/noise/default.yaml", "pipeline_config": "config/sim/corridor.yaml"}
    (rec / "assets" / "assets.yaml").write_text(yaml.safe_dump(meta))
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(tmp_path / "r"), "--bridged"]
    assert main([*argv, str(rec), "--scenario", str(_scenario(tmp_path))]) == 2  # E-01 vs corridor
    meta["pipeline_config"] = "config/sim/warehouse.yaml"
    (rec / "assets" / "assets.yaml").write_text(yaml.safe_dump(meta))
    out = tmp_path / "ok"
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out), "--bridged", str(rec)]
    assert main([*argv, "--scenario", str(_scenario(tmp_path)), "--tests", "E-01"]) == 0


def test_bridged_e03_without_prep_is_setup_error(
    bridged_bag: tuple[Path, object], tmp_path: Path
) -> None:
    """--bridged needs no prep for E-01/E-02, but E-03 still needs prep.mutate."""
    bridged, _ = bridged_bag
    no_prep = _scenario(tmp_path, prep=False)
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--bridged", str(bridged)]
    assert main([*argv, "--out", str(tmp_path / "a"), "--scenario", str(no_prep)]) == 2
    out = tmp_path / "b"
    e01 = [*argv, "--out", str(out), "--scenario", str(no_prep), "--tests", "E-01,E-02"]
    assert main(e01) == 0


def test_bridged_map_out_e01_only(bridged_bag: tuple[Path, object], tmp_path: Path) -> None:
    """--map-out hands the bench <test-dir>/map.pcd; without E-03 no E-03 dataset is built."""
    bridged, _ = bridged_bag
    out = tmp_path / "map"
    argv = ["--bench-cmd", _fake_bench_cmd(tmp_path), "--out", str(out), "--bridged", str(bridged)]
    argv += ["--scenario", str(_scenario(tmp_path)), "--tests", "E-01", "--map-out"]
    assert main(argv) == 0
    assert (out / "E-01" / "map.pcd").read_bytes() == b"fake pcd"
    log = (out / "E-01" / "bench.log").read_text(encoding="utf-8")
    assert "--map-out " in log and "map.pcd" in log
    assert not (out / "data" / "bag_e03").exists()
    assert not (out / "data" / "mssdata_e03").exists()
    res = json.loads((out / "integration.json").read_text(encoding="utf-8"))
    assert res["data_source"]["mutate"] is None
