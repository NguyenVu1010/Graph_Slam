"""Explorer runner: chain, cache, failure states and CLI, with a fake bench."""

import json
import os
import shutil
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from conftest import _REPO, cli_env

from mss_explorer.runner import RunResult, list_runs, run_experiment, run_states
from mss_explorer.spec import ExperimentSpec, SpecError, dump_spec
from mss_sim.synth import SynthInfo

_FAKE = """
import os, shutil, sys, time
from pathlib import Path
args = sys.argv[1:]
out = Path(args[args.index("--out") + 1])
out.mkdir(parents=True, exist_ok=True)
with open(os.environ["FAKE_COUNT"], "a") as f:
    f.write("run\\n")
mode = os.environ.get("FAKE_MODE", "ok")
if "--map-out" in args:
    Path(args[args.index("--map-out") + 1]).write_text("map")
if mode == "fail":
    sys.exit(1)
if mode == "sleep":
    time.sleep(30)
if mode == "noest":
    sys.exit(0)
gt = (out.parent / "gt.tum").read_text().splitlines()
lines = gt[:3] if mode == "short" else gt
if mode == "gap":
    lines = gt[: len(gt) // 10] + gt[-len(gt) // 10 :]
(out / "est.tum").write_text("\\n".join(lines) + "\\n")
(out / "kf_opt.tum").write_text("\\n".join(lines[::10]) + "\\n")
(out / "bench.json").write_text('{"stats": {"n_loops_accepted": 2}}')
sys.exit(3 if mode == "exit3" else 0)
"""


@pytest.fixture
def setup(raw_bag: tuple[Path, SynthInfo], tmp_path: Path, monkeypatch: pytest.MonkeyPatch):  # noqa: ANN201
    raw, _ = raw_bag
    rec = tmp_path / "rec"
    shutil.copytree(raw, rec / "raw_bag")
    (rec / "assets").mkdir()
    meta = {"profile": "sim/noise/default.yaml", "pipeline_config": "config/sim/warehouse.yaml"}
    (rec / "assets" / "assets.yaml").write_text(yaml.safe_dump(meta))
    sc_text = (_REPO / "sim/scenarios/E-01.yaml").read_text()
    sc_text = sc_text.replace("duration_s: 60", "duration_s: 6").replace(
        "t_end_s: 60", "t_end_s: 6"
    )
    scenario = tmp_path / "E-01-6s.yaml"
    scenario.write_text(sc_text)
    fake = tmp_path / "fake_bench.py"
    fake.write_text(_FAKE)
    count = tmp_path / "count.txt"
    monkeypatch.setenv("FAKE_COUNT", str(count))
    monkeypatch.setenv("FAKE_MODE", "ok")
    spec = ExperimentSpec(scenario=scenario, recording=rec)
    return spec, f"{sys.executable} {fake}", tmp_path / "explorer", count


def _n_runs(count: Path) -> int:
    return len(count.read_text().splitlines()) if count.exists() else 0


def test_run_ok_and_cached(setup) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "ok", r.reason
    assert r.ate_online_m == pytest.approx(0.0, abs=1e-9)
    assert r.ate_opt_m == pytest.approx(0.0, abs=1e-9)
    assert r.n_loops_accepted == 2
    assert not (r.run_dir / "tmp").exists()
    assert (r.run_dir / "errors_online.csv").read_text().startswith("t_s,err_m")
    again = run_experiment(spec, bench_cmd=bench, root=root)
    assert again == r and _n_runs(count) == 1


def test_run_failed_not_cached(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    monkeypatch.setenv("FAKE_MODE", "fail")
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "failed" and "exit 1" in r.reason
    monkeypatch.setenv("FAKE_MODE", "ok")
    assert run_experiment(spec, bench_cmd=bench, root=root).status == "ok"
    assert _n_runs(count) == 2


def test_run_timeout_failed(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    monkeypatch.setenv("FAKE_MODE", "sleep")
    r = run_experiment(spec, bench_cmd=bench, root=root, timeout_s=1.0)
    assert r.status == "failed" and "timeout" in r.reason


def test_run_lost(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    monkeypatch.setenv("FAKE_MODE", "short")
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "lost" and r.ate_online_m is None and r.reason


def test_run_running_lock(setup) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    first = run_experiment(replace(spec, seed=5), bench_cmd=bench, root=root)
    shutil.rmtree(first.run_dir)
    first.run_dir.mkdir(parents=True)
    (first.run_dir / ".lock").write_text(str(os.getpid()))
    r = run_experiment(replace(spec, seed=5), bench_cmd=bench, root=root)
    assert r.status == "running" and _n_runs(count) == 1


def test_stale_lock_replaced(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    first = run_experiment(spec, bench_cmd=bench, root=root)
    shutil.rmtree(first.run_dir)
    first.run_dir.mkdir(parents=True)
    (first.run_dir / ".lock").write_text("999999999")
    assert run_experiment(spec, bench_cmd=bench, root=root).status == "ok"


def test_recording_without_raw_bag(setup, tmp_path: Path) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    with pytest.raises(SpecError, match="raw_bag"):
        run_experiment(replace(spec, recording=tmp_path / "nowhere"), bench_cmd=bench, root=root)


def test_recording_of_another_scenario(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    meta = {"profile": "sim/noise/default.yaml", "pipeline_config": "config/sim/outdoor.yaml"}
    (spec.recording / "assets" / "assets.yaml").write_text(yaml.safe_dump(meta))
    with pytest.raises(SpecError, match="pipeline_config"):
        run_experiment(spec, bench_cmd=bench, root=root)


def test_overlay_sets_factors(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    spec = replace(spec, factors={"gnss": True, "wheel_odom": None, "loop_closure": False})
    r = run_experiment(spec, bench_cmd=bench, root=root)
    p = yaml.safe_load((r.run_dir / "overlay.yaml").read_text())["mss_node"]["ros__parameters"]
    assert p["gnss"]["enabled"] is True and p["loop_closure"]["enabled"] is False
    base = yaml.safe_load((_REPO / "config/sim/warehouse.yaml").read_text())
    want = base["mss_node"]["ros__parameters"].get("wheel_odom", {}).get("enabled")
    assert p.get("wheel_odom", {}).get("enabled") == want


def test_list_runs(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    r = run_experiment(replace(spec, label="ref"), bench_cmd=bench, root=root)
    runs = list_runs(root)
    assert [(s.label, x) for s, x in runs] == [("ref", r)]
    assert isinstance(runs[0][1], RunResult)


def test_cli_exit_codes(setup, tmp_path: Path) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    good, bad = tmp_path / "good.yaml", tmp_path / "bad.yaml"
    dump_spec(spec, good)
    bad.write_text("scenario: nowhere.yaml\nrecording: x\n")
    env = {**cli_env(), "MSS_BENCH_CMD": bench}

    def cli(path: Path) -> subprocess.CompletedProcess:
        cmd = [sys.executable, "-m", "mss_explorer", "run", str(path), "--root", str(root)]
        return subprocess.run(cmd, env=env, capture_output=True, text=True, check=False)

    ok = cli(good)
    assert ok.returncode == 0, ok.stderr
    assert json.loads(ok.stdout)["status"] == "ok"
    assert cli(bad).returncode == 2
    env["FAKE_MODE"] = "fail"
    s9 = tmp_path / "s9.yaml"
    dump_spec(replace(spec, seed=9), s9)
    assert cli(s9).returncode == 1


def test_overlay_from_variant(setup, tmp_path: Path) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    text = (
        spec.scenario.read_text()
        + "pipeline_config_variants:\n  loops_on: config/sim/warehouse_e07_loops_on.yaml\n"
    )
    spec.scenario.write_text(text)
    r = run_experiment(replace(spec, variant="loops_on"), bench_cmd=bench, root=root)
    p = yaml.safe_load((r.run_dir / "overlay.yaml").read_text())["mss_node"]["ros__parameters"]
    assert p["loop_closure"]["kernel_release"] is True


def test_recording_without_assets(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    (spec.recording / "assets" / "assets.yaml").unlink()
    with pytest.raises(SpecError, match="assets.yaml"):
        run_experiment(spec, bench_cmd=bench, root=root)


def test_run_exit3_ok_with_reason(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    monkeypatch.setenv("FAKE_MODE", "exit3")
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "ok" and "exit 3" in r.reason


def test_run_no_estimate_failed(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    monkeypatch.setenv("FAKE_MODE", "noest")
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "failed" and "est.tum" in r.reason
    run_experiment(spec, bench_cmd=bench, root=root)
    assert _n_runs(count) == 2  # not cached


def test_run_gap_lost(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    monkeypatch.setenv("FAKE_MODE", "gap")
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "lost" and "covers" in r.reason


def test_key_changes_with_recording(setup) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    a = run_experiment(spec, bench_cmd=bench, root=root)
    (spec.recording / "record_summary.json").write_text('{"raw_bag_files": [{"sha256": "x"}]}')
    b = run_experiment(spec, bench_cmd=bench, root=root)
    assert a.key != b.key and _n_runs(count) == 2


def test_key_changes_with_bench_binary(setup) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    a = run_experiment(spec, bench_cmd=bench, root=root)
    fake = Path(bench.split()[-1])
    fake.write_text(fake.read_text() + "\n# rebuilt\n")
    b = run_experiment(spec, bench_cmd=bench, root=root)
    assert a.key != b.key and _n_runs(count) == 2


def test_key_changes_with_uncommitted_edit(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    import mss_explorer.runner as runner

    spec, bench, root, _ = setup
    monkeypatch.setattr(runner, "_tracked_diff", lambda: "edit-1")
    a = run_experiment(spec, bench_cmd=bench, root=root)
    monkeypatch.setattr(runner, "_tracked_diff", lambda: "edit-2")
    b = run_experiment(spec, bench_cmd=bench, root=root)
    assert a.key != b.key


def test_fresh_empty_lock_is_live(setup) -> None:  # noqa: ANN001
    spec, bench, root, count = setup
    first = run_experiment(replace(spec, seed=6), bench_cmd=bench, root=root)
    shutil.rmtree(first.run_dir)
    first.run_dir.mkdir(parents=True)
    (first.run_dir / ".lock").write_text("")  # another process between create and pid write
    r = run_experiment(replace(spec, seed=6), bench_cmd=bench, root=root)
    assert r.status == "running" and _n_runs(count) == 1


def test_run_states_change_only_when_a_run_finishes(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    first = run_experiment(spec, bench_cmd=bench, root=root)
    shutil.rmtree(first.run_dir)
    first.run_dir.mkdir(parents=True)
    dump_spec(spec, first.run_dir / "spec.yaml")
    (first.run_dir / ".lock").write_text(str(os.getpid()))
    running = run_states(root)
    assert running == ((first.key, "running"),) and run_states(root) == running
    (first.run_dir / ".lock").unlink()
    assert run_experiment(spec, bench_cmd=bench, root=root).status == "ok"
    assert run_states(root) == ((first.key, "ok"),)


def test_running_run_reports_step_and_elapsed(setup, monkeypatch: pytest.MonkeyPatch) -> None:  # noqa: ANN001
    import threading
    import time

    spec, bench, root, _ = setup
    monkeypatch.setenv("FAKE_MODE", "sleep")
    worker = threading.Thread(
        target=run_experiment,
        args=(spec,),
        kwargs={"bench_cmd": bench, "root": root, "timeout_s": 4},
    )
    worker.start()
    seen = None
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and seen is None:
        for _, r in list_runs(root):
            if r.status == "running" and "bench" in r.reason:
                seen = r
        time.sleep(0.2)
    worker.join()
    assert seen is not None
    assert seen.reason.startswith("step 4/5: bench") and seen.wall_s > 0
    assert not (seen.run_dir / "stage.txt").exists()


def test_run_writes_map(setup) -> None:  # noqa: ANN001
    spec, bench, root, _ = setup
    r = run_experiment(spec, bench_cmd=bench, root=root)
    assert r.status == "ok" and (r.run_dir / "map.pcd").read_text() == "map"
