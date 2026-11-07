"""Run one experiment offline and cache it under ``<root>/runs/<key>/``.

Chain: raw bag -> noise bridge (scaled profile, extra IMU noise) -> faults -> bag_to_mssdata
-> bench (generated overlay) -> metrics. The run directory keeps ``spec.yaml``, ``overlay.yaml``,
``gt.tum``, ``bench/`` (est.tum, kf_opt.tum, bench.json), ``errors_online.csv``,
``errors_opt.csv``, ``result.json`` and ``log.txt``; the intermediate bags under ``tmp/`` are
deleted. ``ok`` and ``lost`` results are served from the cache; ``failed`` ones run again.
"""

from __future__ import annotations

import hashlib
import json
import os
import shlex
import shutil
import subprocess
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Literal

import numpy as np
import yaml

from bag_to_mssdata import convert
from mss_eval.ate import align, ate, read_tum_stamps_ns
from mss_explorer.metrics import error_series, rpe_t
from mss_explorer.spec import (
    FACTORS,
    REPO,
    ExperimentSpec,
    SpecError,
    dump_spec,
    load_spec,
    spec_key,
)
from mss_sim.bridge import run_bridge
from mss_sim.config_overlay import load_pipeline_config
from mss_sim.faults import apply_faults
from mss_sim.noise_scale import imu_extra, lidar_extra_sigma, scale_profile
from mss_sim.profile import load_profile
from mss_sim.scenario import Scenario, load_scenario

Status = Literal["ok", "lost", "failed", "running"]
DEFAULT_CONFIG = REPO / "config/default.yaml"
TIMEOUT_PER_SCENARIO_S = 10.0  # default bench timeout = this x the scenario duration
MIN_COVERAGE = 0.5  # est.tum must span this fraction of the scenario, else the run is "lost"
MAX_GAP_S = 1.0  # a gap between estimates longer than this does not count as covered
LOCK_GRACE_S = 10.0  # an empty lock this young belongs to a process still writing its pid
_CACHED: tuple[Status, ...] = ("ok", "lost")


@dataclass(frozen=True)
class RunResult:
    key: str
    status: Status
    reason: str
    ate_online_m: float | None
    ate_opt_m: float | None
    rpe10_m: float | None
    rpe50_m: float | None
    n_loops_accepted: int | None
    wall_s: float
    run_dir: Path


def _empty(key: str, status: Status, reason: str, run_dir: Path, wall_s: float = 0.0) -> RunResult:
    return RunResult(key, status, reason, None, None, None, None, None, wall_s, run_dir)


def _save(r: RunResult) -> None:
    doc = {**asdict(r), "run_dir": r.run_dir.name}
    (r.run_dir / "result.json").write_text(json.dumps(doc, indent=2) + "\n")


def _load(run_dir: Path) -> RunResult | None:
    path = run_dir / "result.json"
    if not path.is_file():
        return None
    doc = json.loads(path.read_text())
    return RunResult(**{**doc, "run_dir": run_dir})


def git_commit() -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(REPO), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _tracked_diff() -> str:
    """Uncommitted edits to tracked inputs (config, sim, tools, eval), so they change the key."""
    try:
        out = subprocess.run(
            [
                "git",
                "-C",
                str(REPO),
                "diff",
                "HEAD",
                "--no-color",
                "--",
                "config",
                "sim",
                "tools",
                "eval",
            ],
            capture_output=True,
            check=True,
        )
        return hashlib.sha256(out.stdout).hexdigest()
    except (OSError, subprocess.CalledProcessError):
        return "no-git-diff"


def _file_stamp(p: Path) -> str:
    st = p.stat()
    return f"{p.resolve().as_posix()}:{st.st_size}:{st.st_mtime_ns}"


def run_fingerprint(spec: ExperimentSpec, bench_cmd: str) -> str:
    """What else decides a run besides the spec: git commit, uncommitted edits, the recording
    (``record_summary.json`` hashes, else the raw bag file stamps, plus ``assets.yaml``) and the
    bench command with the size and mtime of every file it names (the binary, its libraries)."""
    parts = [git_commit(), _tracked_diff()]
    summary = spec.recording / "record_summary.json"
    if summary.is_file():
        parts.append(summary.read_text())
    else:
        raw = spec.recording / "raw_bag"
        parts += sorted(_file_stamp(p) for p in raw.iterdir() if p.is_file())
    assets = spec.recording / "assets" / "assets.yaml"
    parts.append(assets.read_text() if assets.is_file() else "")
    parts.append(bench_cmd)
    for tok in shlex.split(bench_cmd):
        for cand in tok.split("=", 1)[-1].split(":"):
            if cand and Path(cand).is_file():
                parts.append(_file_stamp(Path(cand)))
    return hashlib.sha256("\n".join(parts).encode()).hexdigest()


def _rel(p: Path) -> str:
    try:
        return p.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return p.resolve().as_posix()


def check_recording(spec: ExperimentSpec, sc: Scenario) -> Path:
    raw = spec.recording / "raw_bag"
    if not raw.is_dir():
        raise SpecError(f"recording: {raw} is missing (record with tools/record_scenario.sh)")
    meta_path = spec.recording / "assets" / "assets.yaml"
    if not meta_path.is_file():
        msg = f"recording: {meta_path} is missing (not a tools/record_scenario.sh output)"
        raise SpecError(msg)
    meta = yaml.safe_load(meta_path.read_text()) or {}
    for name, expected in (("profile", sc.profile), ("pipeline_config", sc.pipeline_config)):
        if meta.get(name) != _rel(expected):
            raise SpecError(
                f"recording: made with {name} {meta.get(name)!r}, scenario {sc.id} uses "
                f"{_rel(expected)!r}"
            )
    return raw


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _lock(run_dir: Path) -> bool:
    """Take ``run_dir/.lock``; False when a live process holds it (a stale lock is replaced).

    The lock appears with its pid already in it (written to a temp file, then hard-linked), so
    no reader sees it empty; an empty or unreadable lock younger than ``LOCK_GRACE_S`` counts
    as live all the same."""
    path = run_dir / ".lock"
    tmp = run_dir / f".lock.{os.getpid()}.{time.monotonic_ns()}"
    tmp.write_text(str(os.getpid()))
    try:
        for _ in range(2):
            try:
                os.link(tmp, path)
                return True
            except FileExistsError:
                pass
            try:
                pid = int(path.read_text().strip() or "0")
                age_s = time.time() - path.stat().st_mtime
            except (OSError, ValueError):
                pid, age_s = 0, 0.0
            if pid > 0 and _pid_alive(pid):
                return False
            if pid <= 0 and age_s < LOCK_GRACE_S:
                return False
            path.unlink(missing_ok=True)
        return False
    finally:
        tmp.unlink(missing_ok=True)


def write_overlay(
    sc: Scenario, factors: dict[str, bool | None], out: Path, variant: str | None = None
) -> Path:
    """The scenario's overlay (or its ``variant``) with the explicit factor flags set, checked
    against default.yaml."""
    base = sc.pipeline_config if variant is None else sc.pipeline_config_variants[variant]
    doc = yaml.safe_load(base.read_text())
    params = doc.setdefault("mss_node", {}).setdefault("ros__parameters", {})
    for name in FACTORS:
        if factors.get(name) is not None:
            params.setdefault(name, {})["enabled"] = factors[name]
    out.write_text(yaml.safe_dump(doc, sort_keys=False))
    try:
        load_pipeline_config(DEFAULT_CONFIG, out)
    except (KeyError, ValueError) as e:
        raise SpecError(f"factors: overlay rejected: {e}") from e
    return out


def _find(doc: Any, key: str) -> Any:
    if isinstance(doc, dict):
        if key in doc:
            return doc[key]
        for v in doc.values():
            found = _find(v, key)
            if found is not None:
                return found
    return None


def _write_errors(path: Path, est: Path, gt: Path) -> None:
    t, err = error_series(est, gt)
    rows = "\n".join(f"{a:.6f},{b:.6f}" for a, b in zip(t, err, strict=True))
    path.write_text("t_s,err_m\n" + rows + "\n")


def _score(key: str, run_dir: Path, sc: Scenario, wall_s: float, note: str = "") -> RunResult:
    bench, gt = run_dir / "bench", run_dir / "gt.tum"
    est = bench / "est.tum"
    try:
        al = align(est, gt)
    except (OSError, ValueError) as e:
        return _empty(key, "lost", f"no usable estimate: {e}", run_dir, wall_s)
    dt = np.diff(np.sort(read_tum_stamps_ns(est))) * 1e-9
    covered_s = float(dt[dt <= MAX_GAP_S].sum())
    if covered_s < MIN_COVERAGE * sc.duration_s:
        reason = f"estimate covers {covered_s:.1f} s of {sc.duration_s:.0f} s"
        return _empty(key, "lost", reason, run_dir, wall_s)
    err = np.linalg.norm(al.gt - al.est_aligned, axis=1)
    _write_errors(run_dir / "errors_online.csv", est, gt)
    ate_opt = None
    opt = bench / "kf_opt.tum"
    try:
        ate_opt = ate(opt, gt).rmse_m
        _write_errors(run_dir / "errors_opt.csv", opt, gt)
    except (OSError, ValueError):
        pass
    loops = None
    if (bench / "bench.json").is_file():
        loops = _find(json.loads((bench / "bench.json").read_text()), "n_loops_accepted")
    return RunResult(
        key=key,
        status="ok",
        reason=note,
        ate_online_m=float(np.sqrt(np.mean(err * err))),
        ate_opt_m=ate_opt,
        rpe10_m=rpe_t(al.est_aligned, al.gt, 10.0),
        rpe50_m=rpe_t(al.est_aligned, al.gt, 50.0),
        n_loops_accepted=None if loops is None else int(loops),
        wall_s=wall_s,
        run_dir=run_dir,
    )


STEPS = ("noise", "faults", "convert", "bench", "score")  # what a running run reports


def _step(run_dir: Path, name: str) -> None:
    """Record the step a running run is in (``stage.txt``), read back by list_runs."""
    (run_dir / "stage.txt").write_text(f"step {STEPS.index(name) + 1}/{len(STEPS)}: {name}")


def _execute(
    spec: ExperimentSpec,
    sc: Scenario,
    raw: Path,
    run_dir: Path,
    key: str,
    bench_cmd: str,
    timeout_s: float,
) -> RunResult:
    t_start = time.monotonic()
    tmp = run_dir / "tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    shutil.rmtree(run_dir / "bench", ignore_errors=True)
    tmp.mkdir()
    seed = sc.seed if spec.seed is None else spec.seed
    overlay = write_overlay(sc, spec.factors, run_dir / "overlay.yaml", spec.variant)
    profile = load_profile(sc.profile)
    _step(run_dir, "noise")
    run_bridge(
        raw,
        sc,
        scale_profile(profile, spec.noise),
        tmp / "bridged",
        seed=seed,
        imu_extra=imu_extra(profile.imu, spec.noise.imu),
        lidar_extra_sigma_m=lidar_extra_sigma(profile.lidar, spec.noise.lidar),
        believed_gnss=profile.gnss,
    )
    shutil.copyfile(tmp / "bridged" / "gt.tum", run_dir / "gt.tum")
    bag = tmp / "bridged" / "bag"
    if spec.faults:
        t0 = int(read_tum_stamps_ns(run_dir / "gt.tum").min())
        _step(run_dir, "faults")
        apply_faults(bag, tmp / "faulted", spec.faults, t0, seed)
        bag = tmp / "faulted"
    _step(run_dir, "convert")
    convert(bag, tmp / "mssdata", origin_lla=sc.origin_lla, pipeline_config=overlay)
    cmd = [
        *shlex.split(bench_cmd),
        str(DEFAULT_CONFIG),
        str((tmp / "mssdata").resolve()),
        "--overlay",
        str(overlay.resolve()),
        "--out",
        str((run_dir / "bench").resolve()),
    ]
    _step(run_dir, "bench")
    with (run_dir / "log.txt").open("w") as log:
        try:
            proc = subprocess.run(
                cmd, stdout=log, stderr=subprocess.STDOUT, timeout=timeout_s, check=False
            )
        except subprocess.TimeoutExpired:
            wall = time.monotonic() - t_start
            return _empty(key, "failed", f"bench timeout after {timeout_s:.0f} s", run_dir, wall)
    wall = time.monotonic() - t_start
    if proc.returncode not in (0, 3):
        return _empty(key, "failed", f"bench exit {proc.returncode} (see log.txt)", run_dir, wall)
    if not (run_dir / "bench" / "est.tum").is_file():
        reason = f"bench exit {proc.returncode} without est.tum (see log.txt)"
        return _empty(key, "failed", reason, run_dir, wall)
    note = "bench exit 3: some records were not processed" if proc.returncode == 3 else ""
    _step(run_dir, "score")
    return _score(key, run_dir, sc, wall, note)


def run_experiment(
    spec: ExperimentSpec,
    *,
    bench_cmd: str,
    root: Path = REPO / "results/explorer",
    timeout_s: float | None = None,
) -> RunResult:
    """Run ``spec`` (or return its cached result); ``SpecError`` for an unusable spec."""
    try:
        sc = load_scenario(spec.scenario)
    except (OSError, ValueError) as e:
        raise SpecError(f"scenario: {e}") from e
    raw = check_recording(spec, sc)
    key = spec_key(spec, run_fingerprint(spec, bench_cmd))
    run_dir = Path(root) / "runs" / key
    run_dir.mkdir(parents=True, exist_ok=True)
    if not _lock(run_dir):
        return _empty(key, "running", "another process is running this spec", run_dir)
    try:
        cached = _load(run_dir)
        if cached is not None and cached.status in _CACHED:
            return cached
        dump_spec(spec, run_dir / "spec.yaml")
        limit = timeout_s if timeout_s is not None else TIMEOUT_PER_SCENARIO_S * sc.duration_s
        try:
            result = _execute(spec, sc, raw, run_dir, key, bench_cmd, limit)
        except SpecError:
            raise
        except Exception as e:  # a broken step must not leave a result that looks valid
            result = _empty(key, "failed", f"{type(e).__name__}: {e}", run_dir)
        _save(result)
        return result
    finally:
        shutil.rmtree(run_dir / "tmp", ignore_errors=True)
        (run_dir / "stage.txt").unlink(missing_ok=True)
        (run_dir / ".lock").unlink(missing_ok=True)


def list_runs(root: Path) -> list[tuple[ExperimentSpec, RunResult]]:
    """Every run under ``root/runs`` with a readable spec, in start order; a run in progress
    (live lock, no result yet) is reported as ``running``."""
    out = []
    runs = Path(root) / "runs"
    dirs = sorted(runs.iterdir(), key=lambda p: p.stat().st_mtime) if runs.is_dir() else []
    for d in dirs:
        try:
            spec = load_spec(d / "spec.yaml")
        except SpecError:
            continue
        result = _load(d)
        lock = d / ".lock"
        if lock.exists():
            try:
                alive = _pid_alive(int(lock.read_text().strip() or "0"))
            except (OSError, ValueError):
                alive = False
            if alive:
                result = _running(d, lock)
        if result is not None:
            out.append((spec, replace(result, run_dir=d)))
    return out


def _running(run_dir: Path, lock: Path) -> RunResult:
    """A run in progress: reason = its step (``stage.txt``), wall_s = time since it started."""
    try:
        step = (run_dir / "stage.txt").read_text().strip()
        started = lock.stat().st_mtime
    except OSError:
        step, started = "starting", time.time()
    return _empty(run_dir.name, "running", step, run_dir, max(0.0, time.time() - started))


def run_states(root: Path) -> tuple[tuple[str, str], ...]:
    """(key, status) of every listed run: changes only when a run starts or finishes, so a
    page polling it redraws its results only then."""
    return tuple((r.key, r.status) for _, r in list_runs(root))
