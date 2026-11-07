"""Sensor impact explorer page: ``streamlit run tools/mss_explorer/app.py [-- --root DIR]``.

Pick a clean recording under ``<root>/recordings/<name>/`` (``raw_bag/``; the scenario is
``sim/scenarios/<name>.yaml`` unless the recording holds a ``scenario.yaml``), toggle the GNSS,
wheel odometry and loop closure factors, scale sensor noise, add timed faults and run. Runs are
cached by mss_explorer.runner and compared side by side. Root: ``--root``, else the
``MSS_EXPLORER_ROOT`` environment variable, else ``results/explorer``. Bench command:
``MSS_BENCH_CMD``. Serves localhost only (ADR-032).
"""

from __future__ import annotations

import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
for _d in ("tools", "sim", "eval"):
    if str(_REPO / _d) not in sys.path:
        sys.path.insert(0, str(_REPO / _d))

import pandas as pd  # noqa: E402
import streamlit as st  # noqa: E402
import yaml  # noqa: E402

from mss_explorer import runner  # noqa: E402
from mss_explorer.plots import (  # noqa: E402
    COLUMNS,
    contribution_figure,
    error_figure,
    export_report,
    metrics_table,
    trajectory_figure,
)
from mss_explorer.runner import list_runs  # noqa: E402
from mss_explorer.spec import FACTORS, SpecError, spec_from_dict  # noqa: E402
from mss_sim.faults import KINDS, Fault, validate_fault  # noqa: E402
from mss_sim.noise_scale import NoiseScale  # noqa: E402
from mss_sim.scenario import load_scenario  # noqa: E402

MAX_PARALLEL = 2  # bench processes at once (WSL has about 10 GB RAM)
POLL_S = 3.0  # status refresh period while a run is queued or running [s]


def _root() -> Path:
    args = sys.argv[1:]
    if "--root" in args and args.index("--root") + 1 < len(args):
        return Path(args[args.index("--root") + 1]).resolve()
    return Path(os.environ.get("MSS_EXPLORER_ROOT", _REPO / "results/explorer")).resolve()


@st.cache_resource
def _executor() -> ThreadPoolExecutor:
    return ThreadPoolExecutor(max_workers=MAX_PARALLEL)


def _recordings(root: Path) -> dict[str, tuple[Path, Path]]:
    """name -> (recording dir, scenario file), for directories that hold ``raw_bag/``."""
    out = {}
    base = root / "recordings"
    for d in sorted(base.iterdir()) if base.is_dir() else []:
        scenario = d / "scenario.yaml"
        if not scenario.is_file():
            scenario = _REPO / "sim/scenarios" / f"{d.name}.yaml"
        if (d / "raw_bag").is_dir() and scenario.is_file():
            out[d.name] = (d, scenario)
    return out


def _scenario_factors(scenario: Path, variant: str | None) -> dict[str, bool]:
    sc = load_scenario(scenario)
    path = sc.pipeline_config if variant is None else sc.pipeline_config_variants[variant]
    doc = yaml.safe_load(path.read_text()) or {}
    params = doc.get("mss_node", {}).get("ros__parameters", {})
    return {f: bool(params.get(f, {}).get("enabled", False)) for f in FACTORS}


def _sidebar(root: Path, bench_cmd: str) -> None:
    recs = _recordings(root)
    st.sidebar.header("Experiment")
    if not recs:
        st.sidebar.warning(f"No recording under {root / 'recordings'} (tools/record_scenario.sh)")
        return
    name = st.sidebar.selectbox("Recording", list(recs), key="recording")
    rec, scenario = recs[name]
    duration_s = load_scenario(scenario).duration_s
    st.sidebar.subheader("Factors")
    st.sidebar.caption("LiDAR and IMU are the backbone and stay on.")
    variants = [None, *sorted(load_scenario(scenario).pipeline_config_variants)]
    variant = st.sidebar.selectbox(
        "Overlay", variants, format_func=lambda v: v or "default", key="variant"
    )
    defaults = _scenario_factors(scenario, variant)
    factors = {
        f: st.sidebar.toggle(f, value=defaults[f], key=f"factor_{variant}_{f}") for f in FACTORS
    }
    st.sidebar.subheader("Noise (x the recorded profile)")
    st.sidebar.caption("LiDAR and IMU start at 1x: Gazebo already put that noise in the recording.")
    noise = NoiseScale(
        lidar=st.sidebar.slider("LiDAR range (from 1x)", 1.0, 20.0, 1.0, 0.5, key="n_lidar"),
        imu=st.sidebar.slider("IMU (from 1x)", 1.0, 20.0, 1.0, 0.5, key="n_imu"),
        gnss=st.sidebar.slider("GNSS", 0.0, 20.0, 1.0, 0.5, key="n_gnss"),
        odom=st.sidebar.slider("Wheel odometry", 0.0, 20.0, 1.0, 0.5, key="n_odom"),
    )
    _fault_editor(duration_s)
    label = st.sidebar.text_input("Label", key="label")
    if st.sidebar.button("Run", key="run", type="primary", disabled=not bench_cmd):
        doc = {
            "scenario": str(scenario),
            "recording": str(rec),
            "variant": variant,
            "factors": factors,
            "noise": vars(noise),
            "faults": [vars(f) for f in st.session_state.faults],
            "label": label,
        }
        try:
            spec = spec_from_dict(doc)
            runner.check_recording(spec, load_scenario(spec.scenario))
        except SpecError as e:
            st.sidebar.error(str(e))
            return
        job = _executor().submit(runner.run_experiment, spec, bench_cmd=bench_cmd, root=root)
        st.session_state.setdefault("jobs", []).append((label or "(no label)", job))
        time.sleep(0.3)  # let the worker take the lock, so the run lists as running


def _fault_editor(duration_s: float) -> None:
    st.sidebar.subheader("Faults")
    faults: list[Fault] = st.session_state.setdefault("faults", [])
    sensor = st.sidebar.selectbox("Sensor", list(KINDS), key="f_sensor")
    kind = st.sidebar.selectbox("Kind", list(KINDS[sensor]), key="f_kind")
    spec = KINDS[sensor][kind]
    c1, c2 = st.sidebar.columns(2)
    t0 = c1.number_input("t start [s]", 0.0, None, 0.0, key="f_start")
    t1 = c2.number_input("t end [s]", 0.0, None, min(10.0, duration_s), key="f_end")
    mag = None
    if spec.unit:
        mag = st.sidebar.number_input(f"Magnitude [{spec.unit}]", value=spec.default, key="f_mag")
    if st.sidebar.button("Add fault", key="f_add"):
        try:
            faults.append(validate_fault(Fault(sensor, kind, t0, t1, mag), duration_s))
        except ValueError as e:
            st.sidebar.error(str(e))
    for i, f in enumerate(list(faults)):
        unit = f" {f.magnitude:g} {KINDS[f.sensor][f.kind].unit}" if f.magnitude is not None else ""
        cols = st.sidebar.columns([4, 1])
        cols[0].write(f"{f.sensor}:{f.kind}{unit} [{f.t_start_s:g}, {f.t_end_s:g}) s")
        if cols[1].button("✕", key=f"f_del_{i}"):
            faults.pop(i)
            st.rerun()


def _progress(root: Path) -> bool:
    """One progress bar per run in progress (its step and time so far). True if any."""
    running = [(s, r) for s, r in list_runs(root) if r.status == "running"]
    for s, r in running:
        name = r.reason.rsplit(": ", 1)[-1]
        done = runner.STEPS.index(name) / len(runner.STEPS) if name in runner.STEPS else 0.0
        text = f"Running `{s.label or r.key}`: {r.reason or 'starting'}, {r.wall_s:.0f} s so far"
        st.progress(done, text=text)
    return bool(running)


def _jobs(any_running: bool) -> bool:
    """Runs submitted from this page that have no progress bar yet (waiting for a worker, or
    starting); a run that raised stays shown until cleared. True while any is unfinished."""
    jobs = st.session_state.setdefault("jobs", [])
    errors: list[str] = st.session_state.setdefault("job_errors", [])
    pending = []
    for label, job in jobs:
        if not job.done():
            pending.append((label, job))
            if not job.running():
                st.markdown(f"- `{label}`: waiting for a free worker")
            elif not any_running:
                st.markdown(f"- `{label}`: starting")
        elif job.exception() is not None:
            errors.append(f"{label}: {type(job.exception()).__name__}: {job.exception()}")
    st.session_state["jobs"] = pending
    for message in errors:
        st.error(message)
    if errors and st.button("Clear errors", key="clear_errors"):
        errors.clear()
        st.rerun()
    return bool(pending)


def _results(root: Path) -> None:
    runs = list_runs(root)
    if not runs:
        st.info("No run yet: set up an experiment in the sidebar and press Run.")
        return
    names = {r.key: (s.label or r.key) for s, r in runs}
    keys = st.multiselect(
        "Runs", list(names), default=list(names), format_func=names.get, key="shown"
    )
    shown = [(s, r) for s, r in runs if r.key in keys]
    if not shown:
        return
    ok = [(s, r) for s, r in shown if r.status == "ok"]
    n_running = sum(r.status == "running" for _, r in shown)
    if n_running:
        st.warning(
            f"{n_running} run(s) still running: their rows have no numbers yet, and the plots "
            "show finished runs only. The page redraws when a run finishes."
        )
    ref_key = st.selectbox("Reference", [r.key for _, r in shown], format_func=names.get, key="ref")
    rows = metrics_table(shown, ref_key)
    st.dataframe(pd.DataFrame(rows, columns=list(COLUMNS)), hide_index=True)
    if ok:
        duration = max(load_scenario(s.scenario).duration_s for s, _ in ok)
        t_cursor = st.slider("Time [s]", 0.0, float(duration), 0.0, 1.0, key="cursor")
        left, right = st.columns(2)
        gt = ok[0][1].run_dir / "gt.tum"
        left.plotly_chart(trajectory_figure(ok, gt, t_cursor), key="traj")
        which = right.radio("Error", ["online", "opt"], horizontal=True, key="which")
        right.plotly_chart(error_figure(ok, which), key="err")
        st.plotly_chart(contribution_figure(rows), key="contrib")
    if st.button("Export report", key="export"):
        out = root / "reports" / time.strftime("%Y%m%d-%H%M%S")
        st.success(f"Report written: {export_report(shown, ref_key, out)}")


def main() -> None:
    st.set_page_config(page_title="Sensor impact explorer", layout="wide")
    st.title("Sensor impact explorer")
    root = _root()
    bench_cmd = os.environ.get("MSS_BENCH_CMD", "")
    if not bench_cmd:
        st.error(
            "MSS_BENCH_CMD is not set: the bench command, e.g. "
            "'env LD_LIBRARY_PATH=<gtsam lib> <build>/bench/mss_bench'."
        )
    _sidebar(root, bench_cmd)
    shown = runner.run_states(root)
    pending = any(not j.done() for _, j in st.session_state.get("jobs", []))
    busy = pending or any(status == "running" for _, status in shown)

    # Only this small status block polls; the table and plots below are redrawn by a full
    # rerun when a run starts or finishes, so they do not flicker while a run is going.
    @st.fragment(run_every=POLL_S if busy else None)
    def status() -> None:
        still_pending = _jobs(_progress(root))
        if busy and (runner.run_states(root) != shown or pending != still_pending):
            st.rerun(scope="app")

    status()
    _results(root)


main()
