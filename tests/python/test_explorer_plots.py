"""Explorer figures, comparison table and report export (needs the ``explorer`` extra)."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("plotly")

from conftest import _REPO  # noqa: E402

from mss_explorer.plots import (  # noqa: E402
    contribution_figure,
    error_figure,
    export_report,
    metrics_table,
    trajectory_figure,
)
from mss_explorer.runner import RunResult  # noqa: E402
from mss_explorer.spec import ExperimentSpec  # noqa: E402
from mss_sim.faults import Fault  # noqa: E402


def _tum(path: Path, t: np.ndarray, xyz: np.ndarray) -> None:
    rows = [f"{a:.9f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1" for a, (x, y, z) in zip(t, xyz, strict=True)]
    path.write_text("\n".join(rows) + "\n")


def _run(
    tmp: Path,
    key: str,
    label: str,
    ate: float | None,
    status: str = "ok",
    faults: tuple[Fault, ...] = (),
) -> tuple[ExperimentSpec, RunResult]:
    d = tmp / key
    (d / "bench").mkdir(parents=True)
    t = 10.0 + np.arange(60, dtype=np.float64)
    gt = np.column_stack([np.cos(t / 10), np.sin(t / 10), np.zeros_like(t)]) * 10
    _tum(d / "gt.tum", t, gt)
    _tum(d / "bench" / "est.tum", t, gt + (ate or 0.0))
    (d / "errors_online.csv").write_text(
        "t_s,err_m\n" + "\n".join(f"{i:.1f},{ate or 0.0:.3f}" for i in range(60)) + "\n"
    )
    spec = ExperimentSpec(
        scenario=_REPO / "sim/scenarios/ROOM-01.yaml", recording=tmp, faults=faults, label=label
    )
    r = RunResult(key, status, "", ate, ate, None, None, 1, 1.0, d)  # type: ignore[arg-type]
    return spec, r


def test_trajectory_figure_has_gt_and_runs(tmp_path: Path) -> None:
    runs = [_run(tmp_path, "a", "ref", 0.1), _run(tmp_path, "b", "gnss", 0.2)]
    fig = trajectory_figure(runs, runs[0][1].run_dir / "gt.tum", t_cursor_s=5.0)
    names = {tr.name for tr in fig.data}
    assert {"ground truth", "ref", "gnss"} <= names


def test_trajectory_figure_dashes_fault_segment(tmp_path: Path) -> None:
    runs = [_run(tmp_path, "a", "out", 0.1, faults=(Fault("gnss", "outage", 10.0, 20.0),))]
    fig = trajectory_figure(runs, runs[0][1].run_dir / "gt.tum")
    dashed = [tr for tr in fig.data if tr.line.dash == "dash"]
    assert len(dashed) == 1 and len(dashed[0].x) == 10


def test_error_figure_shades_faults(tmp_path: Path) -> None:
    faults = (Fault("gnss", "outage", 10.0, 20.0), Fault("lidar", "blackout", 30.0, 35.0))
    runs = [_run(tmp_path, "a", "x", 0.1, faults=faults), _run(tmp_path, "b", "y", 0.1)]
    fig = error_figure(runs)
    rects = [s for s in fig.layout.shapes if s.type == "rect"]
    assert len(rects) == 2
    labels = {a.text for a in fig.layout.annotations}
    assert {"gnss:outage", "lidar:blackout"} <= labels


def test_metrics_table_lost_and_delta(tmp_path: Path) -> None:
    runs = [
        _run(tmp_path, "a", "ref", 0.5),
        _run(tmp_path, "b", "worse", 1.0),
        _run(tmp_path, "c", "lost", None, status="lost"),
    ]
    rows = {r["label"]: r for r in metrics_table(runs, ref_key="a")}
    assert rows["worse"]["ΔATE %"] == pytest.approx(100.0)
    assert rows["ref"]["ΔATE %"] == pytest.approx(0.0) and rows["ref"]["ref"]
    assert rows["lost"]["status"] == "lost" and rows["lost"]["ATE opt [m]"] == "—"
    assert rows["lost"]["ΔATE %"] == "—"


def test_contribution_excludes_ref_and_failed(tmp_path: Path) -> None:
    runs = [
        _run(tmp_path, "a", "ref", 0.5),
        _run(tmp_path, "b", "worse", 1.0),
        _run(tmp_path, "c", "lost", None, status="lost"),
    ]
    fig = contribution_figure(metrics_table(runs, ref_key="a"))
    assert list(fig.data[0].y) == ["worse"]
    assert list(fig.data[0].x) == pytest.approx([100.0])


def test_export_report_writes_files(tmp_path: Path) -> None:
    runs = [_run(tmp_path, "a", "ref", 0.5), _run(tmp_path, "b", "worse", 1.0)]
    md = export_report(runs, ref_key="a", out_dir=tmp_path / "report")
    text = md.read_text()
    assert md.name == "summary.md" and "ref" in text and "worse" in text
    assert (md.parent / "trajectories.png").stat().st_size > 0
    assert (md.parent / "errors.png").stat().st_size > 0


def test_export_report_refuses_source_tree(tmp_path: Path) -> None:
    runs = [_run(tmp_path, "a", "ref", 0.5)]
    with pytest.raises(ValueError, match="results"):
        export_report(runs, ref_key="a", out_dir=_REPO / "docs" / "report")


def test_separate_fault_windows_not_joined(tmp_path: Path) -> None:
    faults = (Fault("gnss", "outage", 10.0, 15.0), Fault("lidar", "blackout", 30.0, 35.0))
    runs = [_run(tmp_path, "a", "two", 0.1, faults=faults)]
    fig = trajectory_figure(runs, runs[0][1].run_dir / "gt.tum")
    dashed = [tr for tr in fig.data if tr.line.dash == "dash"]
    assert len(dashed) == 1 and None in list(dashed[0].x)


def test_delta_uses_same_ate_kind_for_both(tmp_path: Path) -> None:
    ref = _run(tmp_path, "a", "ref", 0.5)
    run = _run(tmp_path, "b", "no opt", 1.0)
    run = (run[0], replace(run[1], ate_opt_m=None, ate_online_m=1.0))
    ref = (ref[0], replace(ref[1], ate_opt_m=0.25, ate_online_m=0.5))
    rows = {r["label"]: r for r in metrics_table([ref, run], ref_key="a")}
    assert rows["no opt"]["ΔATE %"] == pytest.approx(100.0)  # online vs online, not vs opt


def test_metrics_table_has_reason(tmp_path: Path) -> None:
    lost = _run(tmp_path, "c", "lost", None, status="lost")
    lost = (lost[0], replace(lost[1], reason="estimate covers 1.0 s of 6 s"))
    rows = metrics_table([_run(tmp_path, "a", "ref", 0.5), lost], ref_key="a")
    assert rows[1]["reason"] == "estimate covers 1.0 s of 6 s"


def test_export_includes_lost_and_failed(tmp_path: Path) -> None:
    lost = _run(tmp_path, "c", "gone", None, status="lost")
    lost = (lost[0], replace(lost[1], reason="no usable estimate"))
    md = export_report([_run(tmp_path, "a", "ref", 0.5), lost], "a", tmp_path / "rep")
    text = md.read_text()
    assert "gone" in text and "no usable estimate" in text


def test_trajectory_figure_loop_markers(tmp_path: Path) -> None:
    run = _run(tmp_path, "a", "ref", 0.1)
    t_ns = [round((10.0 + 5) * 1e9), round((10.0 + 50) * 1e9)]
    (run[1].run_dir / "bench" / "loops.csv").write_text(
        f"t_i_ns,t_j_ns,state\n{t_ns[0]},{t_ns[1]},released\n"
    )
    fig = trajectory_figure([run], run[1].run_dir / "gt.tum")
    loops = [tr for tr in fig.data if tr.name == "ref loops"]
    assert len(loops) == 1 and len([x for x in loops[0].x if x is not None]) == 2
