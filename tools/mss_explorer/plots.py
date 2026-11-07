"""Comparison figures (Plotly), the metrics table and the report export (matplotlib PNGs).

Plotly is imported inside the figure functions, so ``import mss_explorer`` works without the
``explorer`` extra. The ATE compared between runs is the optimised one (``kf_opt.tum``, where
loop closure acts) when both runs have it, else the online one (``est.tum``).
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from mss_eval.ate import Alignment, align, read_tum_stamps_ns
from mss_eval.tum import read_tum
from mss_explorer.metrics import delta_ate_pct
from mss_explorer.runner import RunResult
from mss_explorer.spec import REPO, ExperimentSpec, spec_to_dict
from mss_sim.faults import Fault

Run = tuple[ExperimentSpec, RunResult]
DASH = "—"
COLUMNS = (
    "label",
    "status",
    "ATE online [m]",
    "ATE opt [m]",
    "RPE10 [m]",
    "RPE50 [m]",
    "loops",
    "ΔATE %",
    "reason",
)
MIN_BOX_HEIGHT_M = 0.5  # world boxes thinner than this (floor, ceiling slabs) are not drawn
MAP_SLICE_Z_M = (0.2, 2.5)  # map points drawn: this height band in the gt frame, m
MAX_MAP_POINTS = 30_000  # map points drawn at most (random subset, fixed seed)


def _name(run: Run) -> str:
    return run[0].label or run[1].key


def _delta(ref: RunResult, r: RunResult) -> float | str:
    """ATE change of ``r`` against ``ref`` with the same kind for both: optimised when both have
    it, else online."""
    if ref.status != "ok" or r.status != "ok":
        return DASH
    if ref.ate_opt_m is not None and r.ate_opt_m is not None:
        a, b = ref.ate_opt_m, r.ate_opt_m
    else:
        a, b = ref.ate_online_m, r.ate_online_m
    if a is None or b is None or a == 0:
        return DASH
    return round(delta_ate_pct(a, b), 1)


def _segments(x: np.ndarray, y: np.ndarray, mask: np.ndarray) -> tuple[list, list]:
    """Points where ``mask``, with None between separate runs of it (no joining line)."""
    xs: list = []
    ys: list = []
    for i in np.flatnonzero(mask):
        if xs and not mask[i - 1]:
            xs.append(None)
            ys.append(None)
        xs.append(float(x[i]))
        ys.append(float(y[i]))
    return xs, ys


def _loops(r: RunResult) -> np.ndarray:
    """(t_i_ns, t_j_ns) of the bench's loops.csv, active and released only."""
    path = r.run_dir / "bench" / "loops.csv"
    if not path.is_file():
        return np.zeros((0, 2), dtype=np.int64)
    rows = [line.split(",") for line in path.read_text().splitlines()[1:] if line.strip()]
    keep = [(int(a), int(b)) for a, b, state in rows if state != "removed"]
    return np.array(keep, dtype=np.int64).reshape(-1, 2)


def _in_faults(t_s: np.ndarray, faults: Sequence[Fault]) -> np.ndarray:
    mask = np.zeros(len(t_s), dtype=bool)
    for f in faults:
        mask |= (t_s >= f.t_start_s) & (t_s < f.t_end_s)
    return mask


def _aligned(r: RunResult) -> tuple[np.ndarray, Alignment] | None:
    """(t_s from the first gt stamp, alignment) of the online estimate, or None."""
    gt = r.run_dir / "gt.tum"
    try:
        al = align(r.run_dir / "bench" / "est.tum", gt)
    except (OSError, ValueError):
        return None
    return (al.t_ns - int(read_tum_stamps_ns(gt).min())) * 1e-9, al


def _errors(al: Alignment) -> np.ndarray:
    """Position error of every pair [m], the same as errors_online.csv."""
    return np.linalg.norm(al.gt - al.est_aligned, axis=1)


def map_points(r: RunResult) -> np.ndarray | None:
    """(N, 3) points of the run's ``map.pcd`` in the gt frame, cut to ``MAP_SLICE_Z_M`` and at
    most ``MAX_MAP_POINTS``; None without a map. The map is in w at the optimised poses, so it
    is moved by the rigid fit of ``kf_opt.tum`` (of ``est.tum`` when that fit is not possible)."""
    from mss_eval.map_png import read_pcd

    try:
        pts = read_pcd(r.run_dir / "map.pcd")
    except (OSError, ValueError):
        return None
    al = None
    for est in ("kf_opt.tum", "est.tum"):
        try:
            al = align(r.run_dir / "bench" / est, r.run_dir / "gt.tum")
            break
        except (OSError, ValueError):
            continue
    if al is None:
        return None
    p = pts @ al.rot.T + al.t
    p = p[(p[:, 2] > MAP_SLICE_Z_M[0]) & (p[:, 2] < MAP_SLICE_Z_M[1])]
    if len(p) > MAX_MAP_POINTS:
        p = p[np.random.default_rng(0).choice(len(p), MAX_MAP_POINTS, replace=False)]
    return p


def _world_shapes(fig: Any, world_sdf: Path) -> None:
    """The world's collision boxes, top view, as grey rectangles under the traces."""
    from gen_sim_assets import world_boxes

    for b in world_boxes(world_sdf):
        if b.hi[2] - b.lo[2] < MIN_BOX_HEIGHT_M:
            continue
        fig.add_shape(
            type="rect",
            x0=float(b.lo[0]),
            x1=float(b.hi[0]),
            y0=float(b.lo[1]),
            y1=float(b.hi[1]),
            fillcolor="rgba(120,120,120,0.35)",
            line_width=0,
            layer="below",
        )


def _error_vectors(t: np.ndarray, al: Alignment, every_s: float) -> tuple[list, list]:
    """Estimate-to-truth segments, one every ``every_s`` seconds, None-separated."""
    vx: list = []
    vy: list = []
    next_t = -np.inf
    for i in range(len(t)):
        if t[i] < next_t:
            continue
        next_t = t[i] + every_s
        vx += [float(al.est_aligned[i, 0]), float(al.gt[i, 0]), None]
        vy += [float(al.est_aligned[i, 1]), float(al.gt[i, 1]), None]
    return vx, vy


def trajectory_figure(
    runs: Sequence[Run],
    gt: Path,
    t_cursor_s: float | None = None,
    *,
    color_by_error: bool = False,
    error_vectors_s: float | None = None,
    world_sdf: Path | None = None,
    map_run: Run | None = None,
) -> Any:
    """Top view: ground truth, one trace per run (fault segments dashed), cursor markers.

    ``color_by_error`` colours each run's poses by position error, on one scale for all runs;
    ``error_vectors_s`` draws an estimate-to-truth segment every that many seconds;
    ``world_sdf`` draws the world's boxes underneath; ``map_run`` draws that run's map. Every
    run point carries its time (``customdata``, s), so a click on it selects that time."""
    import plotly.graph_objects as go

    fig = go.Figure()
    if world_sdf is not None:
        _world_shapes(fig, world_sdf)
    if map_run is not None:
        m = map_points(map_run[1])
        if m is not None:
            fig.add_trace(
                go.Scattergl(
                    x=m[:, 0],
                    y=m[:, 1],
                    name=f"{_name(map_run)} map",
                    mode="markers",
                    marker={"size": 2, "color": "rgba(40,90,160,0.45)"},
                    hoverinfo="skip",
                )
            )
    g = read_tum(gt)
    g_t = (read_tum_stamps_ns(gt) - int(read_tum_stamps_ns(gt).min())) * 1e-9
    fig.add_trace(
        go.Scatter(
            x=g.p[:, 0],
            y=g.p[:, 1],
            name="ground truth",
            mode="lines",
            line={"color": "black", "width": 3},
            customdata=g_t,
            hovertemplate="t %{customdata:.1f} s",
        )
    )
    aligned = [(run, got) for run in runs if (got := _aligned(run[1])) is not None]
    if color_by_error and aligned:
        top = max(float(_errors(al).max()) for _, (_, al) in aligned)
        fig.update_layout(
            coloraxis={
                "cmin": 0.0,
                "cmax": top,
                "colorscale": "Turbo",
                "colorbar": {"title": {"text": "error [m]"}},
            }
        )
    for run, (t, al) in aligned:
        p = al.est_aligned
        err = _errors(al)
        trace: dict[str, Any] = {
            "x": p[:, 0],
            "y": p[:, 1],
            "name": _name(run),
            "customdata": t,
            "text": [f"{e:.3f} m" for e in err],
            "hovertemplate": "t %{customdata:.1f} s<br>error %{text}",
        }
        if color_by_error:
            trace |= {
                "mode": "lines+markers",
                "line": {"color": "rgba(90,90,90,0.4)", "width": 1},
                "marker": {"size": 5, "color": err, "coloraxis": "coloraxis"},
            }
        else:
            trace["mode"] = "lines"
        fig.add_trace(go.Scatter(**trace))
        if error_vectors_s:
            vx, vy = _error_vectors(t, al, error_vectors_s)
            fig.add_trace(
                go.Scatter(
                    x=vx,
                    y=vy,
                    name=f"{_name(run)} error",
                    mode="lines",
                    line={"width": 1, "color": "crimson"},
                    hoverinfo="skip",
                )
            )
        bad = _in_faults(t, run[0].faults)
        if bad.any():
            xs, ys = _segments(p[:, 0], p[:, 1], bad)
            fig.add_trace(
                go.Scatter(
                    x=xs,
                    y=ys,
                    name=f"{_name(run)} (fault)",
                    mode="lines",
                    line={"dash": "dash", "width": 4},
                )
            )
        loops = _loops(run[1])
        if len(loops):
            t0_ns = int(read_tum_stamps_ns(run[1].run_dir / "gt.tum").min())
            idx = np.clip(np.searchsorted(t, (loops - t0_ns) * 1e-9), 0, len(t) - 1)
            lx: list = []
            ly: list = []
            for i, j in idx:
                lx += [float(p[i, 0]), float(p[j, 0]), None]
                ly += [float(p[i, 1]), float(p[j, 1]), None]
            fig.add_trace(
                go.Scatter(
                    x=lx,
                    y=ly,
                    name=f"{_name(run)} loops",
                    mode="lines+markers",
                    line={"width": 1},
                    marker={"size": 6},
                )
            )
        if t_cursor_s is not None:
            i = int(np.clip(np.searchsorted(t, t_cursor_s), 0, len(t) - 1))
            fig.add_trace(
                go.Scatter(
                    x=[p[i, 0]],
                    y=[p[i, 1]],
                    mode="markers",
                    showlegend=False,
                    hoverinfo="skip",
                    marker={"size": 14, "symbol": "circle-open", "line": {"width": 3}},
                )
            )
    fig.update_layout(
        xaxis_title="x [m]",
        yaxis_title="y [m]",
        yaxis_scaleanchor="x",
        margin={"l": 40, "r": 10, "t": 30, "b": 40},
        # Below the plot: on the right it would sit on the error colour bar.
        legend={"orientation": "h", "y": -0.15, "yanchor": "top", "x": 0.0},
    )
    return fig


def selected_time(selection: dict | None) -> float | None:
    """The time [s] (``customdata``) of the first selected point that carries one; None when
    no such point is selected (map points and world boxes carry no time)."""
    points = ((selection or {}).get("selection") or {}).get("points") or []
    for pt in points:
        value = pt.get("customdata")
        if isinstance(value, list | tuple):
            value = value[0] if value else None
        if value is not None:
            return float(value)
    return None


def error_figure(
    runs: Sequence[Run], which: str = "online", t_cursor_s: float | None = None
) -> Any:
    """Position error over time per run; each distinct fault window shaded and labelled; a
    dotted vertical line at ``t_cursor_s``."""
    import plotly.graph_objects as go

    fig = go.Figure()
    for run in runs:
        path = run[1].run_dir / f"errors_{which}.csv"
        if not path.is_file():
            continue
        data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
        fig.add_trace(
            go.Scatter(
                x=data[:, 0], y=data[:, 1], customdata=data[:, 0], name=_name(run), mode="lines"
            )
        )
    seen = set()
    for run in runs:
        for f in run[0].faults:
            key = (f.sensor, f.kind, f.t_start_s, f.t_end_s)
            if key in seen:
                continue
            seen.add(key)
            fig.add_vrect(
                x0=f.t_start_s,
                x1=f.t_end_s,
                fillcolor="red",
                opacity=0.12,
                line_width=0,
                annotation_text=f"{f.sensor}:{f.kind}",
                annotation_position="top left",
            )
    if t_cursor_s is not None:
        fig.add_vline(x=t_cursor_s, line_width=2, line_dash="dot", line_color="black")
    fig.update_layout(
        xaxis_title="t [s]",
        yaxis_title="position error [m]",
        margin={"l": 40, "r": 10, "t": 30, "b": 40},
    )
    return fig


def _fmt(v: float | None) -> float | str:
    return DASH if v is None else round(float(v), 3)


def metrics_table(runs: Sequence[Run], ref_key: str) -> list[dict]:
    ref = next((r for _, r in runs if r.key == ref_key), None)
    rows = []
    for run in runs:
        r = run[1]
        delta = DASH if ref is None else _delta(ref, r)
        rows.append(
            {
                "label": _name(run),
                "status": r.status,
                "ATE online [m]": _fmt(r.ate_online_m),
                "ATE opt [m]": _fmt(r.ate_opt_m),
                "RPE10 [m]": _fmt(r.rpe10_m),
                "RPE50 [m]": _fmt(r.rpe50_m),
                "loops": DASH if r.n_loops_accepted is None else r.n_loops_accepted,
                "ΔATE %": delta,
                "reason": r.reason,
                "ref": r.key == ref_key,
                "key": r.key,
            }
        )
    return rows


def contribution_figure(rows: list[dict]) -> Any:
    """Horizontal bars of the ATE change against the reference, largest at the top."""
    import plotly.graph_objects as go

    usable = [r for r in rows if not r["ref"] and r["status"] == "ok" and r["ΔATE %"] != DASH]
    usable.sort(key=lambda r: r["ΔATE %"])
    colors = ["#c0392b" if r["ΔATE %"] > 0 else "#27ae60" for r in usable]
    fig = go.Figure(
        go.Bar(
            x=[r["ΔATE %"] for r in usable],
            y=[r["label"] for r in usable],
            orientation="h",
            marker_color=colors,
        )
    )
    fig.update_layout(
        xaxis_title="ATE change vs reference [%]", margin={"l": 10, "r": 10, "t": 30, "b": 40}
    )
    return fig


def _check_out_dir(out_dir: Path) -> None:
    resolved = Path(out_dir).resolve()
    inside_repo = resolved.is_relative_to(REPO)
    if inside_repo and not resolved.is_relative_to(REPO / "results"):
        raise ValueError(f"{out_dir}: reports go under results/, never into the source tree")


def export_report(runs: Sequence[Run], ref_key: str, out_dir: Path) -> Path:
    """``summary.md`` (table and specs), ``trajectories.png`` and ``errors.png`` in ``out_dir``."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _check_out_dir(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = metrics_table(runs, ref_key)
    lines = [
        "# Sensor impact explorer report",
        "",
        "| " + " | ".join(COLUMNS) + " |",
        "|" + "---|" * len(COLUMNS),
    ]
    for row in rows:
        mark = " (ref)" if row["ref"] else ""
        cells = [f"{row['label']}{mark}"] + [str(row[c]) for c in COLUMNS[1:]]
        lines.append("| " + " | ".join(cells) + " |")
    lines += ["", "![trajectories](trajectories.png)", "", "![errors](errors.png)", ""]
    for spec, r in runs:
        doc = {**spec_to_dict(spec), "key": r.key, "reason": r.reason}
        lines += [
            f"## {spec.label or r.key}",
            "",
            "```yaml",
            yaml.safe_dump(doc, sort_keys=False).rstrip(),
            "```",
            "",
        ]
    md = out_dir / "summary.md"
    md.write_text("\n".join(lines))

    fig, ax = plt.subplots(figsize=(7, 6))
    gts = [r.run_dir / "gt.tum" for _, r in runs if (r.run_dir / "gt.tum").is_file()]
    g = read_tum(gts[0]) if gts else None
    if g is not None:
        ax.plot(g.p[:, 0], g.p[:, 1], "k-", lw=2, label="ground truth")
    for run in runs:
        got = _aligned(run[1])
        if got is not None:
            p = got[1].est_aligned
            ax.plot(p[:, 0], p[:, 1], lw=1, label=_name(run))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(fontsize=8)
    fig.savefig(out_dir / "trajectories.png", dpi=120, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    for run in runs:
        path = run[1].run_dir / "errors_online.csv"
        if path.is_file():
            d = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
            ax.plot(d[:, 0], d[:, 1], lw=1, label=_name(run))
        for f in run[0].faults:
            ax.axvspan(f.t_start_s, f.t_end_s, color="red", alpha=0.1)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("position error [m]")
    ax.legend(fontsize=8)
    fig.savefig(out_dir / "errors.png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    return md


__all__ = [
    "contribution_figure",
    "error_figure",
    "export_report",
    "map_points",
    "metrics_table",
    "selected_time",
    "trajectory_figure",
]
