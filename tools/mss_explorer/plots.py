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

from mss_eval.ate import align, read_tum_stamps_ns
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
)


def _name(run: Run) -> str:
    return run[0].label or run[1].key


def _ate(r: RunResult) -> float | None:
    return r.ate_opt_m if r.ate_opt_m is not None else r.ate_online_m


def _in_faults(t_s: np.ndarray, faults: Sequence[Fault]) -> np.ndarray:
    mask = np.zeros(len(t_s), dtype=bool)
    for f in faults:
        mask |= (t_s >= f.t_start_s) & (t_s < f.t_end_s)
    return mask


def _aligned(r: RunResult) -> tuple[np.ndarray, np.ndarray] | None:
    """(t_s from the first gt stamp, aligned xyz) of the online estimate, or None."""
    gt = r.run_dir / "gt.tum"
    try:
        al = align(r.run_dir / "bench" / "est.tum", gt)
    except (OSError, ValueError):
        return None
    return (al.t_ns - int(read_tum_stamps_ns(gt).min())) * 1e-9, al.est_aligned


def trajectory_figure(runs: Sequence[Run], gt: Path, t_cursor_s: float | None = None) -> Any:
    """Top view: ground truth, one trace per run (fault segments dashed), cursor markers."""
    import plotly.graph_objects as go

    fig = go.Figure()
    g = read_tum(gt)
    fig.add_trace(
        go.Scatter(
            x=g.p[:, 0],
            y=g.p[:, 1],
            name="ground truth",
            mode="lines",
            line={"color": "black", "width": 3},
        )
    )
    for run in runs:
        got = _aligned(run[1])
        if got is None:
            continue
        t, p = got
        fig.add_trace(go.Scatter(x=p[:, 0], y=p[:, 1], name=_name(run), mode="lines"))
        bad = _in_faults(t, run[0].faults)
        if bad.any():
            q = np.where(bad[:, None], p, np.nan)
            fig.add_trace(
                go.Scatter(
                    x=q[bad, 0],
                    y=q[bad, 1],
                    name=f"{_name(run)} (fault)",
                    mode="lines",
                    line={"dash": "dash", "width": 4},
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
                    marker={"size": 12, "symbol": "circle-open"},
                )
            )
    fig.update_layout(
        xaxis_title="x [m]",
        yaxis_title="y [m]",
        yaxis_scaleanchor="x",
        margin={"l": 40, "r": 10, "t": 30, "b": 40},
    )
    return fig


def error_figure(runs: Sequence[Run], which: str = "online") -> Any:
    """Position error over time per run; each distinct fault window shaded and labelled."""
    import plotly.graph_objects as go

    fig = go.Figure()
    for run in runs:
        path = run[1].run_dir / f"errors_{which}.csv"
        if not path.is_file():
            continue
        data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
        fig.add_trace(go.Scatter(x=data[:, 0], y=data[:, 1], name=_name(run), mode="lines"))
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
    ref_ate = None if ref is None or ref.status != "ok" else _ate(ref)
    rows = []
    for run in runs:
        r = run[1]
        a = _ate(r) if r.status == "ok" else None
        delta = DASH if a is None or not ref_ate else round(delta_ate_pct(ref_ate, a), 1)
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
    g = read_tum(runs[0][1].run_dir / "gt.tum")
    ax.plot(g.p[:, 0], g.p[:, 1], "k-", lw=2, label="ground truth")
    for run in runs:
        got = _aligned(run[1])
        if got is not None:
            ax.plot(got[1][:, 0], got[1][:, 1], lw=1, label=_name(run))
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
    "metrics_table",
    "trajectory_figure",
]
