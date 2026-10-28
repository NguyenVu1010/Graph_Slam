"""Measure gpu_lidar's own range error per (range x incidence) cell for one world and trajectory.

    python tools/calibrate_lidar_render.py <raw_bag> <scenario.yaml> --out <table.yaml>
        [--gazebo "gz-sim 8.15.0"] [--render-engine ogre2]

Input: the RAW bag of a recording made with the Gazebo LiDAR noise at 0
(``tools/calibrate_lidar_render.sh``: ``record_scenario.sh --lidar-gz-sigma 0``, same scenario,
seed and robot). The raw bag has no bridge noise, and Gazebo renders each cloud at one instant,
so every point is mapped with the scan stamp. The residual to the world's sidecar planes is the
renderer's error alone.

Selection as in the noise report: silhouette points (``lidar.edge_jump_m``; a dropped neighbour
counts as an edge on both sides) and points below ``lidar.min_incidence_cos`` excluded; cells on
the noise-free plane range; outliers cut at 5 x the cell sigma, starting from 5 declared sigma
and iterated once (``noise_measure.cell_sigma``). Output YAML (written to ``--out``, never into
the source tree; ``sim/noise/render/<world>.yaml`` is the tracked copy): world, scenario, UTC
timestamp, Gazebo version, render engine, the LiDAR block of ``sim/models/robot.yaml`` and the
declared range sigma (re-run when they change), raw bag SHA256, the selection and
``cells: [[r_lo, r_hi, cos_lo, cos_hi, n_points, sigma_m], ...]`` for the cells with at least
``MIN_GATE_POINTS`` points. Exit 0 ok, 2 error.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import sys
from pathlib import Path

import numpy as np
import yaml

from mss_eval.noise_measure import (
    CELL_OUTLIER_SIGMAS,
    INCIDENCE_BINS_COS,
    MIN_GATE_POINTS,
    RANGE_BINS_M,
    Trajectory,
    build_render_table,
    range_sigma_selection,
)
from mss_eval.noise_report import lidar_pass, load_planes, read_bag
from mss_sim.profile import load_profile
from mss_sim.scenario import load_scenario

REPO = Path(__file__).resolve().parents[1]
ROBOT_YAML = REPO / "sim" / "models" / "robot.yaml"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def measure(raw_bag: Path, planes: Path, lidar_cfg: dict, min_range_m: float) -> list:
    """Render table cells of ``raw_bag`` against ``planes`` (``build_render_table``)."""
    bag = read_bag(raw_bag)
    if not bag.gt:
        raise ValueError(f"{raw_bag}: no /ground_truth")
    t, p, q = zip(*bag.gt, strict=True)
    traj = Trajectory(np.array(t, dtype=np.int64), np.array(p), np.array(q))
    samples = lidar_pass(
        raw_bag, bag, traj, load_planes(planes), min_range_m, float(lidar_cfg["edge_jump_m"])
    )
    keep = range_sigma_selection(samples.cos, samples.edge, lidar_cfg)
    start_m = CELL_OUTLIER_SIGMAS * float(lidar_cfg["range_sigma_m"])
    return build_render_table(samples.residual, samples.cos, samples.range_m, keep, start_m)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("raw_bag", type=Path, help="raw bag recorded with the Gazebo LiDAR noise at 0")
    ap.add_argument("scenario", type=Path, help="scenario YAML of that recording")
    ap.add_argument("--out", type=Path, required=True, help="table YAML to write")
    ap.add_argument("--gazebo", default="unknown", help="Gazebo version string")
    ap.add_argument("--render-engine", default="ogre2", help="gz render engine of the recording")
    args = ap.parse_args(argv)
    try:
        sc = load_scenario(args.scenario)
        profile = load_profile(sc.profile)
        lid = profile.lidar
        cfg = {
            "range_sigma_m": lid.range_sigma_m,
            "edge_jump_m": lid.edge_jump_m,
            "min_incidence_cos": lid.min_incidence_cos,
        }
        planes = REPO / "sim" / "worlds" / f"{sc.world}.planes.yaml"
        cells = measure(args.raw_bag, planes, cfg, lid.min_range_m)
    except (OSError, ValueError, KeyError) as exc:
        print(f"calibrate_lidar_render: {exc}", file=sys.stderr)
        return 2
    files = sorted(f for f in args.raw_bag.iterdir() if f.is_file())
    doc = {
        "world": sc.world,
        "scenario": sc.id,
        "timestamp_utc": datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "gazebo": args.gazebo,
        "render_engine": args.render_engine,
        "lidar": yaml.safe_load(ROBOT_YAML.read_text(encoding="utf-8"))["lidar"],
        "declared_range_sigma_m": lid.range_sigma_m,
        "what": "gpu_lidar range error alone (Gazebo noise 0, raw bag), per cell; measured, "
        "not tuned. Re-run when the world, trajectory, sensor or Gazebo version changes.",
        "raw_bag_sha256": {f.name: _sha256(f) for f in files},
        "selection": {
            "edge_jump_m": lid.edge_jump_m,
            "min_incidence_cos": lid.min_incidence_cos,
            "outlier_cut": f"{CELL_OUTLIER_SIGMAS:g} x the cell sigma, from "
            f"{CELL_OUTLIER_SIGMAS * lid.range_sigma_m:g} m, iterated once",
            "range": "noise-free plane range",
            "min_points": MIN_GATE_POINTS,
        },
        "range_bins_m": list(RANGE_BINS_M),
        "incidence_bins_cos": list(INCIDENCE_BINS_COS),
        "cells": cells,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    header = (
        f"# LiDAR render table for world '{sc.world}' (tools/calibrate_lidar_render.py).\n"
        "# cells: [range lo m, range hi m, |n.u| lo, |n.u| hi, points, sigma m].\n"
    )
    args.out.write_text(
        header + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, width=100),
        encoding="utf-8",
        newline="\n",
    )
    print(f"{args.out}: {len(cells)} cells")
    return 0


if __name__ == "__main__":
    sys.exit(main())
