"""Noise report (ADR-011 section 7): declared noise profile vs noise measured in a sim bag.

    python -m mss_eval.noise_report <bag_dir> <profile.yaml> --out <dir>
        [--planes planes.yaml] [--pipeline-config cfg.yaml] [--static-s 5]
        [--allan-gate] [--odom-gate]

Writes ``<out>/noise_report.html`` and ``<out>/noise_report.json``; exits 1 when a gated row
fails, so a bag with the wrong noise is never packaged. Reads MCAP with ``rosbags`` directly and
YAML with ``pyyaml``; never imports ``mss_sim`` or ROS (ADR-003/010).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

from mss_eval.noise_measure import (
    Report,
    Row,
    Section,
    Trajectory,
    estimate_gnss_sigma,
    estimate_range_sigma,
    gnss_section,
    imu_section,
    lidar_section,
    odom_section,
    ray_residuals,
    scan_to_world,
)
from mss_eval.pointcloud import cloud_to_arrays
from mss_eval.report_html import render

__all__ = ["Report", "Row", "Section", "estimate_gnss_sigma", "estimate_range_sigma", "run"]

_GT, _TF, _IMU = "/ground_truth", "/tf_static", "/imu/data"
_LIDAR, _GNSS, _ODOM = "/lidar/points", "/gnss/fix", "/wheel/odom"
_BODY_FRAME, _LIDAR_FRAME = "base_link", "lidar_link"
_NS_PER_S = 1_000_000_000
_STATIC_SPEED_MPS = 0.05
_MAX_GAP_S = 0.5
_GT_MISSING = "ground_truth missing"
_TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


@dataclass
class _Bag:
    topics: dict[str, str]
    stamps: dict[str, list[int]] = field(default_factory=dict)  # header stamp, else log time
    imu: list[tuple[int, tuple[float, ...], tuple[float, ...]]] = field(default_factory=list)
    gt: list[tuple[int, tuple[float, ...], tuple[float, ...]]] = field(default_factory=list)
    gnss: list[tuple[int, tuple[float, ...], tuple[float, ...]]] = field(default_factory=list)
    odom: list[tuple[int, float, float]] = field(default_factory=list)
    T_b_l: np.ndarray | None = None  # noqa: N815


def _stamp(msg: Any, log_ns: int) -> int:
    h = getattr(msg, "header", None)
    return log_ns if h is None else int(h.stamp.sec) * _NS_PER_S + int(h.stamp.nanosec)


def _t_b_l(tf_msg: Any) -> np.ndarray | None:
    for ts in tf_msg.transforms:
        if ts.header.frame_id == _BODY_FRAME and ts.child_frame_id == _LIDAR_FRAME:
            tr, q = ts.transform.translation, ts.transform.rotation
            t = np.eye(4)
            t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # scipy: x y z w
            t[:3, 3] = (tr.x, tr.y, tr.z)
            return t
    return None


def _read_bag(path: Path) -> _Bag:
    """Everything except LiDAR clouds (those are streamed by ``_lidar_pass``)."""
    with Reader(path) as reader:
        bag = _Bag({c.topic: c.msgtype for c in reader.connections})
        conns = [c for c in reader.connections if c.topic != _LIDAR]
        for conn, log_ns, raw in reader.messages(connections=conns) if conns else ():
            msg = _TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            t = _stamp(msg, int(log_ns))
            bag.stamps.setdefault(conn.topic, []).append(t)
            if conn.topic == _IMU:
                w, a = msg.angular_velocity, msg.linear_acceleration
                bag.imu.append((t, (w.x, w.y, w.z), (a.x, a.y, a.z)))
            elif conn.topic == _GT:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                bag.gt.append((t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))  # x, y, z, w
            elif conn.topic == _GNSS:
                cov = np.asarray(msg.position_covariance, dtype=float).reshape(3, 3)
                bag.gnss.append(
                    (t, (msg.latitude, msg.longitude, msg.altitude), tuple(cov.diagonal()))
                )
            elif conn.topic == _ODOM:
                tw = msg.twist.twist
                bag.odom.append((t, tw.linear.x, tw.angular.z))
            elif conn.topic == _TF and bag.T_b_l is None:
                bag.T_b_l = _t_b_l(msg)
    return bag


def _lidar_pass(
    path: Path,
    bag: _Bag,
    traj: Trajectory | None,
    planes: tuple[np.ndarray, np.ndarray] | None,
    min_range_m: float,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    """Record LiDAR stamps; with planes and GT, return (residuals, cos, n_nan, n_not_inf)."""
    dists: list[np.ndarray] = []
    coss: list[np.ndarray] = []
    n_nan = n_not_inf = 0
    t_b_l = bag.T_b_l if bag.T_b_l is not None else np.eye(4)
    with Reader(path) as reader:
        conns = [c for c in reader.connections if c.topic == _LIDAR]
        for conn, log_ns, raw in reader.messages(connections=conns) if conns else ():
            msg = _TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            stamp = _stamp(msg, int(log_ns))
            bag.stamps.setdefault(_LIDAR, []).append(stamp)
            if planes is None or traj is None:
                continue
            xyz, t_off, _, _, _ = cloud_to_arrays(msg)
            n_nan += int(np.isnan(xyz).any(axis=1).sum())
            n_not_inf += int((~np.isinf(xyz).any(axis=1)).sum())
            ok = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) >= min_range_m)
            t_i = stamp + (t_off[ok].astype(np.int64) if t_off is not None else 0)
            p_w, o_w = scan_to_world(xyz[ok], np.broadcast_to(t_i, ok.sum()), traj, t_b_l)
            res, cos = ray_residuals(p_w, o_w, *planes)
            dists.append(res.astype(np.float32))
            coss.append(cos.astype(np.float32))
    if not dists:
        return np.empty(0, np.float32), np.empty(0, np.float32), n_nan, n_not_inf
    return np.concatenate(dists), np.concatenate(coss), n_nan, n_not_inf


def _bag_section(bag: _Bag, bridge_report: dict | None) -> Section:
    rows: list[Row] = []
    has_gt = _GT in bag.topics and bool(bag.gt)
    rows.append(
        Row(
            f"{_GT} present",
            "yes",
            "yes" if has_gt else "no",
            "required",
            has_gt,
            "" if has_gt else _GT_MISSING,
        )
    )
    dropped_gnss = (bridge_report or {}).get("n_dropped_gnss", 0)
    for topic in sorted(bag.topics):
        st = np.asarray(bag.stamps.get(topic, []), dtype=np.int64)
        inc = bool(np.all(np.diff(st) > 0))
        rows.append(
            Row(f"{topic} stamps increasing", None, "yes" if inc else "no", "strictly", inc)
        )
        if len(st) < 2:
            continue
        gap = float(np.max(np.diff(st)) / _NS_PER_S)
        # Declared GNSS outages are missing messages: only gate GNSS gaps when none were dropped.
        gated = topic != _GNSS or dropped_gnss == 0
        rows.append(
            Row(
                f"{topic} max gap s",
                None,
                gap,
                f"< {_MAX_GAP_S:g} s",
                gap < _MAX_GAP_S if gated else None,
                "" if gated else f"{dropped_gnss} fixes dropped by declared outages",
            )
        )
    for key, value in (bridge_report or {}).items():
        if key == "n_msgs":
            for topic, n in value.items():
                got = len(bag.stamps.get(topic, []))
                rows.append(Row(f"n_msgs {topic}", n, got, "bridge_report vs bag", None))
        else:
            rows.append(Row(key, None, value, "bridge_report", None))
    return Section(
        "Bag",
        rows,
        [],
        "SIM-02: stamps strictly increasing, ground truth present, max gap per topic.",
    )


def _missing_gt(name: str) -> Section:
    return Section(name, [Row("skipped", None, None, "needs /ground_truth", None, _GT_MISSING)])


def _pipeline_params(path: Path | None) -> dict:
    if path is None:
        return {}
    cfg = yaml.safe_load(path.read_text()) or {}
    return cfg.get("mss_node", {}).get("ros__parameters", cfg)


def _load_planes(path: Path) -> tuple[np.ndarray, np.ndarray]:
    items = yaml.safe_load(path.read_text())
    normals = np.array([p["n"] for p in items], dtype=float)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    return normals, np.array([p["d"] for p in items], dtype=float)


def _resolve(bag_dir: Path) -> tuple[Path, Path]:
    """(bag path, directory holding ``bridge_report.json``) for a bridge output or bare bag."""
    if (bag_dir / "bag").is_dir():
        return bag_dir / "bag", bag_dir
    return bag_dir, bag_dir.parent


def _is_static(traj: Trajectory, t0_ns: int, static_s: float) -> bool:
    t = traj.t_ns[(traj.t_ns >= t0_ns) & (traj.t_ns <= t0_ns + int(static_s * _NS_PER_S))]
    if len(t) < 2:
        return False
    v = np.linalg.norm(np.diff(traj.position(t), axis=0), axis=1) / (np.diff(t) / _NS_PER_S)
    return bool(np.max(v) < _STATIC_SPEED_MPS)


def run(
    bag_dir: Path,
    profile: Path,
    out_dir: Path,
    *,
    planes: Path | None = None,
    pipeline_config: Path | None = None,
    static_s: float = 5.0,
    allan_gate: bool = False,
    odom_gate: bool = False,
) -> Report:
    """Measure, write ``noise_report.html`` and ``noise_report.json`` into ``out_dir``."""
    bag_path, side_dir = _resolve(Path(bag_dir))
    prof = yaml.safe_load(Path(profile).read_text())
    pipe = _pipeline_params(pipeline_config)
    br_path = side_dir / "bridge_report.json"
    bridge_report = json.loads(br_path.read_text()) if br_path.exists() else None

    bag = _read_bag(bag_path)
    traj = None
    if bag.gt:
        t, p, q = zip(*bag.gt, strict=True)
        traj = Trajectory(np.array(t, dtype=np.int64), np.array(p), np.array(q))
    plane_arrays = _load_planes(planes) if planes is not None else None
    min_range = float(prof["lidar"].get("min_range_m", 0.0))
    dists, coss, n_nan, n_not_inf = _lidar_pass(bag_path, bag, traj, plane_arrays, min_range)

    sections: list[Section] = []
    if bag.imu:
        t, w, a = zip(*bag.imu, strict=True)
        t_imu = np.array(t, dtype=np.int64)
        static = traj is not None and _is_static(traj, int(t_imu[0]), static_s)
        sections.append(
            imu_section(
                t_imu,
                np.array(w),
                np.array(a),
                prof["imu"],
                static=static,
                static_s=static_s,
                allan_gate=allan_gate,
                pipeline_imu=pipe.get("imu"),
            )
        )
    if traj is None:
        sections += [_missing_gt(n) for n in ("GNSS", "Wheel odom")]
        if planes is not None:
            sections.append(_missing_gt("LiDAR"))
    else:
        if bag.gnss:
            t, lla, cov = zip(*bag.gnss, strict=True)
            p_b_g = np.asarray(pipe.get("frames", {}).get("p_b_g_m", [0.0, 0.0, 0.0]), float)
            sections.append(
                gnss_section(
                    np.array(t, dtype=np.int64),
                    np.array(lla),
                    np.array(cov),
                    traj,
                    prof["gnss"],
                    p_b_g,
                    pipe.get("gnss"),
                )
            )
        if plane_arrays is not None:
            sections.append(
                lidar_section(dists, coss, n_nan, n_not_inf, prof["lidar"], pipe.get("lidar"))
            )
        if len(bag.odom) > 1:
            t, v, w = zip(*bag.odom, strict=True)
            sections.append(
                odom_section(
                    np.array(t, dtype=np.int64), np.array(v), np.array(w), traj, odom_gate=odom_gate
                )
            )
    sections.append(_bag_section(bag, bridge_report))

    passed = all(r.ok is not False for s in sections for r in s.rows)
    report = Report(sections, passed)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    title = f"Noise report: {bag_path.parent.name if bag_path.name == 'bag' else bag_path.name}"
    (out_dir / "noise_report.html").write_text(render(report, title), encoding="utf-8")
    (out_dir / "noise_report.json").write_text(json.dumps(_to_json(report), indent=2))
    return report


def _clean(v: Any) -> Any:
    if isinstance(v, float | np.floating):
        return float(v) if math.isfinite(v) else None
    if isinstance(v, np.bool_ | np.integer):
        return v.item()
    return v


def _to_json(report: Report) -> dict:
    sections = [
        {
            "name": s.name,
            "note": s.note,
            "rows": [{k: _clean(v) for k, v in asdict(r).items()} for r in s.rows],
        }
        for s in report.sections
    ]
    return {"passed": report.passed, "sections": sections}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag_dir", type=Path, help="bridge output dir (with bag/) or the bag itself")
    ap.add_argument("profile", type=Path, help="declared noise profile, sim/noise/*.yaml")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--planes", type=Path, help="YAML list of {n: [x, y, z], d: m}, n . p = d")
    ap.add_argument("--pipeline-config", type=Path, help="pipeline YAML (lever arm, beliefs)")
    ap.add_argument("--static-s", type=float, default=5.0, help="initial static window, s")
    ap.add_argument("--allan-gate", action="store_true", help="gate on Allan deviation")
    ap.add_argument("--odom-gate", action="store_true", help="gate odom drift to 1-3 %%")
    args = ap.parse_args(argv)
    report = run(
        args.bag_dir,
        args.profile,
        args.out,
        planes=args.planes,
        pipeline_config=args.pipeline_config,
        static_s=args.static_s,
        allan_gate=args.allan_gate,
        odom_gate=args.odom_gate,
    )
    failed = [f"{s.name}: {r.name}" for s in report.sections for r in s.rows if r.ok is False]
    print("PASS" if report.passed else "FAIL\n  " + "\n  ".join(failed))
    print(args.out / "noise_report.html")
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
