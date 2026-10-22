"""Convert a bridged MCAP bag into the flat binary ``mssdata`` dataset read by the C++ bench.

Usage: python tools/bag_to_mssdata.py <bag_dir> <out_dir> [--origin-lla LAT LON ALT]
       [--pipeline-config cfg.yaml]
Prints the report as one JSON line; exit code 0 on success, 2 on error.

Layout (little-endian, packed): ``docs/mssdata_format.md``; decision: ADR-014.
Records are sorted by ``header.stamp`` ascending; records with an equal stamp keep the first one
(in bag log-time order). The bag is read-only, the dataset is derived and can be regenerated.
"""

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from mss_sim.bag_io import BagReader, cloud_to_arrays, stamp_ns
from mss_sim.geodesy import lla_to_enu

FORMAT_VERSION = 1

# Packed (align=False) little-endian records; sizes are asserted below and by the C++ reader.
IMU_DTYPE = np.dtype([("t", "<i8"), ("acc", "<f8", 3), ("gyro", "<f8", 3)], align=False)
GNSS_DTYPE = np.dtype(
    [("t", "<i8"), ("p_e_g", "<f8", 3), ("cov", "<f8", 9), ("status", "i1")], align=False
)
ODOM_DTYPE = np.dtype([("t", "<i8"), ("v", "<f8"), ("omega", "<f8")], align=False)
GT_DTYPE = np.dtype([("t", "<i8"), ("p", "<f8", 3), ("q_wxyz", "<f8", 4)], align=False)
LIDAR_HEADER_DTYPE = np.dtype([("t_start", "<i8"), ("n", "<u4")], align=False)
LIDAR_POINT_DTYPE = np.dtype(
    [("xyz", "<f4", 3), ("intensity", "<f4"), ("t_offset", "<u4"), ("ring", "<u2")], align=False
)
assert IMU_DTYPE.itemsize == 56
assert GNSS_DTYPE.itemsize == 105
assert ODOM_DTYPE.itemsize == 24
assert GT_DTYPE.itemsize == 64
assert LIDAR_HEADER_DTYPE.itemsize == 12
assert LIDAR_POINT_DTYPE.itemsize == 22

_T_IMU, _T_GNSS, _T_ODOM, _T_GT = "/imu/data", "/gnss/fix", "/wheel/odom", "/ground_truth"
_T_LIDAR, _T_TF = "/lidar/points", "/tf_static"
_LIDAR_FRAME = "lidar_link"
_COV_UNKNOWN = 0  # NavSatFix COVARIANCE_TYPE_UNKNOWN


@dataclass(frozen=True)
class ConvertReport:
    n: dict[str, int]
    origin_lla: tuple[float, float, float]
    gnss_cov_filled_from_sigma: bool


@dataclass(frozen=True)
class _PipelineParams:
    p_b_g_m: tuple[float, float, float]
    sigma_h_m: float | None
    sigma_v_m: float | None


def _load_pipeline_params(path: Path | None) -> _PipelineParams | None:
    if path is None:
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    params: dict[str, Any] = data.get("mss_node", {}).get("ros__parameters", {})
    p_b_g = params.get("frames", {}).get("p_b_g_m")
    if p_b_g is None:
        print(f"warning: {path} has no frames.p_b_g_m, using [0, 0, 0]", file=sys.stderr)
        p_b_g = [0.0, 0.0, 0.0]
    if len(p_b_g) != 3:
        raise ValueError(f"frames.p_b_g_m must have 3 entries, got {p_b_g}")
    gnss = params.get("gnss", {})
    sigma_h, sigma_v = gnss.get("sigma_h_m"), gnss.get("sigma_v_m")
    for name, value in (("sigma_h_m", sigma_h), ("sigma_v_m", sigma_v)):
        if value is not None and not value > 0.0:
            raise ValueError(f"gnss.{name} must be > 0, got {value}")
    return _PipelineParams(
        (float(p_b_g[0]), float(p_b_g[1]), float(p_b_g[2])),
        None if sigma_h is None else float(sigma_h),
        None if sigma_v is None else float(sigma_v),
    )


def _sorted_unique(records: np.ndarray) -> np.ndarray:
    """Sort by ``t`` (stable) and drop later records with an equal ``t``."""
    ordered = records[np.argsort(records["t"], kind="stable")]
    if len(ordered) < 2:
        return ordered
    keep = np.concatenate(([True], np.diff(ordered["t"]) != 0))
    return ordered[keep]


def _check_out_dir(out_dir: Path) -> None:
    if out_dir.exists() and (not out_dir.is_dir() or any(out_dir.iterdir())):
        raise ValueError(f"output directory {out_dir} is not empty")
    out_dir.mkdir(parents=True, exist_ok=True)


def _read_t_b_l(msg: Any) -> tuple[dict[str, list[float]], str] | None:
    for tf in msg.transforms:
        if tf.child_frame_id != _LIDAR_FRAME:
            continue
        tr, q = tf.transform.translation, tf.transform.rotation
        return (
            {
                "translation_m": [float(tr.x), float(tr.y), float(tr.z)],
                # ROS field order is x, y, z, w; the dataset stores Hamilton (w, x, y, z).
                "quaternion_wxyz": [float(q.w), float(q.x), float(q.y), float(q.z)],
            },
            str(tf.header.frame_id),
        )
    return None


def _write_scan(path: Path, t_start: int, msg: Any) -> None:
    arrays = cloud_to_arrays(msg)
    if arrays.t_ns is None or arrays.ring is None:
        raise ValueError("bag has no per-point time; run the bridge first")
    n = arrays.xyz.shape[0]
    header = np.zeros(1, dtype=LIDAR_HEADER_DTYPE)
    header["t_start"], header["n"] = t_start, n
    points = np.zeros(n, dtype=LIDAR_POINT_DTYPE)
    points["xyz"] = arrays.xyz
    points["intensity"] = arrays.intensity
    points["t_offset"] = arrays.t_ns
    points["ring"] = arrays.ring
    with path.open("wb") as f:
        header.tofile(f)
        points.tofile(f)


def convert(
    bag_dir: Path,
    out_dir: Path,
    *,
    origin_lla: tuple[float, float, float] | None = None,
    pipeline_config: Path | None = None,
) -> ConvertReport:
    """Convert the bridged bag ``bag_dir`` into the dataset directory ``out_dir`` (new or empty)."""
    params = _load_pipeline_params(pipeline_config)
    _check_out_dir(out_dir)
    lidar_dir = out_dir / "lidar"
    lidar_dir.mkdir()

    imu: list[tuple] = []
    gnss: list[tuple] = []  # (t, lat, lon, alt, cov_type, cov9, status)
    odom: list[tuple] = []
    gt: list[tuple] = []
    scans: list[tuple[int, int]] = []  # (t_start, arrival index)
    tf_info: tuple[dict[str, list[float]], str] | None = None

    with BagReader(bag_dir) as reader:
        wanted = [
            t for t in (_T_IMU, _T_GNSS, _T_ODOM, _T_GT, _T_LIDAR, _T_TF) if t in reader.topics()
        ]
        for topic, _, msg in reader.messages(wanted):
            if topic == _T_LIDAR:
                _write_scan(lidar_dir / f"tmp_{len(scans):06d}.bin", stamp_ns(msg.header), msg)
                scans.append((stamp_ns(msg.header), len(scans)))
            elif topic == _T_IMU:
                a, w = msg.linear_acceleration, msg.angular_velocity
                imu.append((stamp_ns(msg.header), (a.x, a.y, a.z), (w.x, w.y, w.z)))
            elif topic == _T_GNSS:
                gnss.append(
                    (
                        stamp_ns(msg.header),
                        float(msg.latitude),
                        float(msg.longitude),
                        float(msg.altitude),
                        int(msg.position_covariance_type),
                        np.asarray(msg.position_covariance, dtype=np.float64),
                        int(msg.status.status),
                    )
                )
            elif topic == _T_ODOM:
                tw = msg.twist.twist
                odom.append((stamp_ns(msg.header), float(tw.linear.x), float(tw.angular.z)))
            elif topic == _T_GT:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                # ROS field order is x, y, z, w; the dataset stores Hamilton (w, x, y, z).
                gt.append((stamp_ns(msg.header), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)))
            elif tf_info is None:
                tf_info = _read_t_b_l(msg)
    if tf_info is None:
        raise ValueError(f"bag has no {_T_TF} transform with child frame '{_LIDAR_FRAME}'")

    # Lidar: order by stamp, drop duplicates, rename to the final contiguous numbering.
    scans.sort(key=lambda s: s[0])  # stable
    kept: list[tuple[int, int]] = []
    for t_start, idx in scans:
        if kept and kept[-1][0] == t_start:
            (lidar_dir / f"tmp_{idx:06d}.bin").unlink()
        else:
            kept.append((t_start, idx))
    for number, (_, idx) in enumerate(kept):
        (lidar_dir / f"tmp_{idx:06d}.bin").rename(lidar_dir / f"{number:06d}.bin")

    imu_rec = np.zeros(len(imu), dtype=IMU_DTYPE)
    for i, (t, a, w) in enumerate(imu):
        imu_rec[i] = (t, a, w)
    odom_rec = np.array(odom, dtype=ODOM_DTYPE)
    gt_rec = np.zeros(len(gt), dtype=GT_DTYPE)
    for i, (t, p, q) in enumerate(gt):
        gt_rec[i] = (t, p, q)

    gnss_rec, used_origin, filled = _gnss_records(gnss, origin_lla, params)

    files = {"imu": imu_rec, "gnss": gnss_rec, "odom": odom_rec, "gt": gt_rec}
    counts: dict[str, int] = {}
    for name, rec in files.items():
        rec = _sorted_unique(rec)
        rec.tofile(out_dir / f"{name}.bin")
        counts[name] = len(rec)
    counts["lidar"] = len(kept)

    t_b_l, body_frame = tf_info
    index = {
        "version": FORMAT_VERSION,
        "topics": {
            **{n: {"file": f"{n}.bin", "count": counts[n]} for n in ("imu", "gnss", "odom", "gt")},
            "lidar": {"dir": "lidar", "count": counts["lidar"]},
        },
        "origin_lla": list(used_origin),
        "T_b_l": t_b_l,
        "p_b_g_m": list(params.p_b_g_m) if params else [0.0, 0.0, 0.0],
        "body_frame": body_frame,
        "lidar_frame": _LIDAR_FRAME,
        "gnss_cov_filled_from_sigma": filled,
    }
    (out_dir / "index.json").write_text(json.dumps(index, indent=2) + "\n", encoding="utf-8")
    return ConvertReport(counts, used_origin, filled)


def _gnss_records(
    gnss: list[tuple],
    origin_lla: tuple[float, float, float] | None,
    params: _PipelineParams | None,
) -> tuple[np.ndarray, tuple[float, float, float], bool]:
    """GNSS records in ENU about the origin (explicit or the earliest fix)."""
    rec = np.zeros(len(gnss), dtype=GNSS_DTYPE)
    if not gnss:
        return rec, origin_lla if origin_lla is not None else (0.0, 0.0, 0.0), False
    lla = np.array([g[1:4] for g in gnss], dtype=np.float64)
    if origin_lla is not None:
        origin = origin_lla
    else:
        first = min(range(len(gnss)), key=lambda i: gnss[i][0])
        origin = (float(lla[first, 0]), float(lla[first, 1]), float(lla[first, 2]))
    enu = lla_to_enu(lla, origin)
    filled = False
    for i, (t, _, _, _, cov_type, cov9, status) in enumerate(gnss):
        if cov_type == _COV_UNKNOWN:
            if params is None or params.sigma_h_m is None or params.sigma_v_m is None:
                raise ValueError(
                    "GNSS position_covariance_type is 0 (unknown); pass --pipeline-config "
                    "with gnss.sigma_h_m and gnss.sigma_v_m to fill the covariance"
                )
            cov9 = np.diag([params.sigma_h_m**2, params.sigma_h_m**2, params.sigma_v_m**2]).reshape(
                9
            )
            filled = True
        rec[i] = (t, enu[i], cov9, status)
    return rec, origin, filled


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--origin-lla", type=float, nargs=3, metavar=("LAT", "LON", "ALT"))
    parser.add_argument("--pipeline-config", type=Path)
    args = parser.parse_args(argv)
    origin = tuple(args.origin_lla) if args.origin_lla else None
    try:
        report = convert(
            args.bag_dir, args.out_dir, origin_lla=origin, pipeline_config=args.pipeline_config
        )
    except (ValueError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(json.dumps(asdict(report)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
