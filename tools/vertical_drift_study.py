"""Vertical-drift study helpers (docs/derivations/vertical_drift_study.md, backlog S2).

Usage:
  python tools/vertical_drift_study.py bias <mssdata>
  python tools/vertical_drift_study.py debias <mssdata> <out_dir> acc|accxy|accz|gyro
  python tools/vertical_drift_study.py compare <est.tum> <mssdata>
  python tools/vertical_drift_study.py floor <mssdata> --pipeline-config <overlay.yaml>

Each prints one JSON line; exit 2 on bad input. Reads an ``mssdata`` dataset
(``tools/bag_to_mssdata.py``) and the bench's ``est.tum``; never links core.

- ``bias``: the true IMU biases, oracle from the ground truth over the still start (the robot
  is at rest until the first ground-truth motion; 0.2 s margin): ``ba = mean(f) - R_w_b^T
  [0, 0, g]``, ``bg = mean(gyro)``.
- ``debias``: a copy of the dataset whose ``imu.bin`` has the true bias removed (``acc``: all
  three axes, ``accxy``, ``accz``, ``gyro``); the other files are symlinked (copied where
  symlinks are not allowed). ``out_dir`` must not exist.
- ``compare``: estimate vs ground truth anchored at the first estimate (yaw and translation
  only, so a tilt shows as a climb): vertical error ``ez`` over time, and the tilt error
  ``c = up_true x up_est`` in b (``up = R_w_b^T e_z``; a pitch error is ``c_y``, roll ``c_x``).
  Also the SE(3)-aligned ATE split into horizontal and vertical RMS.
- ``floor``: oracle for the LiDAR and the extrinsic: every 10th scan's points, through the true
  body pose (interpolated per point) and the overlay's ``T_b_l``, within ``--z-max-m`` of the
  floor plane z = 0; the floor height and a plane fit ``z = z0 + s_x x_b + s_y y_b``.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from bag_to_mssdata import GT_DTYPE, IMU_DTYPE, LIDAR_HEADER_DTYPE, LIDAR_POINT_DTYPE
from mss_eval.ate import read_tum_stamps_ns
from mss_eval.tum import read_tum

GRAVITY_MPS2 = 9.80665
STILL_SPEED_MPS = 1e-3  # m/s, ground-truth speed above which the robot moves
STILL_MARGIN_NS = 200_000_000  # ns cut before the first motion
MIN_RANGE_M = 0.5  # m, LiDAR points closer are skipped (profile min range)
SCAN_STRIDE = 10  # every 10th scan in ``floor``
WINDOWS_S = ((0.0, 10.0), (10.0, 30.0), (30.0, 50.0), (50.0, float("inf")))


class StudyError(Exception):
    pass


def load_gt(data: Path) -> tuple[np.ndarray, np.ndarray, Rotation]:
    g = np.fromfile(data / "gt.bin", dtype=GT_DTYPE)
    if len(g) < 2:
        raise StudyError(f"{data}: gt.bin has fewer than 2 records")
    q = g["q_wxyz"]  # Hamilton (w, x, y, z); scipy wants (x, y, z, w)
    return g["t"], g["p"], Rotation.from_quat(np.c_[q[:, 1:], q[:, :1]])


def still_window_ns(t: np.ndarray, p: np.ndarray) -> tuple[int, int]:
    v = np.linalg.norm(np.diff(p, axis=0), axis=1) / (np.diff(t) * 1e-9)
    moving = np.nonzero(v > STILL_SPEED_MPS)[0]
    t_move = int(t[moving[0]]) if len(moving) else int(t[-1])
    return int(t[0]), t_move - STILL_MARGIN_NS


def true_bias(data: Path) -> dict[str, Any]:
    imu = np.fromfile(data / "imu.bin", dtype=IMU_DTYPE)
    t, p, R = load_gt(data)
    t0, t1 = still_window_ns(t, p)
    m = (imu["t"] >= t0) & (imu["t"] < t1)
    if m.sum() < 2:
        raise StudyError(f"{data}: no IMU samples in the still start")
    idx = np.clip(np.searchsorted(t, imu["t"][m]), 0, len(t) - 1)
    f_rest = R[idx].inv().apply([0.0, 0.0, GRAVITY_MPS2])  # specific force at rest, in b
    ba = (imu["acc"][m] - f_rest).mean(axis=0)
    bg = imu["gyro"][m].mean(axis=0)
    return {
        "still_s": (t1 - t0) * 1e-9,
        "n_imu": int(m.sum()),
        "ba_mps2": ba.tolist(),
        "bg_radps": bg.tolist(),
        # Tilt the shipped initialiser takes from b (c_x, c_y of compare): (-b_y, b_x) / g.
        "tilt_from_bias_mrad": [-1e3 * ba[1] / GRAVITY_MPS2, 1e3 * ba[0] / GRAVITY_MPS2],
    }


def debias(data: Path, out: Path, mode: str) -> dict[str, Any]:
    b = true_bias(data)
    ba, bg = np.array(b["ba_mps2"]), np.array(b["bg_radps"])
    sub_a = {"acc": ba, "accxy": ba * [1, 1, 0], "accz": ba * [0, 0, 1], "gyro": 0 * ba}[mode]
    sub_g = bg if mode == "gyro" else 0 * bg
    out.mkdir(parents=True, exist_ok=False)
    for f in data.iterdir():
        if f.name == "imu.bin":
            continue
        try:
            os.symlink(f.resolve(), out / f.name)
        except OSError:  # no symlink privilege (Windows): copy
            (shutil.copytree if f.is_dir() else shutil.copy2)(f, out / f.name)
    imu = np.fromfile(data / "imu.bin", dtype=IMU_DTYPE)
    imu["acc"] -= sub_a
    imu["gyro"] -= sub_g
    imu.tofile(out / "imu.bin")
    return {"mode": mode, "out": str(out), **b}


def _interp(
    t: np.ndarray, p: np.ndarray, R: Rotation, tq: np.ndarray
) -> tuple[np.ndarray, Rotation]:
    i = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
    a = (tq - t[i - 1]) / np.maximum(t[i] - t[i - 1], 1)
    return p[i - 1] + (p[i] - p[i - 1]) * a[:, None], R[np.where(a < 0.5, i - 1, i)]


def compare(est_tum: Path, data: Path) -> dict[str, Any]:
    est = read_tum(est_tum)
    te = read_tum_stamps_ns(est_tum)
    t, p, R = load_gt(data)
    m = (te >= t[0]) & (te <= t[-1])
    if m.sum() < 10:
        raise StudyError(f"{est_tum}: fewer than 10 estimates inside the ground truth")
    te, pe, Re = te[m], est.p[m], Rotation.from_quat(est.q_xyzw[m])
    pg, Rg = _interp(t, p, R, te)
    Rz = Rotation.from_euler("z", Rg[0].as_euler("ZYX")[0] - Re[0].as_euler("ZYX")[0])
    err = Rz.apply(pe - pe[0]) + pg[0] - pg
    ez = err[:, 2]
    c = np.cross(Rg.inv().apply([0.0, 0.0, 1.0]), Re.inv().apply([0.0, 0.0, 1.0]))
    tt = (te - te[0]) * 1e-9
    # SE(3) Umeyama alignment for the usual ATE.
    mu_e, mu_g = pe.mean(0), pg.mean(0)
    U, _, Vt = np.linalg.svd((pe - mu_e).T @ (pg - mu_g))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    d = ((Vt.T @ D @ U.T) @ (pe - mu_e).T).T + mu_g - pg
    windows = {}
    for lo, hi in WINDOWS_S:
        k = (tt >= lo) & (tt < hi)
        if k.any():
            windows[f"{lo:g}-{hi:g}"] = {
                "mean_ez_m": float(ez[k].mean()),
                "tilt_x_mrad": float(1e3 * c[k, 0].mean()),
                "tilt_y_mrad": float(1e3 * c[k, 1].mean()),
            }
    return {
        "n": int(len(te)),
        "end_ez_m": float(ez[-1]),
        "max_abs_ez_m": float(np.abs(ez).max()),
        "rms_ez_m": float(np.sqrt(np.mean(ez**2))),
        "end_eh_m": float(np.linalg.norm(err[-1, :2])),
        "tilt_x_mrad_mean": float(1e3 * c[:, 0].mean()),
        "tilt_y_mrad_mean": float(1e3 * c[:, 1].mean()),
        "ate_se3_m": float(np.sqrt(np.mean(np.sum(d**2, 1)))),
        "ate_se3_h_m": float(np.sqrt(np.mean(np.sum(d[:, :2] ** 2, 1)))),
        "ate_se3_v_m": float(np.sqrt(np.mean(d[:, 2] ** 2))),
        "windows": windows,
    }


def load_t_b_l(overlay: Path) -> tuple[np.ndarray, Rotation]:
    params = yaml.safe_load(overlay.read_text())["mss_node"]["ros__parameters"]
    tbl = params["frames"]["T_b_l"]
    w, x, y, z = tbl["quaternion_wxyz"]  # Hamilton (w, x, y, z)
    return np.asarray(tbl["translation_m"], dtype=float), Rotation.from_quat([x, y, z, w])


def floor(data: Path, overlay: Path, z_max_m: float) -> dict[str, Any]:
    t, p, R = load_gt(data)
    p_b_l, R_b_l = load_t_b_l(overlay)
    zs, rings, xys = [], [], []
    for f in sorted((data / "lidar").glob("*.bin"))[::SCAN_STRIDE]:
        raw = f.read_bytes()
        h = np.frombuffer(raw[: LIDAR_HEADER_DTYPE.itemsize], dtype=LIDAR_HEADER_DTYPE)[0]
        pts = np.frombuffer(raw[LIDAR_HEADER_DTYPE.itemsize :], dtype=LIDAR_POINT_DTYPE)
        xyz = pts["xyz"].astype(float)
        ok = np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > MIN_RANGE_M)
        tq = h["t_start"] + pts["t_offset"][ok].astype(np.int64)
        pw_b, R_w_b = _interp(t, p, R, tq)
        p_w = R_w_b.apply(R_b_l.apply(xyz[ok]) + p_b_l) + pw_b
        fl = np.abs(p_w[:, 2]) < z_max_m
        zs.append(p_w[fl, 2])
        rings.append(pts["ring"][ok][fl])
        xys.append(R_w_b[fl].inv().apply(p_w[fl] - pw_b[fl])[:, :2])
    z, ring, xy = np.concatenate(zs), np.concatenate(rings), np.concatenate(xys)
    if len(z) < 3:
        raise StudyError(f"{data}: no floor points")
    coef, *_ = np.linalg.lstsq(np.c_[np.ones(len(z)), xy], z, rcond=None)
    return {
        "n_floor": int(len(z)),
        "floor_median_z_m": float(np.median(z)),
        "fit_z0_m": float(coef[0]),
        "fit_slope_x_mrad": float(1e3 * coef[1]),
        "fit_slope_y_mrad": float(1e3 * coef[2]),
        "per_ring_median_z_m": {
            int(k): float(np.median(z[ring == k]))
            for k in np.unique(ring)
            if (ring == k).sum() > 50
        },
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("bias").add_argument("data", type=Path)
    a = sub.add_parser("debias")
    a.add_argument("data", type=Path)
    a.add_argument("out", type=Path)
    a.add_argument("mode", choices=["acc", "accxy", "accz", "gyro"])
    a = sub.add_parser("compare")
    a.add_argument("est", type=Path)
    a.add_argument("data", type=Path)
    a = sub.add_parser("floor")
    a.add_argument("data", type=Path)
    a.add_argument("--pipeline-config", type=Path, required=True)
    a.add_argument("--z-max-m", type=float, default=0.08)  # 4 range sigma of the profile
    args = ap.parse_args(argv)
    try:
        if args.cmd == "bias":
            out = true_bias(args.data)
        elif args.cmd == "debias":
            out = debias(args.data, args.out, args.mode)
        elif args.cmd == "compare":
            out = compare(args.est, args.data)
        else:
            out = floor(args.data, args.pipeline_config, args.z_max_m)
    except (OSError, ValueError, KeyError, StudyError) as exc:
        print(f"vertical_drift_study: error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
