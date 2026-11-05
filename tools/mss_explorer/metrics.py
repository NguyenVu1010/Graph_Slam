"""Explorer metrics on top of mss_eval: error over time, translation RPE, ATE change in %."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mss_eval.ate import align
from mss_eval.ate import read_tum_stamps_ns as _stamps


def error_series(est: Path, gt: Path) -> tuple[np.ndarray, np.ndarray]:
    """``(t_s, err_m)``: position error after the SE(3) fit, ``t_s`` from the first gt stamp."""
    al = align(est, gt)
    t0 = int(_stamps(gt).min())
    err = np.linalg.norm(al.gt - al.est_aligned, axis=1)
    return (al.t_ns - t0) * 1e-9, err


def rpe_t(est_aligned: np.ndarray, gt: np.ndarray, seg_m: float) -> float | None:
    """Translation-only RPE [m]: for each i, the first j whose gt path length from i reaches
    ``seg_m``, error ``|(e_j - e_i) - (g_j - g_i)|``; RMSE over all such pairs, ``None`` if none."""
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt, axis=0), axis=1))])
    j = np.searchsorted(s, s + seg_m - 1e-9)
    ok = j < len(s)
    if not np.any(ok):
        return None
    i, j = np.nonzero(ok)[0], j[ok]
    d = (est_aligned[j] - est_aligned[i]) - (gt[j] - gt[i])
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))


def delta_ate_pct(ref_m: float, run_m: float) -> float:
    """ATE change of a run against the reference run, in percent of the reference."""
    return 100.0 * (run_m - ref_m) / ref_m
