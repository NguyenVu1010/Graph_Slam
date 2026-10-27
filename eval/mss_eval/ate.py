"""Absolute trajectory error (ATE RMSE) between two TUM files, after SE(3) alignment.

Usage: python -m mss_eval.ate <est.tum> <gt.tum> [--plot out.png] [--max-dt-ns N] [--min-pairs N]
Prints ``{"ate_rmse_m": ..., "n_pairs": ..., "n_unmatched": ...}`` as one JSON line; exit 2 on
error. Reads TUM files only (ADR-003: eval never links core); evo is not needed.

Stamps are parsed from the text into int64 nanoseconds (no float round trip, epoch-scale
stamps keep their nanoseconds). Each estimate is paired with the nearest ground-truth stamp
within ``max_dt_ns``; estimates without one are counted as unmatched and ignored. Alignment is
rigid (rotation + translation, no scale), the Umeyama solution on the paired positions.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mss_eval.tum import read_tum

_NS_PER_S = 1_000_000_000
_FRAC_DIGITS = 9
DEFAULT_MAX_DT_NS = 2_500_000  # half the 200 Hz ground-truth period
DEFAULT_MIN_PAIRS = 10


@dataclass(frozen=True)
class AteResult:
    rmse_m: float
    n_pairs: int
    n_unmatched: int


def _parse_stamp_ns(text: str, path: Path, line_no: int) -> int:
    neg = text.startswith("-")
    body = text[1:] if neg else text
    sec, _, frac = body.partition(".")
    if not sec.isdigit() or (frac and not frac.isdigit()) or len(frac) > _FRAC_DIGITS:
        raise ValueError(f"{path}: line {line_no}: bad timestamp {text!r}")
    ns = int(sec) * _NS_PER_S + int(frac.ljust(_FRAC_DIGITS, "0") or "0")
    return -ns if neg else ns


def read_tum_stamps_ns(path: Path) -> np.ndarray:
    """Timestamps of a TUM file as int64 ns, parsed from the text (``sec.frac``, <= 9 digits)."""
    stamps = []
    for line_no, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        stamps.append(_parse_stamp_ns(stripped.split()[0], path, line_no))
    return np.array(stamps, dtype=np.int64)


def associate(
    est_ns: np.ndarray, gt_ns: np.ndarray, max_dt_ns: int
) -> tuple[np.ndarray, np.ndarray]:
    """Index pairs ``(i_est, i_gt)``: nearest GT stamp per estimate with ``|dt| <= max_dt_ns``.

    ``gt_ns`` must be sorted ascending. Estimates without a GT stamp in range are left out.
    """
    est = np.asarray(est_ns, dtype=np.int64)
    gt = np.asarray(gt_ns, dtype=np.int64)
    if len(gt) == 0 or len(est) == 0:
        empty = np.zeros(0, dtype=np.int64)
        return empty, empty
    right = np.clip(np.searchsorted(gt, est, side="left"), 0, len(gt) - 1)
    left = np.clip(right - 1, 0, len(gt) - 1)
    d_right = np.abs(gt[right] - est)  # integer differences: exact
    d_left = np.abs(gt[left] - est)
    nearest = np.where(d_left <= d_right, left, right)
    dt = np.minimum(d_left, d_right)
    ok = dt <= max_dt_ns
    return np.flatnonzero(ok).astype(np.int64), nearest[ok].astype(np.int64)


def umeyama_se3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``R`` (3x3, det +1) and ``t`` (3,) minimising ``sum |dst - (R src + t)|^2`` (no scale)."""
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    mu_src, mu_dst = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_dst).T @ (src - mu_src) / len(src)  # cross-covariance, dst x src
    u, _, vt = np.linalg.svd(cov)
    fix = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        fix[2, 2] = -1.0  # reflection: flip the axis of the smallest singular value
    rot = u @ fix @ vt
    return rot, mu_dst - rot @ mu_src


def _require_finite(path: Path, poses: np.ndarray) -> None:
    """ValueError naming the file line of the first pose with a NaN or infinite value."""
    bad = np.flatnonzero(~np.isfinite(poses).all(axis=1))
    if len(bad) == 0:
        return
    data_lines = [
        n
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if line.strip() and not line.lstrip().startswith("#")
    ]
    raise ValueError(f"{path}: line {data_lines[bad[0]]}: non-finite pose ({len(bad)} rows)")


def _aligned_pairs(
    est_path: Path, gt_path: Path, max_dt_ns: int, min_pairs: int
) -> tuple[np.ndarray, np.ndarray, int]:
    """Aligned estimate positions, matching GT positions and the number of unmatched estimates."""
    est, gt = read_tum(Path(est_path)), read_tum(Path(gt_path))
    _require_finite(Path(est_path), np.column_stack([est.p, est.q_xyzw]))
    _require_finite(Path(gt_path), np.column_stack([gt.p, gt.q_xyzw]))
    gt_ns = read_tum_stamps_ns(gt_path)
    order = np.argsort(gt_ns, kind="stable")
    i_est, i_gt = associate(read_tum_stamps_ns(est_path), gt_ns[order], max_dt_ns)
    n_pairs = len(i_est)
    if n_pairs < min_pairs:
        raise ValueError(
            f"{est_path}: only {n_pairs} pairs with {gt_path} within {max_dt_ns} ns "
            f"(need >= {min_pairs})"
        )
    src, dst = est.p[i_est], gt.p[order][i_gt]
    rot, t = umeyama_se3(src, dst)
    return src @ rot.T + t, dst, len(est) - n_pairs


def ate(
    est_path: Path,
    gt_path: Path,
    *,
    max_dt_ns: int = DEFAULT_MAX_DT_NS,
    min_pairs: int = DEFAULT_MIN_PAIRS,
) -> AteResult:
    """ATE RMSE [m] of ``est_path`` against ``gt_path``; ``ValueError`` below ``min_pairs``."""
    aligned, dst, n_unmatched = _aligned_pairs(est_path, gt_path, max_dt_ns, min_pairs)
    err = np.linalg.norm(dst - aligned, axis=1)
    rmse = float(np.sqrt(np.mean(err * err)))
    return AteResult(rmse_m=rmse, n_pairs=len(dst), n_unmatched=n_unmatched)


def plot_xy(est_path: Path, gt_path: Path, out_png: Path) -> None:
    """Top view (x, y) of the aligned estimate over the full ground truth, as a PNG."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    aligned, _, _ = _aligned_pairs(est_path, gt_path, DEFAULT_MAX_DT_NS, DEFAULT_MIN_PAIRS)
    gt = read_tum(Path(gt_path))
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(gt.p[:, 0], gt.p[:, 1], label="ground truth", color="0.4", linewidth=2)
    ax.plot(aligned[:, 0], aligned[:, 1], label="estimate (SE(3)-aligned)", linewidth=1)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    ax.set_title(Path(est_path).parent.name or "trajectory")
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=100)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("est", type=Path)
    parser.add_argument("gt", type=Path)
    parser.add_argument("--plot", type=Path, help="write the aligned x-y plot to this PNG")
    parser.add_argument("--max-dt-ns", type=int, default=DEFAULT_MAX_DT_NS)
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS)
    args = parser.parse_args(argv)
    try:
        res = ate(args.est, args.gt, max_dt_ns=args.max_dt_ns, min_pairs=args.min_pairs)
        if args.plot is not None:
            plot_xy(args.est, args.gt, args.plot)
    except (ValueError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {"ate_rmse_m": res.rmse_m, "n_pairs": res.n_pairs, "n_unmatched": res.n_unmatched}
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
