"""LiDAR effects: per-point time, motion skew, dropout and bad points (SIM-04).

Organized clouds are ``height`` rings x ``width`` columns, flattened row-major, ring 0 lowest.
All functions are pure except for draws from the caller's ``rng``.
"""

from dataclasses import replace

import numpy as np

from mss_sim.bag_io import CloudArrays
from mss_sim.gt import GroundTruth
from mss_sim.profile import LidarNoise

_NS_PER_S = 1_000_000_000
_N_RINGS = 16
_ELEV_MIN_DEG, _ELEV_MAX_DEG = -15.0, 15.0
_NEAR_RANGE_M = 0.3  # simulated returns from the robot body, below min_range_m
_MIN_NORMAL_NORM = 1e-9


def _finite_rows(xyz: np.ndarray) -> np.ndarray:
    return np.isfinite(xyz).all(axis=1)


def point_times_ns(xyz: np.ndarray, stamp_ns: int, scan_period_s: float) -> np.ndarray:
    """Per-point time offset from ``stamp_ns`` (uint32 ns), from azimuth ``atan2(y, x)``.

    ``stamp_ns`` is unused for the offset itself; it is kept so the signature mirrors the
    other time-aware functions. Non-finite points get 0.
    """
    del stamp_ns
    out = np.zeros(len(xyz), dtype=np.uint32)
    ok = _finite_rows(xyz)
    az = np.mod(np.arctan2(xyz[ok, 1].astype(np.float64), xyz[ok, 0].astype(np.float64)), 2 * np.pi)
    period_ns = scan_period_s * _NS_PER_S
    off = np.minimum(np.round(az / (2 * np.pi) * period_ns), period_ns - 1)
    out[ok] = off.astype(np.uint32)
    return out


def ring_index(arrays: CloudArrays) -> np.ndarray:
    """Ring per point: from the grid if organized, else elevation quantised to 16 levels."""
    n = len(arrays.xyz)
    if arrays.height > 1:
        return np.repeat(np.arange(arrays.height, dtype=np.uint16), arrays.width)[:n]
    out = np.zeros(n, dtype=np.uint16)
    ok = _finite_rows(arrays.xyz)
    p = arrays.xyz[ok].astype(np.float64)
    elev = np.degrees(np.arctan2(p[:, 2], np.hypot(p[:, 0], p[:, 1])))
    frac = (elev - _ELEV_MIN_DEG) / (_ELEV_MAX_DEG - _ELEV_MIN_DEG)
    out[ok] = np.clip(np.round(frac * (_N_RINGS - 1)), 0, _N_RINGS - 1).astype(np.uint16)
    return out


def inject_skew(
    xyz: np.ndarray,
    t_offset_ns: np.ndarray,
    stamp_ns: int,
    gt: GroundTruth,
    T_b_l: np.ndarray,  # noqa: N803
) -> np.ndarray:
    """Apply ``p_i <- T_l(t_i)^-1 T_l(t_scan) p_i``, ``T_l(t) = T_w_b(t) T_b_l``.

    Non-finite rows pass through unchanged.
    """
    out = np.array(xyz, copy=True)
    ok = _finite_rows(xyz)
    if not ok.any():
        return out
    r_bl, p_bl = T_b_l[:3, :3], T_b_l[:3, 3]
    t_i = stamp_ns + np.asarray(t_offset_ns, dtype=np.int64)[ok]
    r_i, p_i = gt.pose_at(t_i)
    r_s, p_s = gt.pose_at(np.array([stamp_ns], dtype=np.int64))
    rl_i = r_i @ r_bl
    pl_i = np.einsum("nij,j->ni", r_i, p_bl) + p_i
    rl_s = r_s[0] @ r_bl
    pl_s = r_s[0] @ p_bl + p_s[0]
    p_w = xyz[ok].astype(np.float64) @ rl_s.T + pl_s - pl_i  # (n,3), still in world axes
    out[ok] = np.einsum("nji,nj->ni", rl_i, p_w).astype(out.dtype)
    return out


def _grazing_mask(xyz: np.ndarray, height: int, width: int, grazing_deg: float) -> np.ndarray:
    """True where the estimated incidence angle exceeds ``grazing_deg`` (organized grid)."""
    ok = _finite_rows(xyz)
    p = np.where(ok[:, None], xyz.astype(np.float64), np.nan).reshape(height, width, 3)
    d_col = np.roll(p, -1, axis=1) - np.roll(p, 1, axis=1)
    d_row = np.empty_like(p)
    d_row[1:-1] = p[2:] - p[:-2]
    d_row[0] = p[1] - p[0]  # one-sided at ring edges
    d_row[-1] = p[-1] - p[-2]
    normal = np.cross(d_col, d_row).reshape(-1, 3)
    p = p.reshape(-1, 3)
    n_norm = np.linalg.norm(normal, axis=1)
    r_norm = np.linalg.norm(p, axis=1)
    valid = np.isfinite(n_norm) & np.isfinite(r_norm) & (n_norm > _MIN_NORMAL_NORM) & (r_norm > 0)
    cos_inc = np.zeros(len(p))
    cos_inc[valid] = np.abs(np.einsum("ij,ij->i", normal[valid], p[valid])) / (
        n_norm[valid] * r_norm[valid]
    )
    incidence_deg = np.degrees(np.arccos(np.clip(cos_inc, 0.0, 1.0)))
    return valid & (incidence_deg > grazing_deg)


def apply_dropout(arrays: CloudArrays, noise: LidarNoise, rng: np.random.Generator) -> np.ndarray:
    """Keep mask (N,) bool. Bernoulli dropout, higher at grazing incidence (organized only).

    Non-finite points are always dropped. Exactly N uniform draws are consumed.
    """
    n = len(arrays.xyz)
    p_drop = np.full(n, noise.dropout_p)
    if arrays.height > 1 and arrays.height * arrays.width == n:
        grazing = _grazing_mask(arrays.xyz, arrays.height, arrays.width, noise.grazing_deg)
        p_drop[grazing] = noise.dropout_p_grazing
    keep = rng.random(n) >= p_drop
    return keep & _finite_rows(arrays.xyz)


def inject_bad_points(xyz: np.ndarray, noise: LidarNoise, rng: np.random.Generator) -> np.ndarray:
    """Replace finite points by NaN (``nan_fraction``) and 0.3 m body returns (``near_fraction``).

    Counts are ``round(fraction * n_finite)``; the two sets are disjoint.
    """
    out = np.array(xyz, copy=True)
    idx = np.flatnonzero(_finite_rows(xyz))
    n_fin = len(idx)
    n_nan = int(round(noise.nan_fraction * n_fin))
    n_near = int(round(noise.near_fraction * n_fin))
    pick = rng.choice(idx, size=min(n_nan + n_near, n_fin), replace=False)
    nan_idx, near_idx = pick[:n_nan], pick[n_nan:]
    out[nan_idx] = np.nan
    d = rng.normal(size=(len(near_idx), 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    out[near_idx] = (_NEAR_RANGE_M * d).astype(out.dtype)
    return out


def process_scan(
    arrays: CloudArrays,
    stamp_ns: int,
    gt: GroundTruth,
    T_b_l: np.ndarray,  # noqa: N803
    noise: LidarNoise,
    rng: np.random.Generator,
    *,
    skew: bool = True,
    range_sigma_apply: bool = False,
) -> CloudArrays:
    """Apply per-point time, optional range noise, skew, bad points and dropout.

    Order: t/ring (if missing) -> range noise (N normal draws, only if ``range_sigma_apply``)
    -> skew -> bad points -> dropout. Dropped points are set to NaN (x, y, z), not removed, so
    ``height``/``width`` stay valid.
    """
    xyz = arrays.xyz
    t_ns = (
        arrays.t_ns
        if arrays.t_ns is not None
        else point_times_ns(xyz, stamp_ns, noise.scan_period_s)
    )
    ring = arrays.ring if arrays.ring is not None else ring_index(arrays)
    if range_sigma_apply:
        ok = _finite_rows(xyz)
        safe = np.where(ok[:, None], xyz.astype(np.float64), 0.0)
        r = np.sqrt(np.einsum("ij,ij->i", safe, safe))
        eps = rng.normal(0.0, noise.range_sigma_m, size=len(xyz))
        scale = np.ones(len(xyz))
        pos = ok & (r > 0)
        scale[pos] = 1.0 + eps[pos] / r[pos]
        xyz = (xyz.astype(np.float64) * scale[:, None]).astype(xyz.dtype)
    if skew:
        xyz = inject_skew(xyz, t_ns, stamp_ns, gt, T_b_l)
    xyz = inject_bad_points(xyz, noise, rng)
    keep = apply_dropout(replace(arrays, xyz=xyz), noise, rng)
    # Non-finite input rows (inf non-returns) pass through; only finite dropped points become NaN.
    drop = ~keep & _finite_rows(xyz)
    xyz = xyz.copy()
    xyz[drop] = np.nan
    return replace(arrays, xyz=xyz, t_ns=t_ns, ring=ring)
