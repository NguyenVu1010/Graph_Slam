"""Allan deviation of a rate signal (gyro rad/s, accel m/s^2) and IEEE-952 style fits.

Model: ``adev(tau)^2 = N^2 / tau + K^2 * tau / 3`` with ``N`` the white-noise density
(unit/sqrt(Hz)) and ``K`` the rate random-walk density (unit/s/sqrt(Hz)).
"""

from __future__ import annotations

import allantools
import numpy as np

_WHITE_TAU_MAX_S = 1.0  # N is fitted on the -1/2 slope, tau <= 1 s
_WALK_SHARE_MIN = 0.5  # K is fitted where the walk term is at least this share of adev^2
_WALK_MIN_POINTS = 2
# Taus above max_tau / 8 average over too few independent windows (< ~20) to fit K reliably.
_WALK_TAU_FRACTION = 1.0 / 8.0


def oadev(x: np.ndarray, rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation on octave taus. Returns ``(taus_s, adev)``."""
    taus, adev, _err, _n = allantools.oadev(
        np.asarray(x, dtype=float), rate=float(rate_hz), data_type="freq", taus="octave"
    )
    ok = np.isfinite(adev) & (adev > 0)
    return np.asarray(taus)[ok], np.asarray(adev)[ok]


def fit_white_and_walk(taus: np.ndarray, adev: np.ndarray) -> tuple[float, float]:
    """Fit ``(N, K)``: N on the -1/2 slope (tau <= 1 s), K on the +1/2 slope (longest taus).

    ``N`` is the fixed-slope fit ``adev = N / sqrt(tau)``. ``K`` comes from
    ``K^2 = 3 (adev^2 - N^2 / tau) / tau`` averaged (in log) over the longest taus where the
    walk term dominates, excluding the three longest octaves (too few independent windows).
    ``K`` is NaN when the data are too short to show the +1/2 slope.
    """
    taus = np.asarray(taus, dtype=float)
    adev = np.asarray(adev, dtype=float)
    short = taus <= _WHITE_TAU_MAX_S
    if not short.any():
        short = taus == taus.min()
    n = float(np.exp(np.mean(np.log(adev[short] * np.sqrt(taus[short])))))
    walk_var = adev**2 - n**2 / taus
    use = (walk_var >= _WALK_SHARE_MIN * adev**2) & (taus <= taus.max() * _WALK_TAU_FRACTION)
    if use.sum() < _WALK_MIN_POINTS:
        return n, float("nan")
    k_sq = 3.0 * walk_var[use] / taus[use]
    return n, float(np.exp(0.5 * np.mean(np.log(k_sq))))


def theory_curve(taus: np.ndarray, n: float, k: float) -> np.ndarray:
    """``sqrt(N^2 / tau + K^2 tau / 3)``."""
    taus = np.asarray(taus, dtype=float)
    return np.sqrt(n**2 / taus + k**2 * taus / 3.0)
