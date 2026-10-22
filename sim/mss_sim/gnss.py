"""GNSS degrader (SIM-06): outage, multipath, slow drift, degraded mode and spatial zones."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point

from mss_sim.geodesy import Lla, enu_to_lla
from mss_sim.profile import GnssNoise
from mss_sim.scenario import GnssSegment, GnssZone

_STATUS_FIX = 0  # sensor_msgs NavSatStatus STATUS_FIX


@dataclass(frozen=True)
class GnssSample:
    lla: Lla
    cov_enu_diag: np.ndarray  # (3,) variances [E, N, U], m^2
    status: int


class GnssDegrader:
    """Turns a true antenna position (ENU, ``p_e_g``) into a degraded GNSS fix.

    Segment times are scenario-relative: ``(t_ns - t0_ns)`` in seconds. A zone containing the
    position wins over a segment. Outside every segment and zone the mode is ``"nominal"``.

    rng draw order per step (stable for every mode, so same-seed runs reproduce):
    drift normals (3) -> white normals (3) -> multipath decisions (only while in multipath
    mode with no active event: start uniform, then amplitude, direction, hold uniforms).
    Outage still consumes the drift and white draws and returns ``None``.
    """

    def __init__(
        self,
        noise: GnssNoise,
        segments: list[GnssSegment],
        zones: list[GnssZone],
        origin_lla: Lla,
        rng: np.random.Generator,
        t0_ns: int,
    ) -> None:
        self._n = noise
        self._segments = segments
        self._zones = [(z.mode, z.polygon()) for z in zones]
        self._origin = origin_lla
        self._rng = rng
        self._t0_ns = t0_ns
        self._drift: np.ndarray | None = None
        self._last_t_s = 0.0
        self._event_bias = np.zeros(3)
        self._event_end_s = -math.inf

    def mode_at(self, t_s: float, p_enu: np.ndarray) -> str:
        pt = Point(float(p_enu[0]), float(p_enu[1]))
        for mode, poly in self._zones:
            if poly.covers(pt):
                return mode
        for seg in self._segments:
            if seg.t_start_s <= t_s <= seg.t_end_s:
                return seg.mode
        return "nominal"

    def step(self, t_ns: int, p_e_g_true: np.ndarray) -> GnssSample | None:
        n = self._n
        t_s = (t_ns - self._t0_ns) * 1e-9
        p_true = np.asarray(p_e_g_true, dtype=float)

        drift_n = self._rng.standard_normal(3)
        white_n = self._rng.standard_normal(3)
        if self._drift is None:
            self._drift = n.drift_sigma_m * drift_n
            dt = 0.0
        else:
            dt = t_s - self._last_t_s
            a = math.exp(-dt / n.drift_tau_s)
            self._drift = a * self._drift + n.drift_sigma_m * math.sqrt(1.0 - a * a) * drift_n
        self._last_t_s = t_s

        mode = self.mode_at(t_s, p_true)
        if mode == "outage":
            return None

        sigma = np.array([n.sigma_h_m, n.sigma_h_m, n.sigma_v_m])
        scale = n.degraded_factor if mode == "degraded" else 1.0

        if mode == "multipath" and t_s >= self._event_end_s:
            if self._rng.random() < n.multipath_start_rate_per_s * dt:
                amp = self._rng.uniform(n.multipath_amp_min_m, n.multipath_amp_max_m)
                ang = self._rng.uniform(0.0, 2.0 * math.pi)
                hold = self._rng.uniform(n.multipath_hold_min_s, n.multipath_hold_max_s)
                self._event_bias = np.array([amp * math.cos(ang), amp * math.sin(ang), 0.0])
                self._event_end_s = t_s + hold
        bias = self._event_bias if t_s < self._event_end_s else np.zeros(3)

        p = p_true + self._drift + bias + sigma * scale * white_n
        cov = (sigma * scale) ** 2
        return GnssSample(lla=enu_to_lla(p, self._origin), cov_enu_diag=cov, status=_STATUS_FIX)
