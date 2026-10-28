"""Pure-pursuit waypoint follower for the simulated robot (ADR-011 sections 2 and 5). No ROS.

``PurePursuit.command(t_s, x, y, yaw)`` returns the ``(v, omega)`` to send as ``/cmd_vel`` from
the robot's planar pose in world. Behaviour:

- the first ``STILL_S`` seconds after the first call command zero (IMU initialisation and the
  static window of the noise report, ADR-011 section 5);
- the target speed is the route's ``speed_mps`` times a factor drawn uniformly in
  ``1 +- SPEED_VARIATION`` for every ``SPEED_PIECE_M`` metres of route, from the ``driver`` stream
  of the scenario seed (``SeedSequence(seed).spawn(6)[5]``; streams 0..4 are the noise bridge's
  [lidar, gnss, odom, imu, mutate]);
- speed, acceleration and yaw rate stay within ``Limits`` (ADR-011 section 2); the speed also
  drops as ``sqrt(2 a d)`` before the last waypoint and so that ``|v * curvature| <= omega_max``;
- once the route left is within ``GOAL_TOLERANCE_M`` it ramps the speed down to zero (yaw rate
  zero), then sets ``done`` and commands zero from then on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mss_sim.scenario import Route

STILL_S = 5.0  # still start, s (ADR-011 section 5)
SPEED_VARIATION = 0.10  # +-10 % target-speed variation (ADR-011 section 5)
SPEED_PIECE_M = 5.0  # route length per drawn speed factor, m
GOAL_TOLERANCE_M = 0.10  # stop when this close to the last waypoint, m
DRIVER_STREAM = 5  # index of the driver stream in SeedSequence(seed).spawn(N_STREAMS)
N_STREAMS = 6
_MIN_LOOKAHEAD_M = 1e-6


@dataclass(frozen=True)
class Limits:
    """Command limits; the values come from ``sim/models/robot.yaml`` ``drive``."""

    v_max_mps: float
    a_max_mps2: float
    omega_max_radps: float

    @classmethod
    def from_robot(cls, robot: dict) -> Limits:
        d = robot["drive"]
        return cls(
            float(d["max_linear_velocity_mps"]),
            float(d["max_linear_acceleration_mps2"]),
            float(d["max_angular_velocity_radps"]),
        )


def driver_rng(seed: int) -> np.random.Generator:
    """The ``driver`` stream of a scenario seed."""
    return np.random.default_rng(np.random.SeedSequence(seed).spawn(N_STREAMS)[DRIVER_STREAM])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PurePursuit:
    def __init__(self, route: Route, limits: Limits, seed: int) -> None:
        self._p = np.asarray(route.waypoints, dtype=float)
        if len(self._p) < 2:
            raise ValueError("route needs at least 2 waypoints")
        seg = np.diff(self._p, axis=0)
        self._seg_len = np.linalg.norm(seg, axis=1)
        if np.any(self._seg_len <= 0):
            raise ValueError("route has repeated consecutive waypoints")
        self._cum = np.concatenate([[0.0], np.cumsum(self._seg_len)])  # arc length at waypoints
        self.length_m = float(self._cum[-1])
        self._lookahead = route.lookahead_m
        self._limits = limits
        n_pieces = int(math.ceil(self.length_m / SPEED_PIECE_M))
        factors = 1.0 + driver_rng(seed).uniform(-SPEED_VARIATION, SPEED_VARIATION, n_pieces)
        self._piece_speed = np.minimum(route.speed_mps * factors, limits.v_max_mps)
        self._idx = 0  # current segment (monotonic)
        self._t0: float | None = None
        self._t_prev: float | None = None
        self._v = 0.0
        self.done = False

    def target_speed(self, s_m: float) -> float:
        """Target speed at arc length ``s_m`` (before limits)."""
        k = min(int(s_m / SPEED_PIECE_M), len(self._piece_speed) - 1)
        return float(self._piece_speed[max(k, 0)])

    def _project(self, xy: np.ndarray) -> tuple[float, np.ndarray]:
        """Arc length and point of the closest point on the current or next segment."""
        best: tuple[float, int, float] | None = None  # (distance, segment, fraction)
        for i in range(self._idx, min(self._idx + 2, len(self._seg_len))):
            a, d = self._p[i], self._p[i + 1] - self._p[i]
            u = float(np.clip(np.dot(xy - a, d) / np.dot(d, d), 0.0, 1.0))
            dist = float(np.linalg.norm(a + u * d - xy))
            if best is None or dist < best[0]:
                best = (dist, i, u)
        assert best is not None
        _, self._idx, u = best
        s = float(self._cum[self._idx] + u * self._seg_len[self._idx])
        return s, self.point_at(s)

    def point_at(self, s_m: float) -> np.ndarray:
        """Point at arc length ``s_m`` (clamped to the route)."""
        s = min(max(s_m, 0.0), self.length_m)
        i = min(int(np.searchsorted(self._cum, s, side="right")) - 1, len(self._seg_len) - 1)
        u = (s - self._cum[i]) / self._seg_len[i]
        return self._p[i] + u * (self._p[i + 1] - self._p[i])

    def command(self, t_s: float, x: float, y: float, yaw: float) -> tuple[float, float]:
        """``(v_mps, omega_radps)`` at time ``t_s`` (s, any origin) for pose ``(x, y, yaw)``."""
        if self._t0 is None:
            self._t0 = t_s
        dt = 0.0 if self._t_prev is None else max(t_s - self._t_prev, 0.0)
        self._t_prev = t_s
        lim = self._limits
        if self.done or t_s - self._t0 < STILL_S:
            self._v = 0.0
            return 0.0, 0.0
        xy = np.array([x, y])
        s, _ = self._project(xy)
        remaining = self.length_m - s
        step = lim.a_max_mps2 * dt
        if self._idx == len(self._seg_len) - 1 and remaining <= GOAL_TOLERANCE_M:
            self._v = max(self._v - step, 0.0)  # ramp down within the acceleration limit
            self.done = self._v == 0.0
            return self._v, 0.0
        goal = self.point_at(s + self._lookahead)
        d = goal - xy
        dist = max(float(np.linalg.norm(d)), _MIN_LOOKAHEAD_M)
        alpha = _wrap(math.atan2(d[1], d[0]) - yaw)
        curvature = 2.0 * math.sin(alpha) / dist
        # Brake so that the speed reaches zero at the goal tolerance.
        to_stop = max(remaining - GOAL_TOLERANCE_M, 0.0)
        v_target = min(self.target_speed(s), math.sqrt(2.0 * lim.a_max_mps2 * to_stop))
        if abs(curvature) > 0.0:
            v_target = min(v_target, lim.omega_max_radps / abs(curvature))
        self._v = min(max(v_target, self._v - step, 0.0), self._v + step, lim.v_max_mps)
        omega = float(np.clip(self._v * curvature, -lim.omega_max_radps, lim.omega_max_radps))
        return self._v, omega
