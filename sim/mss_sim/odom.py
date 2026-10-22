"""Wheel-encoder quantisation and odometry with the kinematics the robot believes (SIM-07)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mss_sim.profile import OdomNoise


@dataclass(frozen=True)
class OdomTwist:
    v_mps: float
    omega_radps: float


class WheelQuantizer:
    """Integrates wheel rates into encoder ticks and derives a twist from tick deltas.

    The twist uses ``kinematics_believed`` (deliberately wrong), so integrated odometry is
    biased relative to the true motion that generated the wheel rates.

    ``step`` returns ``None`` and increments ``n_errors`` when a joint name is missing or the
    stamp does not increase. The very first valid call only establishes the time base (there
    is no dt yet): it returns ``None`` without counting an error and draws no noise.
    """

    def __init__(
        self,
        noise: OdomNoise,
        rng: np.random.Generator,
        left_joint: str = "left_wheel_joint",
        right_joint: str = "right_wheel_joint",
    ) -> None:
        self._noise = noise
        self._rng = rng
        self._left = left_joint
        self._right = right_joint
        self._rad_per_tick = 2.0 * math.pi / noise.ticks_per_rev
        self._last_t_ns: int | None = None
        self._angle = [0.0, 0.0]  # rad, left/right
        self._ticks = [0, 0]
        self.n_errors = 0

    def step(self, t_ns: int, names: list[str], velocity_radps: list[float]) -> OdomTwist | None:
        if self._left not in names or self._right not in names:
            self.n_errors += 1
            return None
        if self._last_t_ns is None:
            self._last_t_ns = t_ns
            return None
        if t_ns <= self._last_t_ns:
            self.n_errors += 1
            return None

        dt = (t_ns - self._last_t_ns) / 1e9
        self._last_t_ns = t_ns
        rates = (
            velocity_radps[names.index(self._left)],
            velocity_radps[names.index(self._right)],
        )
        v_wheel = [0.0, 0.0]
        kin = self._noise.kinematics_believed
        for i in range(2):
            self._angle[i] += rates[i] * dt
            ticks = math.floor(self._angle[i] / self._rad_per_tick)
            d_ticks = ticks - self._ticks[i]
            self._ticks[i] = ticks
            v_wheel[i] = d_ticks * self._rad_per_tick / dt * kin.wheel_radius_m

        v = 0.5 * (v_wheel[0] + v_wheel[1]) + float(
            self._rng.normal(0.0, self._noise.speed_sigma_mps)
        )
        omega = (v_wheel[1] - v_wheel[0]) / kin.track_m
        return OdomTwist(v, omega)


def integrate_pose(twists: list[tuple[int, OdomTwist]]) -> np.ndarray:
    """Midpoint integration of stamped twists into ``[x, y, yaw]`` rows starting at zero."""
    out = np.zeros((len(twists), 3))
    x = y = yaw = 0.0
    for k in range(1, len(twists)):
        dt = (twists[k][0] - twists[k - 1][0]) / 1e9
        tw = twists[k][1]
        yaw_mid = yaw + tw.omega_radps * dt / 2.0
        x += tw.v_mps * dt * math.cos(yaw_mid)
        y += tw.v_mps * dt * math.sin(yaw_mid)
        yaw += tw.omega_radps * dt
        out[k] = (x, y, yaw)
    return out
