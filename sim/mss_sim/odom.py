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
    """Encoder model: wheel rates integrate into ticks, read every ``1 / noise.rate_hz`` seconds.

    The joint-state stream may be faster than the encoder read rate (Gazebo publishes every
    physics step, 1 kHz): angles integrate between samples and a twist is produced only when a
    read window of ``1 / rate_hz`` has elapsed since the previous read, from the tick difference
    over that window. So ``/wheel/odom`` comes at ``rate_hz`` whatever the input rate, and the
    tick quantisation of a read is one tick per window, as on a real controller. The twist uses
    ``kinematics_believed`` (deliberately wrong), so integrated odometry is biased relative to
    the true motion that generated the wheel rates.

    ``step`` returns ``None`` between reads. Invalid input returns ``None``, changes no state and
    is counted per reason in ``errors`` (``missing_joint``: a wheel name absent or names and
    velocities of different length; ``non_finite``: a NaN or inf wheel rate;
    ``stamp_not_increasing``: a stamp <= the previous one); ``n_errors`` is their sum. The very
    first valid call only establishes the time base (no read yet) and draws no noise.
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
        self._window_ns = int(round(1e9 / noise.rate_hz))
        self._last_t_ns: int | None = None
        self._last_read_ns: int | None = None
        self._angle = [0.0, 0.0]  # rad, left/right
        self._ticks = [0, 0]  # ticks at the last read
        self.errors = {"missing_joint": 0, "non_finite": 0, "stamp_not_increasing": 0}

    @property
    def n_errors(self) -> int:
        return sum(self.errors.values())

    def step(self, t_ns: int, names: list[str], velocity_radps: list[float]) -> OdomTwist | None:
        # Validate everything before touching any state.
        if self._left not in names or self._right not in names or len(velocity_radps) != len(names):
            self.errors["missing_joint"] += 1
            return None
        rates = (
            velocity_radps[names.index(self._left)],
            velocity_radps[names.index(self._right)],
        )
        if not (math.isfinite(rates[0]) and math.isfinite(rates[1])):
            self.errors["non_finite"] += 1
            return None
        if self._last_t_ns is None or self._last_read_ns is None:
            self._last_t_ns = self._last_read_ns = t_ns
            return None
        if t_ns <= self._last_t_ns:
            self.errors["stamp_not_increasing"] += 1
            return None

        dt = (t_ns - self._last_t_ns) / 1e9
        self._last_t_ns = t_ns
        for i in range(2):
            self._angle[i] += rates[i] * dt
        if t_ns - self._last_read_ns < self._window_ns:
            return None
        window_s = (t_ns - self._last_read_ns) / 1e9
        self._last_read_ns = t_ns
        v_wheel = [0.0, 0.0]
        kin = self._noise.kinematics_believed
        for i in range(2):
            ticks = math.floor(self._angle[i] / self._rad_per_tick)
            d_ticks = ticks - self._ticks[i]
            self._ticks[i] = ticks
            v_wheel[i] = d_ticks * self._rad_per_tick / window_s * kin.wheel_radius_m

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
