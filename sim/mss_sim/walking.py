"""6-DoF walking trajectory of a handheld rig (ADR-029, backlog L1). No ROS.

``walk(route, gait, limits, seed)`` returns the body pose ``T_w_b`` (position ``p_w_b``, rotation
``R_w_b``), the velocity ``v_w_b`` (in w) and the angular velocity ``omega_b`` (in b) sampled at
``rate_hz``:

- the horizontal path is the robot's: ``mss_sim.pursuit.PurePursuit`` on the route (5 s still
  start, seeded +-10 % speed, ``limits``), commands at ``CMD_RATE_HZ``, integrated as a unicycle
  on an ``INTERNAL_HZ`` grid;
- the gait adds, with ``k = min(v / gait.cruise_speed_mps, 1)`` scaling the amplitudes and the
  step phase ``phi`` advancing at ``2 pi k f_step``: height ``h + k A_z cos(phi)``, lateral sway
  ``k A_y sin(phi / 2)`` along body y, roll ``k A_roll sin(phi / 2)``, pitch ``k A_pitch sin(phi)``
  and yaw ``k A_yaw sin(phi / 2)``; ``R_w_b = Rz(yaw) Ry(pitch) Rx(roll)``;
- velocities are central differences on the internal grid (so an IMU derived from them matches
  the poses), then every ``INTERNAL_HZ / rate_hz``-th sample is kept. At the first and last grid
  samples ``v_w_b`` is one-sided (``np.gradient``) and ``omega_b`` repeats its neighbour, so the
  ends are valid to first order also when the walk stops mid-motion.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from mss_sim.pursuit import Limits, PurePursuit
from mss_sim.scenario import Route

INTERNAL_HZ = 1000  # integration and differentiation grid, Hz
CMD_RATE_HZ = 50  # pursuit command rate, Hz (as mss_sim.driver)
HANDHELD_YAML = Path(__file__).resolve().parents[1] / "models" / "handheld.yaml"


@dataclass(frozen=True)
class Gait:
    """Gait of ``sim/models/handheld.yaml`` (amplitudes are half peak-to-peak)."""

    body_height_m: float
    cruise_speed_mps: float  # speed at which the amplitudes reach their full value
    step_frequency_hz: float
    bounce_amplitude_m: float
    sway_amplitude_m: float
    roll_amplitude_rad: float
    pitch_amplitude_rad: float
    yaw_amplitude_rad: float

    @classmethod
    def load(cls, path: Path = HANDHELD_YAML) -> tuple[Gait, Limits]:
        """Gait and drive limits from the handheld description; every value must be > 0."""
        d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        g = d["gait"]
        values = {
            "body_height_m": d["body_height_m"],
            "cruise_speed_mps": g["cruise_speed_mps"],
            "step_frequency_hz": g["step_frequency_hz"],
            "bounce_amplitude_m": g["bounce_amplitude_m"],
            "sway_amplitude_m": g["sway_amplitude_m"],
            "roll_amplitude_rad": math.radians(g["roll_amplitude_deg"]),
            "pitch_amplitude_rad": math.radians(g["pitch_amplitude_deg"]),
            "yaw_amplitude_rad": math.radians(g["yaw_amplitude_deg"]),
        }
        for key, v in values.items():
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not v > 0:
                raise ValueError(f"{path}: {key}: must be a number > 0, got {v!r}")
        return cls(**{k: float(v) for k, v in values.items()}), Limits.from_robot(d)


@dataclass(frozen=True)
class Walk:
    t_s: np.ndarray  # (N,) s from the start
    p_w_b: np.ndarray  # (N, 3) m
    R_w_b: np.ndarray  # (N, 3, 3)
    v_w_b: np.ndarray  # (N, 3) m/s, in w
    omega_b: np.ndarray  # (N, 3) rad/s, in b
    done: bool  # the route was completed within t_max_s


def walk(
    route: Route,
    gait: Gait,
    limits: Limits,
    seed: int,
    *,
    t_max_s: float,
    rate_hz: float = 200.0,
    start_xy_yaw: tuple[float, float, float] | None = None,
) -> Walk:
    """Sample the walk at ``rate_hz`` (must divide ``INTERNAL_HZ``) for ``t_max_s`` seconds.

    ``start_xy_yaw`` defaults to the first waypoint, heading to the second.
    """
    decim = INTERNAL_HZ / rate_hz
    if not (decim >= 1 and decim == int(decim)):
        raise ValueError(f"rate_hz must divide {INTERNAL_HZ}, got {rate_hz}")
    wp = np.asarray(route.waypoints, dtype=float)
    if start_xy_yaw is None:
        d = wp[1] - wp[0]
        start_xy_yaw = (float(wp[0, 0]), float(wp[0, 1]), math.atan2(d[1], d[0]))
    n = int(round(t_max_s * INTERNAL_HZ)) + 1
    dt = 1.0 / INTERNAL_HZ
    per_cmd = INTERNAL_HZ // CMD_RATE_HZ
    pursuit = PurePursuit(route, limits, seed)
    x, y, psi = start_xy_yaw
    base = np.empty((n, 3))  # x, y, yaw of the unicycle
    speed = np.empty(n)
    v = w = 0.0
    for i in range(n):
        if i % per_cmd == 0:
            v, w = pursuit.command(i * dt, x, y, psi)
        base[i] = (x, y, psi)
        speed[i] = v
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi += w * dt
    k = np.minimum(speed / gait.cruise_speed_mps, 1.0)
    phi = 2.0 * math.pi * gait.step_frequency_hz * np.cumsum(k) * dt
    half = np.sin(phi / 2.0)
    yaw = base[:, 2] + k * gait.yaw_amplitude_rad * half
    sway = k * gait.sway_amplitude_m * half  # along body y of the base path
    p = np.column_stack(
        [
            base[:, 0] - np.sin(base[:, 2]) * sway,
            base[:, 1] + np.cos(base[:, 2]) * sway,
            gait.body_height_m + k * gait.bounce_amplitude_m * np.cos(phi),
        ]
    )
    roll = k * gait.roll_amplitude_rad * half
    pitch = k * gait.pitch_amplitude_rad * np.sin(phi)
    rot = Rotation.from_euler("ZYX", np.column_stack([yaw, pitch, roll]))  # Rz Ry Rx
    v_w = np.gradient(p, dt, axis=0)
    omega = np.zeros((n, 3))  # central difference: log(R_{i-1}^T R_{i+1}) / (2 dt)
    omega[1:-1] = (rot[:-2].inv() * rot[2:]).as_rotvec() / (2.0 * dt)
    omega[0], omega[-1] = omega[1], omega[-2]  # one-sided ends, as np.gradient for v_w
    keep = slice(None, None, int(decim))
    return Walk(
        t_s=np.arange(n)[keep] * dt,
        p_w_b=p[keep],
        R_w_b=rot.as_matrix()[keep],
        v_w_b=v_w[keep],
        omega_b=omega[keep],
        done=pursuit.done,
    )
