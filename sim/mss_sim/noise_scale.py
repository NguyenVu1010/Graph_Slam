"""Noise multipliers on a profile, and the extra IMU noise the offline bridge adds.

A multiplier scales the simulated sensor only; the pipeline's believed noise is unchanged.
Gazebo already put the 1x IMU noise into the raw bag, so the IMU multiplier ``k >= 1`` is met by
adding independent noise with ``sigma * sqrt(k^2 - 1)``; the variances then add up to ``k^2``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace

from mss_sim.profile import ImuNoise, NoiseProfile

MAX_SCALE = 20.0
_MIN_SCALE = {"lidar": 0.0, "imu": 1.0, "gnss": 0.0, "odom": 0.0}
_GNSS_SCALED = (
    "sigma_h_m",
    "sigma_v_m",
    "drift_sigma_m",
    "multipath_amp_min_m",
    "multipath_amp_max_m",
)


@dataclass(frozen=True)
class NoiseScale:
    lidar: float = 1.0
    imu: float = 1.0
    gnss: float = 1.0
    odom: float = 1.0


@dataclass(frozen=True)
class ImuExtra:
    """Per-sample standard deviations: white noise and the bias random-walk step."""

    gyro_sigma: float  # rad/s
    acc_sigma: float  # m/s^2
    gyro_rw_step: float  # rad/s per sample
    acc_rw_step: float  # m/s^2 per sample


def validate_scale(s: NoiseScale) -> None:
    for f in fields(s):
        value = getattr(s, f.name)
        lo = _MIN_SCALE[f.name]
        if not (math.isfinite(value) and lo <= value <= MAX_SCALE):
            raise ValueError(f"noise.{f.name}: {value} outside [{lo}, {MAX_SCALE}]")


def scale_profile(p: NoiseProfile, s: NoiseScale) -> NoiseProfile:
    gnss = replace(p.gnss, **{n: getattr(p.gnss, n) * s.gnss for n in _GNSS_SCALED})
    lidar = replace(p.lidar, range_sigma_m=p.lidar.range_sigma_m * s.lidar)
    odom = replace(p.odom, speed_sigma_mps=p.odom.speed_sigma_mps * s.odom)
    return replace(p, gnss=gnss, lidar=lidar, odom=odom)


def imu_extra(imu: ImuNoise, k: float) -> ImuExtra | None:
    if k == 1.0:
        return None
    f = math.sqrt(k * k - 1.0)
    root_rate = math.sqrt(imu.rate_hz)
    return ImuExtra(
        gyro_sigma=imu.gyro_noise_density * root_rate * f,
        acc_sigma=imu.acc_noise_density * root_rate * f,
        gyro_rw_step=imu.gyro_random_walk / root_rate * f,
        acc_rw_step=imu.acc_random_walk / root_rate * f,
    )
