"""Noise multipliers for the sensor impact explorer and the bridge's extra IMU noise."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from conftest import _REPO, e01_scenario

from mss_sim.bag_io import BagReader
from mss_sim.bridge import run_bridge
from mss_sim.noise_scale import NoiseScale, imu_extra, scale_profile, validate_scale
from mss_sim.profile import load_profile
from mss_sim.synth import SynthInfo

Fixture = tuple[Path, SynthInfo]
_PROFILE = _REPO / "sim/noise/default.yaml"


def test_scale_profile_multiplies_only_listed_fields() -> None:
    p = load_profile(_PROFILE)
    s = scale_profile(p, NoiseScale(lidar=2, gnss=3, odom=0))
    assert s.lidar.range_sigma_m == pytest.approx(2 * p.lidar.range_sigma_m)
    for name in (
        "sigma_h_m",
        "sigma_v_m",
        "drift_sigma_m",
        "multipath_amp_min_m",
        "multipath_amp_max_m",
    ):
        assert getattr(s.gnss, name) == pytest.approx(3 * getattr(p.gnss, name)), name
    assert s.odom.speed_sigma_mps == 0
    assert s.imu == p.imu
    assert s.clock_offset_ns == p.clock_offset_ns
    assert s.gnss.rate_hz == p.gnss.rate_hz
    assert s.lidar.dropout_p == p.lidar.dropout_p


@pytest.mark.parametrize(
    ("scale", "field"),
    [(NoiseScale(imu=0.5), "imu"), (NoiseScale(gnss=-1), "gnss"), (NoiseScale(lidar=21), "lidar")],
)
def test_validate_scale_rejects(scale: NoiseScale, field: str) -> None:
    with pytest.raises(ValueError, match=field):
        validate_scale(scale)


def test_validate_scale_accepts_bounds() -> None:
    validate_scale(NoiseScale(lidar=0, imu=1, gnss=20, odom=0))


def test_imu_extra_none_at_one() -> None:
    assert imu_extra(load_profile(_PROFILE).imu, 1.0) is None


def test_imu_extra_values() -> None:
    imu = load_profile(_PROFILE).imu
    e = imu_extra(imu, 3.0)
    f = math.sqrt(8.0)
    assert e is not None
    assert e.gyro_sigma == pytest.approx(imu.gyro_noise_density * math.sqrt(imu.rate_hz) * f)
    assert e.acc_sigma == pytest.approx(imu.acc_noise_density * math.sqrt(imu.rate_hz) * f)
    assert e.gyro_rw_step == pytest.approx(imu.gyro_random_walk / math.sqrt(imu.rate_hz) * f)
    assert e.acc_rw_step == pytest.approx(imu.acc_random_walk / math.sqrt(imu.rate_hz) * f)


def _raw_msgs(bag: Path) -> list[tuple[str, int, bytes]]:
    with BagReader(bag) as r:
        return list(r.raw_messages(list(r.topics())))


def _bridge(raw_bag: Fixture, out: Path, **kw: object) -> Path:
    raw, info = raw_bag
    sc = e01_scenario(info)
    run_bridge(raw, sc, load_profile(sc.profile), out, seed=42, **kw)
    return out / "bag"


def test_bridge_identity_when_unscaled(raw_bag: Fixture, tmp_path: Path) -> None:
    a = _bridge(raw_bag, tmp_path / "a")
    b = _bridge(raw_bag, tmp_path / "b", imu_extra=None)
    assert _raw_msgs(a) == _raw_msgs(b)


def _imu(bag: Path) -> tuple[np.ndarray, np.ndarray]:
    gyro, acc = [], []
    with BagReader(bag) as r:
        for _, _, m in r.messages(["/imu/data"]):
            w, a = m.angular_velocity, m.linear_acceleration
            gyro.append((w.x, w.y, w.z))
            acc.append((a.x, a.y, a.z))
    return np.asarray(gyro), np.asarray(acc)


def test_bridge_imu_extra_variance(raw_bag: Fixture, tmp_path: Path) -> None:
    e = imu_extra(load_profile(_PROFILE).imu, 3.0)
    assert e is not None
    e = replace(e, gyro_rw_step=0.0, acc_rw_step=0.0)
    g0, a0 = _imu(_bridge(raw_bag, tmp_path / "a"))
    g1, a1 = _imu(_bridge(raw_bag, tmp_path / "b", imu_extra=e))
    # Pooled over the three axes (~3600 samples): sampling error of the variance ~2.4 %.
    assert np.var(g1 - g0) == pytest.approx(e.gyro_sigma**2, rel=0.10)
    assert np.var(a1 - a0) == pytest.approx(e.acc_sigma**2, rel=0.10)
