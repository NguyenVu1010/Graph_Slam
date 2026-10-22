from pathlib import Path

import pytest
import yaml

from mss_sim.profile import ProfileError, load_profile

REPO = Path(__file__).resolve().parents[2]
DEFAULT = REPO / "sim/noise/default.yaml"


def _write(tmp_path: Path, data: dict) -> Path:
    f = tmp_path / "p.yaml"
    f.write_text(yaml.safe_dump(data))
    return f


def _default() -> dict:
    return yaml.safe_load(DEFAULT.read_text())


def test_default_profile_loads() -> None:
    p = load_profile(DEFAULT)
    assert p.lidar.range_sigma_m == 0.02
    assert p.odom.kinematics_believed.wheel_radius_m == pytest.approx(0.101)
    assert p.odom.ticks_per_rev == 4096
    assert p.clock_offset_ns == {"imu": 0, "lidar": 0, "gnss": 0, "odom": 0}
    assert p.inflation_max == 5.0


def test_missing_field_names_key(tmp_path: Path) -> None:
    bad = _default()
    del bad["gnss"]["sigma_h_m"]
    with pytest.raises(ProfileError, match=r"gnss\.sigma_h_m"):
        load_profile(_write(tmp_path, bad))


def test_unknown_field_names_key(tmp_path: Path) -> None:
    bad = _default()
    bad["odom"]["kinematics_true"]["foo"] = 1.0
    with pytest.raises(ProfileError, match=r"odom\.kinematics_true\.foo"):
        load_profile(_write(tmp_path, bad))


def test_negative_sigma_rejected(tmp_path: Path) -> None:
    bad = _default()
    bad["lidar"]["range_sigma_m"] = -1
    with pytest.raises(ProfileError, match=r"lidar\.range_sigma_m"):
        load_profile(_write(tmp_path, bad))


def test_zero_rate_rejected(tmp_path: Path) -> None:
    bad = _default()
    bad["imu"]["rate_hz"] = 0
    with pytest.raises(ProfileError, match=r"imu\.rate_hz"):
        load_profile(_write(tmp_path, bad))


def test_inflation_max_below_one_rejected(tmp_path: Path) -> None:
    bad = _default()
    bad["inflation_max"] = 0.5
    with pytest.raises(ProfileError, match="inflation_max"):
        load_profile(_write(tmp_path, bad))


def test_wrong_type_rejected(tmp_path: Path) -> None:
    bad = _default()
    bad["gnss"]["sigma_v_m"] = "two"
    with pytest.raises(ProfileError, match=r"gnss\.sigma_v_m"):
        load_profile(_write(tmp_path, bad))
