import numpy as np
import pytest

from mss_sim.geodesy import ecef_to_lla, enu_to_lla, lla_to_ecef, lla_to_enu

ORIGIN = (21.0278, 105.8342, 12.0)


def test_round_trip_under_1mm() -> None:
    rng = np.random.default_rng(0)
    enu = rng.uniform(-5000, 5000, size=(1000, 3))
    back = lla_to_enu(enu_to_lla(enu, ORIGIN), ORIGIN)
    assert back.shape == enu.shape
    assert np.max(np.abs(back - enu)) < 1e-3


def test_east_is_longitude() -> None:
    lat, lon, alt = enu_to_lla(np.array([100.0, 0.0, 0.0]), ORIGIN)
    assert lon > ORIGIN[1]
    # A straight tangent-plane eastward line leaves the ellipsoid by ~d^2/2R, i.e. ~3e-9 deg
    # of latitude at 100 m, so 1e-9 deg is not achievable; 1e-8 deg is ~1 mm.
    assert lat == pytest.approx(ORIGIN[0], abs=1e-8)
    assert alt == pytest.approx(ORIGIN[2], abs=1e-3)


def test_north_and_up() -> None:
    lat, lon, alt = enu_to_lla(np.array([0.0, 100.0, 50.0]), ORIGIN)
    assert lat > ORIGIN[0]
    assert lon == pytest.approx(ORIGIN[1], abs=1e-8)
    assert alt == pytest.approx(ORIGIN[2] + 50.0, abs=1e-3)


def test_origin_maps_to_zero() -> None:
    assert np.allclose(lla_to_enu(ORIGIN, ORIGIN), 0.0, atol=1e-6)


def test_ecef_known_values() -> None:
    assert np.allclose(lla_to_ecef((0.0, 0.0, 0.0)), [6378137.0, 0.0, 0.0])
    assert np.allclose(lla_to_ecef((90.0, 0.0, 0.0)), [0.0, 0.0, 6356752.314245], atol=1e-5)


def test_ecef_round_trip_batch() -> None:
    lla = np.array([[21.0, 105.0, 10.0], [-33.9, 151.2, 100.0], [0.0, -179.0, -50.0]])
    back = ecef_to_lla(lla_to_ecef(lla))
    assert back.shape == (3, 3)
    assert np.allclose(back, lla, atol=1e-9)


def test_single_in_single_out() -> None:
    lla = ecef_to_lla(lla_to_ecef(ORIGIN))
    assert isinstance(lla, tuple)
    assert lla == pytest.approx(ORIGIN, abs=1e-8)
