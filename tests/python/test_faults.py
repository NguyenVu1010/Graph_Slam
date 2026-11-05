"""Timed sensor faults injected into a bridged bag (sensor impact explorer)."""

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mss_sim.bag_io import BagReader, cloud_to_arrays, stamp_ns
from mss_sim.bridge import BridgeReport
from mss_sim.faults import Fault, FaultReport, apply_faults, validate_fault
from mss_sim.geodesy import lla_to_enu

Bridged = tuple[Path, BridgeReport]
_NS = 1_000_000_000
_W0, _W1 = 2.0, 4.0  # fault window, s from the first ground-truth stamp


def _t0(bag: Path) -> int:
    with BagReader(bag) as r:
        return min(stamp_ns(m.header) for _, _, m in r.messages(["/ground_truth"]))


def _msgs(bag: Path, topic: str) -> list[tuple[int, Any, bytes]]:
    """(header stamp, decoded message, raw bytes) in log order."""
    with BagReader(bag) as r:
        dec = [m for _, _, m in r.messages([topic])]
        raw = [d for _, _, d in r.raw_messages([topic])]
    return [(stamp_ns(m.header), m, d) for m, d in zip(dec, raw, strict=True)]


def _inside(stamp: int, t0: int) -> bool:
    return t0 + _W0 * _NS <= stamp < t0 + _W1 * _NS


def _apply(
    bridged: Bridged, tmp_path: Path, *faults: Fault, seed: int = 7
) -> tuple[Path, Path, int, FaultReport]:
    src = bridged[0] / "bag"
    t0 = _t0(src)
    out = tmp_path / "faulted"
    report = apply_faults(src, out, faults, t0, seed)
    return src, out, t0, report


def test_outage_drops_only_window(bridged_bag: Bridged, tmp_path: Path) -> None:
    src, out, t0, rep = _apply(bridged_bag, tmp_path, Fault("gnss", "outage", _W0, _W1))
    before, after = _msgs(src, "/gnss/fix"), _msgs(out, "/gnss/fix")
    assert not any(_inside(s, t0) for s, _, _ in after)
    kept = [(s, d) for s, _, d in before if not _inside(s, t0)]
    assert [(s, d) for s, _, d in after] == kept
    assert rep.n_dropped["/gnss/fix"] == len(before) - len(kept) > 0


def test_lidar_blackout(bridged_bag: Bridged, tmp_path: Path) -> None:
    _, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("lidar", "blackout", _W0, _W1))
    assert not any(_inside(s, t0) for s, _, _ in _msgs(out, "/lidar/points"))


def test_lidar_range_limit(bridged_bag: Bridged, tmp_path: Path) -> None:
    src, out, t0, rep = _apply(bridged_bag, tmp_path, Fault("lidar", "range_limit", _W0, _W1, 3.0))
    before = {s: d for s, _, d in _msgs(src, "/lidar/points")}
    for s, m, d in _msgs(out, "/lidar/points"):
        if _inside(s, t0):
            r = np.linalg.norm(cloud_to_arrays(m).xyz.astype(np.float64), axis=1)
            assert np.nanmax(r) <= 3.0
        else:
            assert d == before[s]
    assert rep.n_modified["/lidar/points"] > 0


def test_lidar_sector_occlusion(bridged_bag: Bridged, tmp_path: Path) -> None:
    f = Fault("lidar", "sector_occlusion", _W0, _W1, 90.0)
    _, out, t0, _ = _apply(bridged_bag, tmp_path, f)
    for s, m, _ in _msgs(out, "/lidar/points"):
        if _inside(s, t0):
            xyz = cloud_to_arrays(m).xyz
            ang = np.degrees(np.abs(np.arctan2(xyz[:, 1], xyz[:, 0])))
            assert np.all(ang[np.isfinite(ang)] >= 45.0)


def _vec(v: Any) -> np.ndarray:
    return np.array([v.x, v.y, v.z])


def test_imu_gyro_bias_step(bridged_bag: Bridged, tmp_path: Path) -> None:
    f = Fault("imu", "gyro_bias_step", _W0, _W1, 0.02)
    src, out, t0, _ = _apply(bridged_bag, tmp_path, f)
    for (s, a, _), (_, b, _) in zip(_msgs(src, "/imu/data"), _msgs(out, "/imu/data"), strict=True):
        d = _vec(b.angular_velocity) - _vec(a.angular_velocity)
        want = np.array([0.0, 0.0, 0.02 if _inside(s, t0) else 0.0])
        np.testing.assert_allclose(d, want, atol=1e-12)


def test_imu_saturation(bridged_bag: Bridged, tmp_path: Path) -> None:
    _, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("imu", "saturation", _W0, _W1, 5.0))
    for s, m, _ in _msgs(out, "/imu/data"):
        if _inside(s, t0):
            assert np.all(np.abs(_vec(m.linear_acceleration)) <= 5.0)


def test_imu_freeze_repeats_last(bridged_bag: Bridged, tmp_path: Path) -> None:
    src, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("imu", "freeze", _W0, _W1))
    last = [m for s, m, _ in _msgs(src, "/imu/data") if s < t0 + _W0 * _NS][-1]
    inside = [m for s, m, _ in _msgs(out, "/imu/data") if _inside(s, t0)]
    assert inside
    for m in inside:
        np.testing.assert_array_equal(_vec(m.angular_velocity), _vec(last.angular_velocity))
        np.testing.assert_array_equal(_vec(m.linear_acceleration), _vec(last.linear_acceleration))


def _lla(m: Any) -> tuple[float, float, float]:
    return (m.latitude, m.longitude, m.altitude)


def test_gnss_offset_east(bridged_bag: Bridged, tmp_path: Path) -> None:
    src, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("gnss", "offset", _W0, _W1, 5.0))
    pairs = zip(_msgs(src, "/gnss/fix"), _msgs(out, "/gnss/fix"), strict=True)
    n = 0
    for (s, a, _), (_, b, _) in pairs:
        want = [5.0, 0.0, 0.0] if _inside(s, t0) else [0.0, 0.0, 0.0]
        np.testing.assert_allclose(lla_to_enu(_lla(b), _lla(a)), want, atol=1e-3)
        n += _inside(s, t0)
    assert n > 0


def test_gnss_jump_length(bridged_bag: Bridged, tmp_path: Path) -> None:
    src, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("gnss", "jump", _W0, _W1, 8.0))
    for (s, a, _), (_, b, _) in zip(_msgs(src, "/gnss/fix"), _msgs(out, "/gnss/fix"), strict=True):
        d = lla_to_enu(_lla(b), _lla(a))
        assert np.hypot(d[0], d[1]) == pytest.approx(8.0 if _inside(s, t0) else 0.0, abs=1e-3)


def test_odom_slip_scales_twist(bridged_bag: Bridged, tmp_path: Path) -> None:
    src, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("odom", "slip", _W0, _W1, 0.7))
    pairs = zip(_msgs(src, "/wheel/odom"), _msgs(out, "/wheel/odom"), strict=True)
    for (s, a, _), (_, b, _) in pairs:
        k = 0.7 if _inside(s, t0) else 1.0
        ta, tb = a.twist.twist, b.twist.twist
        assert tb.linear.x == pytest.approx(k * ta.linear.x, abs=1e-12)
        assert tb.angular.z == pytest.approx(k * ta.angular.z, abs=1e-12)


def test_odom_stuck_zero_twist(bridged_bag: Bridged, tmp_path: Path) -> None:
    _, out, t0, _ = _apply(bridged_bag, tmp_path, Fault("odom", "stuck", _W0, _W1))
    inside = [m for s, m, _ in _msgs(out, "/wheel/odom") if _inside(s, t0)]
    assert inside
    for m in inside:
        assert m.twist.twist.linear.x == 0.0 and m.twist.twist.angular.z == 0.0


@pytest.mark.parametrize(
    ("fault", "field"),
    [
        (Fault("imu", "outage", 1, 2), "kind"),
        (Fault("sonar", "blackout", 1, 2), "sensor"),
        (Fault("gnss", "outage", 2, 2), "t_end_s"),
        (Fault("gnss", "outage", -1, 2), "t_start_s"),
        (Fault("gnss", "outage", 1, 7), "t_end_s"),
        (Fault("lidar", "range_limit", 1, 2, 500.0), "magnitude"),
    ],
)
def test_validate_fault_rejects(fault: Fault, field: str) -> None:
    with pytest.raises(ValueError, match=field):
        validate_fault(fault, duration_s=6.0)


def test_validate_fault_fills_default() -> None:
    assert validate_fault(Fault("gnss", "offset", 1, 2), 6.0).magnitude == 5.0
    assert validate_fault(Fault("gnss", "outage", 1, 2, 3.0), 6.0).magnitude is None


def test_faults_deterministic(bridged_bag: Bridged, tmp_path: Path) -> None:
    f = Fault("imu", "spike", 0.0, 6.0, 20.0)
    src = bridged_bag[0] / "bag"
    t0 = _t0(src)
    outs = []
    for name, seed in (("a", 1), ("b", 1), ("c", 2)):
        apply_faults(src, tmp_path / name, [f], t0, seed)
        outs.append([d for _, _, d in _msgs(tmp_path / name, "/imu/data")])
    assert outs[0] == outs[1]
    assert outs[0] != outs[2]
