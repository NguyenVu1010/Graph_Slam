"""SIM-04: LiDAR per-point time, skew, dropout and bad points."""

import warnings

import numpy as np

from mss_sim.bag_io import CloudArrays
from mss_sim.gt import GroundTruth
from mss_sim.lidar import (
    apply_dropout,
    inject_bad_points,
    inject_skew,
    point_times_ns,
    process_scan,
    ring_index,
)
from mss_sim.profile import LidarNoise

_H, _W = 16, 720
_T_SCAN_S = 0.1
_STAMP_NS = 2_000_000_000
_NOISE = LidarNoise(
    range_sigma_m=0.02,
    edge_jump_m=0.10,
    min_incidence_cos=0.3,
    render_model={"form": "none"},
    dropout_p=0.02,
    dropout_p_grazing=0.3,
    grazing_deg=80.0,
    scan_period_s=_T_SCAN_S,
    nan_fraction=0.001,
    near_fraction=0.001,
    min_range_m=0.5,
)


def _make_gt(omega: float) -> GroundTruth:
    t = np.arange(0, 5_000_000_001, 10_000_000, dtype=np.int64)
    half = 0.5 * omega * t * 1e-9
    q = np.stack([np.cos(half), 0 * half, 0 * half, np.sin(half)], axis=1)  # w, x, y, z
    return GroundTruth(t_ns=t, p_w_b=np.zeros((len(t), 3)), q_wxyz=q)


def _grid(elev_deg: np.ndarray, range_m: float | np.ndarray) -> np.ndarray:
    """Organized (H*W, 3) points; column c has azimuth c*0.5 deg, row r elevation elev_deg[r]."""
    az = np.radians(np.arange(_W) * 0.5)
    el = np.radians(elev_deg)[:, None] * np.ones((1, _W))
    r = np.broadcast_to(range_m, el.shape)
    xyz = np.stack(
        [r * np.cos(el) * np.cos(az), r * np.cos(el) * np.sin(az), r * np.sin(el)], axis=-1
    )
    return xyz.reshape(-1, 3).astype(np.float32)


def _cloud(xyz: np.ndarray, height: int = _H, width: int = _W) -> CloudArrays:
    return CloudArrays(
        xyz=xyz,
        intensity=np.ones(len(xyz), np.float32),
        t_ns=None,
        ring=None,
        height=height,
        width=width,
    )


def test_static_robot_cloud_unchanged() -> None:
    gt = _make_gt(0.0)
    xyz = _grid(np.linspace(-15, 15, _H), 8.0).astype(np.float64)
    t = point_times_ns(xyz, _STAMP_NS, _T_SCAN_S)
    out = inject_skew(xyz, t, _STAMP_NS, gt, np.eye(4))
    np.testing.assert_allclose(out, xyz, atol=1e-12)


def test_constant_yaw_rate_matches_analytic() -> None:
    omega = 1.0
    gt = _make_gt(omega)
    xyz = _grid(np.linspace(-15, 15, _H), 8.0).astype(np.float64)
    t = point_times_ns(xyz, _STAMP_NS, _T_SCAN_S)
    out = inject_skew(xyz, t, _STAMP_NS, gt, np.eye(4))
    a = -omega * t * 1e-9
    c, s = np.cos(a), np.sin(a)
    expect = np.stack(
        [c * xyz[:, 0] - s * xyz[:, 1], s * xyz[:, 0] + c * xyz[:, 1], xyz[:, 2]], axis=1
    )
    np.testing.assert_allclose(out, expect, atol=1e-3)
    assert np.abs(out - xyz).max() > 0.1  # skew is actually applied


def test_unorganized_cloud_uses_atan2() -> None:
    xyz = _grid(np.linspace(-15, 15, _H), 8.0)
    t_org = point_times_ns(xyz, _STAMP_NS, _T_SCAN_S)
    perm = np.random.default_rng(0).permutation(len(xyz))
    t_shuf = point_times_ns(xyz[perm], _STAMP_NS, _T_SCAN_S)
    np.testing.assert_array_equal(t_shuf, t_org[perm])
    assert t_org.dtype == np.uint32
    assert t_org.max() < _T_SCAN_S * 1e9
    flat = _cloud(xyz[perm], height=1, width=len(xyz))
    ring = ring_index(flat)
    assert ring.dtype == np.uint16
    # elevations were linspace(-15, 15, 16): quantisation recovers the row index
    np.testing.assert_array_equal(ring, np.repeat(np.arange(_H), _W)[perm])
    org_ring = ring_index(_cloud(xyz))
    np.testing.assert_array_equal(org_ring, np.repeat(np.arange(_H), _W))


def test_nan_inf_passthrough() -> None:
    xyz = _grid(np.linspace(-15, 15, _H), 8.0)
    xyz[5] = np.inf
    xyz[100] = np.nan
    gt = _make_gt(1.0)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        out = process_scan(
            _cloud(xyz),
            _STAMP_NS,
            gt,
            np.eye(4),
            _NOISE,
            np.random.default_rng(1),
            range_sigma_apply=True,
        )
    assert np.isinf(out.xyz[5]).all()
    assert np.isnan(out.xyz[100]).all()
    assert out.t_ns is not None and out.t_ns[5] == 0
    assert out.height == _H and out.width == _W and len(out.xyz) == _H * _W


def test_dropout_rate_and_grazing() -> None:
    rng = np.random.default_rng(2)
    n = 100_000
    uni = _cloud(rng.normal(size=(n, 3)).astype(np.float32) + 5.0, height=1, width=n)
    kept = apply_dropout(uni, _NOISE, rng).mean()
    assert 0.975 <= kept <= 0.985
    # floor plane z=-1 seen at ~85 deg incidence (rays at -5 deg elevation)
    elev = np.linspace(-5.5, -4.5, _H)
    r = 1.0 / np.sin(np.radians(-elev))[:, None] * np.ones((1, _W))
    xyz = _grid(elev, r)
    np.testing.assert_allclose(xyz[:, 2], -1.0, atol=1e-5)
    kept_g = apply_dropout(_cloud(xyz), _NOISE, np.random.default_rng(3)).mean()
    assert kept_g <= 0.75
    # a wall facing the sensor (sphere of radius 8) is not grazing
    kept_n = apply_dropout(
        _cloud(_grid(np.linspace(-15, 15, _H), 8.0)), _NOISE, np.random.default_rng(3)
    ).mean()
    assert kept_n >= 0.95


def test_bad_points_fractions() -> None:
    n = 20_000
    noise = LidarNoise(**{**_NOISE.__dict__, "nan_fraction": 0.01, "near_fraction": 0.005})
    xyz = np.random.default_rng(4).uniform(2.0, 9.0, size=(n, 3)).astype(np.float32)
    out = inject_bad_points(xyz, noise, np.random.default_rng(5))
    n_nan = int(np.isnan(out).any(axis=1).sum())
    n_near = int((np.linalg.norm(out, axis=1) < noise.min_range_m).sum())
    assert abs(n_nan - 200) <= 1
    assert abs(n_near - 100) <= 1
    near = out[np.linalg.norm(out, axis=1) < noise.min_range_m]
    np.testing.assert_allclose(np.linalg.norm(near, axis=1), 0.3, atol=1e-6)


def test_deterministic_with_seed() -> None:
    xyz = _grid(np.linspace(-15, 15, _H), 8.0)
    gt = _make_gt(1.0)
    outs = [
        process_scan(
            _cloud(xyz),
            _STAMP_NS,
            gt,
            np.eye(4),
            _NOISE,
            np.random.default_rng(7),
            range_sigma_apply=True,
        )
        for _ in range(2)
    ]
    assert outs[0].xyz.tobytes() == outs[1].xyz.tobytes()
    assert outs[0].t_ns is not None and outs[1].t_ns is not None
    assert outs[0].t_ns.tobytes() == outs[1].t_ns.tobytes()
    other = process_scan(
        _cloud(xyz),
        _STAMP_NS,
        gt,
        np.eye(4),
        _NOISE,
        np.random.default_rng(8),
        range_sigma_apply=True,
    )
    assert other.xyz.tobytes() != outs[0].xyz.tobytes()
