"""Synthetic raw bag: topics, rates, geometry of the LiDAR cloud, IMU/GNSS/joint consistency."""

from pathlib import Path

import numpy as np
import pytest

from mss_sim.bag_io import BagReader, cloud_to_arrays, stamp_ns
from mss_sim.geodesy import lla_to_enu
from mss_sim.gt import GroundTruth
from mss_sim.synth import SynthInfo, write_synthetic_raw_bag

_TOPICS = {
    "/ground_truth": "nav_msgs/msg/Odometry",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/gnss/fix": "sensor_msgs/msg/NavSatFix",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
_P_B_G = np.array([-0.20, 0.10, 0.60])
_FLOAT32_PLANE_TOL_M = 1e-4  # points are float32: ~1e-6 relative at 10 m


def _read(bag: tuple[Path, SynthInfo], topic: str) -> list[tuple[int, object]]:
    with BagReader(bag[0]) as reader:
        return [(t, m) for _, t, m in reader.messages([topic])]


def test_topics_and_counts(raw_bag: tuple[Path, SynthInfo]) -> None:
    with BagReader(raw_bag[0]) as reader:
        assert reader.topics() == _TOPICS
    assert len(_read(raw_bag, "/lidar/points")) == 60
    assert len(_read(raw_bag, "/imu/data")) == 1200
    assert len(_read(raw_bag, "/gnss/fix")) == 30
    assert len(_read(raw_bag, "/joint_states")) == 300
    assert len(_read(raw_bag, "/tf_static")) == 1


@pytest.mark.parametrize("topic", [t for t in _TOPICS if t != "/tf_static"])
def test_stamps_strictly_increase_and_match_log_time(
    raw_bag: tuple[Path, SynthInfo], topic: str
) -> None:
    msgs = _read(raw_bag, topic)
    log = np.array([t for t, _ in msgs], dtype=np.int64)
    hdr = np.array([stamp_ns(m.header) for _, m in msgs], dtype=np.int64)
    assert np.all(np.diff(log) > 0)
    np.testing.assert_array_equal(log, hdr)
    assert log[0] == raw_bag[1].t0_ns


def test_lidar_points_lie_on_planes(raw_bag: tuple[Path, SynthInfo]) -> None:
    info = raw_bag[1]
    assert len(info.planes) == 6
    gt = GroundTruth.from_odometry_msgs(m for _, m in _read(raw_bag, "/ground_truth"))
    n_finite_total = 0
    for t_ns, msg in _read(raw_bag, "/lidar/points"):
        arr = cloud_to_arrays(msg)
        assert (arr.height, arr.width) == (16, 720)
        assert arr.t_ns is None and arr.ring is None
        assert [(f.name, f.offset) for f in msg.fields] == [
            ("x", 0),
            ("y", 4),
            ("z", 8),
            ("intensity", 12),
        ]
        assert msg.point_step == 16
        finite = np.isfinite(arr.xyz).all(axis=1)
        assert np.array_equal(finite, np.isfinite(arr.xyz).any(axis=1))  # all-or-nothing
        n_finite_total += int(finite.sum())
        rot, p = gt.pose_at(np.array([t_ns], dtype=np.int64))
        pts_w = arr.xyz[finite].astype(float) @ rot[0].T + p[0]
        dist = np.min(
            [np.abs(pts_w @ n - d) for n, d in info.planes], axis=0
        )  # nearest plane per point
        assert dist.max() < _FLOAT32_PLANE_TOL_M
    assert n_finite_total > 0


def test_lidar_inside_box_all_finite_and_grid_layout(raw_bag: tuple[Path, SynthInfo]) -> None:
    _, msg = _read(raw_bag, "/lidar/points")[0]
    xyz = cloud_to_arrays(msg).xyz.reshape(16, 720, 3)
    assert np.isfinite(xyz).all()  # inside a closed box every ray hits
    assert xyz[0, 0, 2] < 0 < xyz[15, 0, 2]
    az0 = np.arctan2(xyz[:, 0, 1], xyz[:, 0, 0])
    np.testing.assert_allclose(az0, 0.0, atol=1e-6)
    az1 = np.arctan2(xyz[:, 1, 1], xyz[:, 1, 0])
    np.testing.assert_allclose(az1, np.deg2rad(0.5), atol=1e-6)


def test_imu_matches_kinematics_with_white_noise(raw_bag: tuple[Path, SynthInfo]) -> None:
    msgs = [m for _, m in _read(raw_bag, "/imu/data")]
    gyro = np.array(
        [[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in msgs]
    )
    acc = np.array(
        [[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in msgs]
    )
    omega, speed = 0.3, 0.5
    sig_g, sig_a = 1.75e-4 * np.sqrt(200.0), 2.9e-3 * np.sqrt(200.0)
    np.testing.assert_allclose(
        gyro.mean(axis=0), [0, 0, omega], atol=5 * sig_g / np.sqrt(len(gyro))
    )
    np.testing.assert_allclose(gyro.std(axis=0), sig_g, rtol=0.1)
    expect = [0.0, speed * omega, 9.80665]  # centripetal along body +y
    np.testing.assert_allclose(acc.mean(axis=0), expect, atol=5 * sig_a / np.sqrt(len(acc)))
    np.testing.assert_allclose(acc.std(axis=0), sig_a, rtol=0.1)


def test_gnss_is_antenna_position(raw_bag: tuple[Path, SynthInfo]) -> None:
    info = raw_bag[1]
    gt = GroundTruth.from_odometry_msgs(m for _, m in _read(raw_bag, "/ground_truth"))
    msgs = _read(raw_bag, "/gnss/fix")
    t = np.array([t for t, _ in msgs], dtype=np.int64)
    lla = np.array([[m.latitude, m.longitude, m.altitude] for _, m in msgs])
    assert all(
        not np.any(m.position_covariance) and m.position_covariance_type == 0 for _, m in msgs
    )
    rot, p = gt.pose_at(t)
    expect = p + rot @ _P_B_G
    np.testing.assert_allclose(lla_to_enu(lla, info.origin_lla), expect, atol=1e-6)


def test_joint_states_and_gt_circle(raw_bag: tuple[Path, SynthInfo]) -> None:
    msgs = [m for _, m in _read(raw_bag, "/joint_states")]
    assert list(msgs[0].name) == ["left_wheel_joint", "right_wheel_joint"]
    v, w, r, track = 0.5, 0.3, 0.10, 0.40
    np.testing.assert_allclose(
        msgs[5].velocity, [(v - w * track / 2) / r, (v + w * track / 2) / r], atol=1e-12
    )
    gt = GroundTruth.from_odometry_msgs(m for _, m in _read(raw_bag, "/ground_truth"))
    assert gt.t_start_ns == raw_bag[1].t0_ns
    np.testing.assert_allclose(gt.p_w_b[0], 0.0, atol=1e-12)
    radius = v / w
    centre = np.array([0.0, radius, 0.0])
    np.testing.assert_allclose(np.linalg.norm(gt.p_w_b - centre, axis=1), radius, atol=1e-9)


def test_tf_static_identity(raw_bag: tuple[Path, SynthInfo]) -> None:
    ((_, msg),) = _read(raw_bag, "/tf_static")
    (tf,) = msg.transforms
    assert (tf.header.frame_id, tf.child_frame_id) == ("imu_link", "lidar_link")
    assert (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z) == (
        0,
        0,
        0,
    )
    assert tf.transform.rotation.w == 1.0


def test_lidar_non_returns_are_inf_in_all_coordinates(tmp_path: Path) -> None:
    info = write_synthetic_raw_bag(
        tmp_path, duration_s=0.2, max_range_m=9.0, origin_lla=(21.0278, 105.8342, 12.0)
    )
    assert info.duration_s == 0.2
    with BagReader(tmp_path / "raw") as reader:
        (_, _, msg), *_ = reader.messages(["/lidar/points"])
    xyz = cloud_to_arrays(msg).xyz.reshape(16, 720, 3)
    assert not np.isnan(xyz).any()
    miss = np.isinf(xyz).any(axis=2)
    assert miss.any() and not miss.all()
    assert np.isinf(xyz[miss]).all()  # inf in x, y and z together
    assert miss[:, 0].any()  # column 0 has an exactly-zero y direction component
    assert np.linalg.norm(xyz[~miss], axis=1).max() <= 9.0 + 1e-5


# --- Static start, speed ramp and T_b_l (docs/derivations/pipeline.md section 7) ---------

_V, _W, _RADIUS_M, _TRACK_M = 0.5, 0.3, 0.10, 0.40
_STATIC_S, _RAMP_S = 2.0, 2.0
_T_B_L = ((0.10, 0.0, 0.50), (0.99984770, 0.0, 0.01745241, 0.0))  # Hamilton (w, x, y, z)


def _closed_form(t_s: np.ndarray) -> dict[str, np.ndarray]:
    """Section 7 motion for V = 0.5, W = 0.3, static 2 s, ramp 2 s; t_s relative to T0."""
    kappa = _W / _V
    tau = t_s - _STATIC_S
    ramp = (tau >= 0) & (tau < _RAMP_S)
    cruise = tau >= _RAMP_S
    x = np.pi * tau / _RAMP_S
    v = np.where(ramp, _V * (1 - np.cos(x)) / 2, 0.0) + np.where(cruise, _V, 0.0)
    s = np.where(ramp, _V / 2 * (tau - _RAMP_S / np.pi * np.sin(x)), 0.0) + np.where(
        cruise, _V * _RAMP_S / 2 + _V * (tau - _RAMP_S), 0.0
    )
    a_t = np.where(ramp, _V * np.pi / (2 * _RAMP_S) * np.sin(x), 0.0)
    psi = kappa * s
    p = np.stack([np.sin(psi), 1 - np.cos(psi), 0 * psi], axis=1) / kappa
    a_w = a_t[:, None] * np.stack([np.cos(psi), np.sin(psi), 0 * psi], axis=1) + (kappa * v**2)[
        :, None
    ] * np.stack([-np.sin(psi), np.cos(psi), 0 * psi], axis=1)
    a_w[:, 2] += 9.80665  # a_w - g_w with g_w = (0, 0, -9.80665)
    c, sn = np.cos(psi), np.sin(psi)
    acc_b = np.stack(
        [c * a_w[:, 0] + sn * a_w[:, 1], -sn * a_w[:, 0] + c * a_w[:, 1], a_w[:, 2]], 1
    )
    return {"v": v, "s": s, "psi": psi, "omega": kappa * v, "p": p, "acc_b": acc_b}


@pytest.fixture(scope="module")
def ramp_bag(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, SynthInfo]:
    out = tmp_path_factory.mktemp("ramp_bag")
    info = write_synthetic_raw_bag(
        out,
        duration_s=6.0,
        seed=0,
        origin_lla=(21.0278, 105.8342, 12.0),
        static_s=_STATIC_S,
        ramp_s=_RAMP_S,
        t_b_l=_T_B_L,
    )
    return out / "raw", info


def test_static_then_ramp_profile(ramp_bag: tuple[Path, SynthInfo]) -> None:
    gt_msgs = _read(ramp_bag, "/ground_truth")
    t_s = (np.array([t for t, _ in gt_msgs], dtype=np.int64) - ramp_bag[1].t0_ns) * 1e-9
    ref = _closed_form(t_s)
    pos = np.array(
        [
            [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
            for _, m in gt_msgs
        ]
    )
    q = np.array([[m.pose.pose.orientation.w, m.pose.pose.orientation.z] for _, m in gt_msgs])
    lin = np.array([m.twist.twist.linear.x for _, m in gt_msgs])
    ang = np.array([m.twist.twist.angular.z for _, m in gt_msgs])
    still = t_s < _STATIC_S
    assert still.sum() == 400
    assert np.all(pos[still] == 0.0)
    assert np.all(lin[still] == 0.0) and np.all(ang[still] == 0.0)
    np.testing.assert_allclose(pos, ref["p"], atol=1e-9)
    np.testing.assert_allclose(2 * np.arctan2(q[:, 1], q[:, 0]), ref["psi"], atol=1e-9)
    np.testing.assert_allclose(lin, ref["v"], atol=1e-12)
    np.testing.assert_allclose(ang, ref["omega"], atol=1e-12)

    joints = _read(ramp_bag, "/joint_states")
    tj = (np.array([t for t, _ in joints], dtype=np.int64) - ramp_bag[1].t0_ns) * 1e-9
    rj = _closed_form(tj)
    vel = np.array([m.velocity for _, m in joints])
    half = rj["omega"] * _TRACK_M / 2
    np.testing.assert_allclose(vel[:, 0], (rj["v"] - half) / _RADIUS_M, atol=1e-12)
    np.testing.assert_allclose(vel[:, 1], (rj["v"] + half) / _RADIUS_M, atol=1e-12)
    pos_j = np.array([m.position for _, m in joints])
    half_psi = rj["psi"] * _TRACK_M / 2
    np.testing.assert_allclose(pos_j[:, 0], (rj["s"] - half_psi) / _RADIUS_M, atol=1e-9)
    np.testing.assert_allclose(pos_j[:, 1], (rj["s"] + half_psi) / _RADIUS_M, atol=1e-9)


def test_static_then_ramp_imu(ramp_bag: tuple[Path, SynthInfo]) -> None:
    msgs = _read(ramp_bag, "/imu/data")
    t_s = (np.array([t for t, _ in msgs], dtype=np.int64) - ramp_bag[1].t0_ns) * 1e-9
    ref = _closed_form(t_s)
    gyro = np.array(
        [[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in msgs]
    )
    acc = np.array(
        [
            [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
            for _, m in msgs
        ]
    )
    sig_g, sig_a = 1.75e-4 * np.sqrt(200.0), 2.9e-3 * np.sqrt(200.0)
    gyro_ref = np.zeros_like(gyro)
    gyro_ref[:, 2] = ref["omega"]
    assert np.abs(gyro - gyro_ref).max() < 5 * sig_g
    assert np.abs(acc - ref["acc_b"]).max() < 5 * sig_a
    # The ramp is visible above the noise: tangential acceleration peaks at V pi / (2 ramp_s).
    assert ref["acc_b"][:, 0].max() > 10 * sig_a / np.sqrt(len(acc))


def test_t_b_l_written_and_used(ramp_bag: tuple[Path, SynthInfo]) -> None:
    ((_, msg),) = _read(ramp_bag, "/tf_static")
    (tf,) = msg.transforms
    tr, rot = tf.transform.translation, tf.transform.rotation
    assert (tf.header.frame_id, tf.child_frame_id) == ("imu_link", "lidar_link")
    assert (tr.x, tr.y, tr.z) == _T_B_L[0]
    assert (rot.w, rot.x, rot.y, rot.z) == _T_B_L[1]  # ROS fields x y z w, compared as w x y z

    w, x, y, z = _T_B_L[1]
    r_b_l = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    scans = _read(ramp_bag, "/lidar/points")
    gt = GroundTruth.from_odometry_msgs(m for _, m in _read(ramp_bag, "/ground_truth"))
    # Scan 0 is in the still phase (T_w_b = I); scan 55 (t = 5.5 s) is in cruise, where a wrong
    # T_w_l = T_w_b * T_b_l composition (order, or missing R_w_b on p_b_l) would show.
    for k in (0, 55):
        t_ns, cloud = scans[k]
        r_w_b, p_w_b = gt.pose_at(np.array([t_ns], dtype=np.int64))
        yaw = np.arctan2(r_w_b[0][1, 0], r_w_b[0][0, 0])
        if k == 55:
            assert yaw > 0.7 and np.linalg.norm(p_w_b[0]) > 1.0
        xyz = cloud_to_arrays(cloud).xyz
        finite = np.isfinite(xyz).all(axis=1)
        assert finite.sum() > 0
        p_b = xyz[finite].astype(float) @ r_b_l.T + np.array(_T_B_L[0])
        p_w = p_b @ r_w_b[0].T + p_w_b[0]
        dist = np.min([np.abs(p_w @ n - d) for n, d in ramp_bag[1].planes], axis=0)
        assert dist.max() < 1e-5, (k, dist.max())  # float32 points at up to ~11 m
