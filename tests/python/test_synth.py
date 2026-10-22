"""Synthetic raw bag: topics, rates, geometry of the LiDAR cloud, IMU/GNSS/joint consistency."""

from pathlib import Path

import numpy as np
import pytest

from mss_sim.bag_io import BagReader, cloud_to_arrays, stamp_ns
from mss_sim.geodesy import lla_to_enu
from mss_sim.gt import GroundTruth
from mss_sim.synth import SynthInfo

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


def test_lidar_misses_are_inf_and_ring_zero_is_lowest(raw_bag: tuple[Path, SynthInfo]) -> None:
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
    assert (tf.header.frame_id, tf.child_frame_id) == ("base_link", "lidar_link")
    assert (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z) == (
        0,
        0,
        0,
    )
    assert tf.transform.rotation.w == 1.0
