"""Ground-truth container: Odometry ingestion, interpolation, clamping, TUM export."""

import logging
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mss_eval.tum import read_tum
from mss_sim.bag_io import TYPESTORE, make_header
from mss_sim.gt import GroundTruth

_OMEGA = 0.5  # rad/s about z
_RATE_HZ = 200
_DT_NS = 5_000_000
_N = 2 * _RATE_HZ + 1
_VEL = np.array([1.0, -0.5, 0.25])  # m/s


def _odom(t_ns: int) -> object:
    t_s = t_ns * 1e-9
    qx, qy, qz, qw = Rotation.from_rotvec([0.0, 0.0, _OMEGA * t_s]).as_quat()  # x y z w
    p = _VEL * t_s
    ty = TYPESTORE.types
    point = ty["geometry_msgs/msg/Point"](x=p[0], y=p[1], z=p[2])
    quat = ty["geometry_msgs/msg/Quaternion"](x=qx, y=qy, z=qz, w=qw)
    pose = ty["geometry_msgs/msg/Pose"](position=point, orientation=quat)
    cov = np.zeros(36)
    pwc = ty["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=cov)
    v3 = ty["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
    twist = ty["geometry_msgs/msg/Twist"](linear=v3, angular=v3)
    twc = ty["geometry_msgs/msg/TwistWithCovariance"](twist=twist, covariance=cov)
    return ty["nav_msgs/msg/Odometry"](
        header=make_header(t_ns, "map"), child_frame_id="base", pose=pwc, twist=twc
    )


@pytest.fixture
def gt() -> GroundTruth:
    return GroundTruth.from_odometry_msgs([_odom(i * _DT_NS) for i in range(_N)])


def test_interpolation_between_samples(gt: GroundTruth) -> None:
    t_ns = np.array([2_500_000, 1_003_000_000, 1_997_500_000], dtype=np.int64)
    rot, p = gt.pose_at(t_ns)
    assert rot.shape == (3, 3, 3)
    assert p.shape == (3, 3)
    yaw = Rotation.from_matrix(rot).as_rotvec()[:, 2]
    np.testing.assert_allclose(yaw, _OMEGA * t_ns * 1e-9, atol=1e-9)
    np.testing.assert_allclose(p, _VEL[None, :] * (t_ns * 1e-9)[:, None], atol=1e-9)
    assert gt.n_clamped == 0


def test_clamp_before_and_after(gt: GroundTruth, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING, logger="mss_sim"):
        rot, p = gt.pose_at(np.array([-1_000_000_000], dtype=np.int64))
        assert gt.n_clamped == 1
        np.testing.assert_allclose(p[0], gt.p_w_b[0])
        np.testing.assert_allclose(rot[0], np.eye(3), atol=1e-12)
        end = gt.t_end_ns + 5
        rot2, p2 = gt.pose_at(np.array([end, end + 1, gt.t_start_ns], dtype=np.int64))
    assert gt.n_clamped == 3
    np.testing.assert_allclose(p2[0], gt.p_w_b[-1])
    np.testing.assert_allclose(p2[2], gt.p_w_b[0])
    np.testing.assert_allclose(rot2[1], rot2[0])
    assert len([r for r in caplog.records if r.name == "mss_sim"]) == 1


def test_bounds(gt: GroundTruth) -> None:
    assert gt.t_start_ns == 0
    assert gt.t_end_ns == (_N - 1) * _DT_NS


def test_tum_round_trip(gt: GroundTruth, tmp_path: Path) -> None:
    path = tmp_path / "gt.tum"
    gt.to_tum(path)
    traj = read_tum(path)
    assert len(traj) == _N
    np.testing.assert_allclose(traj.t_s, gt.t_ns * 1e-9, atol=1e-9)
    np.testing.assert_allclose(traj.p, gt.p_w_b, atol=1e-9)
    # File order is x y z w; memory order is w x y z.
    np.testing.assert_allclose(traj.q_xyzw[:, :3], gt.q_wxyz[:, 1:], atol=1e-9)
    np.testing.assert_allclose(traj.q_xyzw[:, 3], gt.q_wxyz[:, 0], atol=1e-9)


def test_rejects_bad_input() -> None:
    with pytest.raises(ValueError, match="at least 2"):
        GroundTruth.from_odometry_msgs([_odom(0)])
    with pytest.raises(ValueError, match="strictly increasing"):
        GroundTruth.from_odometry_msgs([_odom(0), _odom(5), _odom(5)])
