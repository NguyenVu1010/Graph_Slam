"""Offline noise bridge: raw (clean) bag -> noisy bag, deterministic for a given seed.

Geometry (``T_b_l``, antenna position) comes from the raw bag, not from the pipeline config.
Seeds: ``SeedSequence(seed).spawn(5)`` in the order [lidar, gnss, odom, imu, mutate]; the imu and
mutate streams are reserved and unused here.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from mss_sim.bag_io import (
    TYPESTORE,
    BagReader,
    BagWriter,
    arrays_to_cloud,
    cloud_to_arrays,
    make_header,
    stamp_ns,
)
from mss_sim.geodesy import lla_to_enu
from mss_sim.gnss import GnssDegrader, GnssSample
from mss_sim.gt import GroundTruth
from mss_sim.lidar import process_scan
from mss_sim.odom import OdomTwist, WheelQuantizer
from mss_sim.profile import NoiseProfile
from mss_sim.scenario import Scenario

_GT, _TF, _IMU = "/ground_truth", "/tf_static", "/imu/data"
_LIDAR, _GNSS, _JOINTS, _ODOM = "/lidar/points", "/gnss/fix", "/joint_states", "/wheel/odom"
_IN_TOPICS = [_GT, _TF, _IMU, _LIDAR, _GNSS, _JOINTS]
_OUT_TYPES = {
    _LIDAR: "sensor_msgs/msg/PointCloud2",
    _IMU: "sensor_msgs/msg/Imu",
    _GNSS: "sensor_msgs/msg/NavSatFix",
    _ODOM: "nav_msgs/msg/Odometry",
    _GT: "nav_msgs/msg/Odometry",
    _TF: "tf2_msgs/msg/TFMessage",
}
_LIDAR_FRAME, _ODOM_FRAME = "lidar_link", "odom"
_COV_DIAGONAL_KNOWN = 2  # NavSatFix COVARIANCE_TYPE_DIAGONAL_KNOWN
_SERVICE_GPS = 1  # NavSatStatus SERVICE_GPS
_COV6_SIZE = 36


@dataclass(frozen=True)
class BridgeReport:
    n_msgs: dict[str, int]
    n_dropped_gnss: int
    n_odom_errors: int
    n_gt_clamped: int
    n_gnss_rejected: int
    # n_odom_errors by reason (WheelQuantizer.errors): missing_joint, non_finite,
    # stamp_not_increasing.
    odom_errors: dict[str, int] = field(default_factory=dict)


def _t_b_l_from_tf(tf_msgs: Iterable[Any]) -> tuple[np.ndarray, str]:
    """4x4 ``T_b_l`` and the body frame name from the first ``* -> lidar_link`` transform.

    The body frame is that transform's parent (``header.frame_id``); nothing is hard-coded.
    """
    for tf_msg in tf_msgs:
        for ts in tf_msg.transforms:
            if ts.child_frame_id == _LIDAR_FRAME:
                tr, q = ts.transform.translation, ts.transform.rotation
                t = np.eye(4)
                t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # scipy: x y z w
                t[:3, 3] = (tr.x, tr.y, tr.z)
                return t, ts.header.frame_id
    msg = f"no /tf_static transform with child {_LIDAR_FRAME}"
    raise ValueError(msg)


def _shift_stamp(msg: Any, offset_ns: int) -> Any:
    """Copy of a message whose ``header.stamp`` is shifted by ``offset_ns``."""
    header = make_header(stamp_ns(msg.header) + offset_ns, msg.header.frame_id)
    return replace(msg, header=header)


def _navsatfix(stamp: int, sample: GnssSample) -> Any:
    ty = TYPESTORE.types
    lat, lon, alt = sample.lla
    return ty["sensor_msgs/msg/NavSatFix"](
        header=make_header(stamp, "gnss_link"),
        status=ty["sensor_msgs/msg/NavSatStatus"](status=sample.status, service=_SERVICE_GPS),
        latitude=float(lat),
        longitude=float(lon),
        altitude=float(alt),
        position_covariance=np.diag(sample.cov_enu_diag).reshape(9).astype(np.float64),
        position_covariance_type=_COV_DIAGONAL_KNOWN,
    )


class _OdomIntegrator:
    """Running ``[x, y, yaw]`` with the same midpoint rule as ``odom.integrate_pose``."""

    def __init__(self) -> None:
        self._last_ns: int | None = None
        self.x = self.y = self.yaw = 0.0

    def step(self, t_ns: int, tw: OdomTwist) -> None:
        if self._last_ns is not None:
            dt = (t_ns - self._last_ns) / 1e9
            yaw_mid = self.yaw + tw.omega_radps * dt / 2.0
            self.x += tw.v_mps * dt * math.cos(yaw_mid)
            self.y += tw.v_mps * dt * math.sin(yaw_mid)
            self.yaw += tw.omega_radps * dt
        self._last_ns = t_ns


def _odometry(stamp: int, pose: _OdomIntegrator, tw: OdomTwist, body_frame: str) -> Any:
    ty = TYPESTORE.types
    half = pose.yaw / 2.0
    # Quaternion field order x, y, z, w (yaw-only rotation about z).
    quat = ty["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
    position = ty["geometry_msgs/msg/Point"](x=pose.x, y=pose.y, z=0.0)
    vec = ty["geometry_msgs/msg/Vector3"]
    twist = ty["geometry_msgs/msg/Twist"](
        linear=vec(x=tw.v_mps, y=0.0, z=0.0), angular=vec(x=0.0, y=0.0, z=tw.omega_radps)
    )
    return ty["nav_msgs/msg/Odometry"](
        header=make_header(stamp, _ODOM_FRAME),
        child_frame_id=body_frame,
        pose=ty["geometry_msgs/msg/PoseWithCovariance"](
            pose=ty["geometry_msgs/msg/Pose"](position=position, orientation=quat),
            covariance=np.zeros(_COV6_SIZE),
        ),
        twist=ty["geometry_msgs/msg/TwistWithCovariance"](
            twist=twist, covariance=np.zeros(_COV6_SIZE)
        ),
    )


def run_bridge(
    raw_bag: Path,
    scenario: Scenario,
    profile: NoiseProfile,
    out_dir: Path,
    *,
    seed: int | None = None,
    skew: bool = True,
) -> BridgeReport:
    """Turn ``raw_bag`` into ``out_dir/bag`` (+ ``gt.tum``, ``bridge_report.json``)."""
    seeds = np.random.SeedSequence(scenario.seed if seed is None else seed).spawn(5)
    rng_lidar, rng_gnss, rng_odom, _rng_imu, _rng_mutate = (np.random.default_rng(s) for s in seeds)
    off = profile.clock_offset_ns
    out_dir.mkdir(parents=True, exist_ok=True)

    with BagReader(raw_bag) as reader:
        gt = GroundTruth.from_odometry_msgs(m for _, _, m in reader.messages([_GT]))
        tf_topic = [_TF] if _TF in reader.topics() else []
        t_b_l, body_frame = _t_b_l_from_tf(m for _, _, m in reader.messages(tf_topic))
        degrader = GnssDegrader(
            profile.gnss,
            scenario.gnss,
            scenario.gnss_zones,
            scenario.origin_lla,
            rng_gnss,
            gt.t_start_ns,
        )
        quantizer = WheelQuantizer(profile.odom, rng_odom)
        pose = _OdomIntegrator()
        counts = dict.fromkeys(_OUT_TYPES, 0)
        n_dropped_gnss = 0

        with BagWriter(out_dir / "bag") as writer:
            for topic, typename in _OUT_TYPES.items():
                writer.add_topic(topic, typename)

            def emit(topic: str, log_time: int, msg: Any) -> None:
                writer.write(topic, log_time, msg)
                counts[topic] += 1

            for topic, log_time, msg in reader.messages(_IN_TOPICS):
                if topic in (_GT, _TF):
                    emit(topic, log_time, msg)
                elif topic == _IMU:
                    emit(topic, log_time + off["imu"], _shift_stamp(msg, off["imu"]))
                elif topic == _LIDAR:
                    stamp = stamp_ns(msg.header)
                    scan = process_scan(
                        cloud_to_arrays(msg), stamp, gt, t_b_l, profile.lidar, rng_lidar, skew=skew
                    )
                    cloud = arrays_to_cloud(scan, stamp + off["lidar"], _LIDAR_FRAME)
                    emit(topic, log_time + off["lidar"], cloud)
                elif topic == _GNSS:
                    stamp = stamp_ns(msg.header)
                    lla = (msg.latitude, msg.longitude, msg.altitude)
                    n_rejected = degrader.n_rejected
                    sample = degrader.step(stamp, lla_to_enu(lla, scenario.origin_lla))
                    if sample is None and degrader.n_rejected == n_rejected:
                        n_dropped_gnss += 1  # declared outage; rejected stamps counted apart
                    elif sample is not None:
                        fix = _navsatfix(stamp + off["gnss"], sample)
                        emit(topic, log_time + off["gnss"], fix)
                else:  # _JOINTS -> _ODOM
                    stamp = stamp_ns(msg.header)
                    tw = quantizer.step(stamp, list(msg.name), [float(v) for v in msg.velocity])
                    if tw is not None:
                        pose.step(stamp, tw)
                        odom = _odometry(stamp + off["odom"], pose, tw, body_frame)
                        emit(_ODOM, log_time + off["odom"], odom)

    gt.to_tum(out_dir / "gt.tum")
    report = BridgeReport(
        counts,
        n_dropped_gnss,
        quantizer.n_errors,
        gt.n_clamped,
        degrader.n_rejected,
        dict(quantizer.errors),
    )
    (out_dir / "bridge_report.json").write_text(json.dumps(asdict(report), indent=2))
    return report
