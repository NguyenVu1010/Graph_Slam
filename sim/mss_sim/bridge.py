"""Offline noise bridge: raw (clean) bag -> noisy bag, deterministic for a given seed.

The per-message models live in ``NoiseLayer``, which the live node (``mss_sim.live``) reuses.
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


def t_b_l_from_tf(tf_msgs: Iterable[Any]) -> tuple[np.ndarray, str]:
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


def shift_stamp(msg: Any, offset_ns: int) -> Any:
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


class NoiseLayer:
    """The per-message noise models, shared by the offline bridge and the live node.

    One instance per run. Each method takes one raw message (``rosbags`` dataclass) and returns
    the noisy message, or ``None`` when nothing is emitted. Every sensor has its own rng stream,
    so the output depends only on each sensor's own input order, not on how the sensors
    interleave. ``t0_ns`` (first ground-truth stamp) anchors the GNSS segment times;
    ``body_frame`` (parent of ``lidar_link`` in ``/tf_static``) is the odometry child frame.
    """

    def __init__(
        self,
        scenario: Scenario,
        profile: NoiseProfile,
        t0_ns: int,
        T_b_l: np.ndarray,  # noqa: N803
        body_frame: str,
        *,
        seed: int | None = None,
        skew: bool = True,
    ) -> None:
        seeds = np.random.SeedSequence(scenario.seed if seed is None else seed).spawn(5)
        rng_lidar, rng_gnss, rng_odom, _rng_imu, _rng_mutate = (
            np.random.default_rng(s) for s in seeds
        )
        self._scenario = scenario
        self._profile = profile
        self._rng_lidar = rng_lidar
        self._t_b_l = T_b_l
        self._body_frame = body_frame
        self._skew = skew
        self.offset_ns = profile.clock_offset_ns
        self.degrader = GnssDegrader(
            profile.gnss, scenario.gnss, scenario.gnss_zones, scenario.origin_lla, rng_gnss, t0_ns
        )
        self.quantizer = WheelQuantizer(profile.odom, rng_odom)
        self._pose = _OdomIntegrator()
        self.n_dropped_gnss = 0

    def imu(self, msg: Any) -> Any:
        return shift_stamp(msg, self.offset_ns["imu"])

    def lidar(self, msg: Any, gt: GroundTruth) -> Any:
        """Noisy cloud; ``gt`` must cover ``[stamp, stamp + scan period]`` (else it clamps)."""
        stamp = stamp_ns(msg.header)
        scan = process_scan(
            cloud_to_arrays(msg),
            stamp,
            gt,
            self._t_b_l,
            self._profile.lidar,
            self._rng_lidar,
            skew=self._skew,
        )
        return arrays_to_cloud(scan, stamp + self.offset_ns["lidar"], _LIDAR_FRAME)

    def gnss(self, msg: Any) -> Any | None:
        stamp = stamp_ns(msg.header)
        lla = (msg.latitude, msg.longitude, msg.altitude)
        n_rejected = self.degrader.n_rejected
        sample = self.degrader.step(stamp, lla_to_enu(lla, self._scenario.origin_lla))
        if sample is None:
            if self.degrader.n_rejected == n_rejected:
                self.n_dropped_gnss += 1  # declared outage; rejected stamps counted apart
            return None
        return _navsatfix(stamp + self.offset_ns["gnss"], sample)

    def joints(self, msg: Any) -> Any | None:
        stamp = stamp_ns(msg.header)
        tw = self.quantizer.step(stamp, list(msg.name), [float(v) for v in msg.velocity])
        if tw is None:
            return None
        self._pose.step(stamp, tw)
        return _odometry(stamp + self.offset_ns["odom"], self._pose, tw, self._body_frame)


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
    off = profile.clock_offset_ns
    out_dir.mkdir(parents=True, exist_ok=True)

    with BagReader(raw_bag) as reader:
        gt = GroundTruth.from_odometry_msgs(m for _, _, m in reader.messages([_GT]))
        tf_topic = [_TF] if _TF in reader.topics() else []
        t_b_l, body_frame = t_b_l_from_tf(m for _, _, m in reader.messages(tf_topic))
        layer = NoiseLayer(
            scenario, profile, gt.t_start_ns, t_b_l, body_frame, seed=seed, skew=skew
        )
        counts = dict.fromkeys(_OUT_TYPES, 0)

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
                    emit(topic, log_time + off["imu"], layer.imu(msg))
                elif topic == _LIDAR:
                    emit(topic, log_time + off["lidar"], layer.lidar(msg, gt))
                elif topic == _GNSS:
                    fix = layer.gnss(msg)
                    if fix is not None:
                        emit(topic, log_time + off["gnss"], fix)
                else:  # _JOINTS -> _ODOM
                    odom = layer.joints(msg)
                    if odom is not None:
                        emit(_ODOM, log_time + off["odom"], odom)

    gt.to_tum(out_dir / "gt.tum")
    report = BridgeReport(
        counts,
        layer.n_dropped_gnss,
        layer.quantizer.n_errors,
        gt.n_clamped,
        layer.degrader.n_rejected,
        dict(layer.quantizer.errors),
    )
    (out_dir / "bridge_report.json").write_text(json.dumps(asdict(report), indent=2))
    return report
