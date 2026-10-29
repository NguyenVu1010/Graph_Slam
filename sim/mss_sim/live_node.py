"""rclpy node: the online noise layer between Gazebo and mss_node (ADR-011, 2026-10-29 amendment).

    python -m mss_sim.live_node <scenario.yaml> [--seed N] [--no-skew] [--raw-prefix /sim/raw] \\
        --ros-args -p use_sim_time:=true

Subscribes the raw Gazebo sensor topics under ``--raw-prefix`` (``/sim/raw/lidar/points``,
``/sim/raw/imu/data``, ``/sim/raw/gnss/fix``, ``/sim/raw/joint_states``) plus ``/ground_truth``
and ``/tf_static``, and publishes ``/lidar/points``, ``/imu/data``, ``/gnss/fix`` and
``/wheel/odom``, the topics mss_node reads. Messages stay serialised CDR end to end (``raw=True``
subscriptions, bytes published), so the noise models are exactly the offline bridge's
(``mss_sim.live.LiveNoise``). Every ``SUMMARY_PERIOD_S`` of wall time and at shutdown it logs the
counters and the wall time spent per scan. Needs ROS 2 (rclpy); the core is unit-tested in
``tests/python/test_live.py``, the node by the live run.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage

from mss_sim.live import GNSS, GT, IMU, JOINTS, LIDAR, ODOM, TF, LiveNoise
from mss_sim.profile import load_profile
from mss_sim.scenario import load_scenario

SUMMARY_PERIOD_S = 10.0  # wall s between counter logs
# Subscription depth per raw topic (messages): >= 2 s at the sensor rate, so a scan being
# processed (tens of ms) never overflows the queue of the 1 kHz joint states.
SUB_DEPTH = {LIDAR: 10, IMU: 400, GNSS: 20, JOINTS: 2000, GT: 400}
PUB_DEPTH = {LIDAR: 5, IMU: 200, GNSS: 10, ODOM: 50}  # messages
_ROS_TYPES = {
    GT: Odometry,
    TF: TFMessage,
    IMU: Imu,
    LIDAR: PointCloud2,
    GNSS: NavSatFix,
    JOINTS: JointState,
    ODOM: Odometry,
}
_PREFIXED = (LIDAR, IMU, GNSS, JOINTS)  # raw Gazebo topics that clash with the noisy names


class LiveNoiseNode(Node):
    def __init__(self, scenario: Path, seed: int | None, skew: bool, raw_prefix: str) -> None:
        super().__init__("mss_sim_live_noise")
        sc = load_scenario(scenario)
        self._live = LiveNoise(sc, load_profile(sc.profile), seed=seed, skew=skew)
        self._scan_wall_ms: list[float] = []
        self._pubs = {t: self.create_publisher(_ROS_TYPES[t], t, d) for t, d in PUB_DEPTH.items()}
        for topic in (GT, IMU, LIDAR, GNSS, JOINTS):
            name = raw_prefix + topic if topic in _PREFIXED else topic
            qos = QoSProfile(depth=SUB_DEPTH[topic], reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(_ROS_TYPES[topic], name, self._callback(topic), qos, raw=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, TF, self._callback(TF), latched, raw=True)
        self._last_summary = time.monotonic()
        seed_used = sc.seed if seed is None else seed
        self.get_logger().info(
            f"live noise: {sc.id}, profile {sc.profile.name}, seed {seed_used}, skew {skew}, "
            f"raw topics under {raw_prefix}"
        )

    def _callback(self, topic: str):  # noqa: ANN202
        def on_msg(cdr: bytes) -> None:
            t0 = time.perf_counter()
            for out_topic, out_cdr in self._live.push(topic, cdr):
                self._pubs[out_topic].publish(out_cdr)
                if out_topic == LIDAR:
                    self._scan_wall_ms.append((time.perf_counter() - t0) * 1e3)
            if time.monotonic() - self._last_summary >= SUMMARY_PERIOD_S:
                self.log_summary()

        return on_msg

    def log_summary(self) -> None:
        self._last_summary = time.monotonic()
        c = self._live.counters
        layer = self._live.layer
        ms = np.array(self._scan_wall_ms) if self._scan_wall_ms else np.zeros(1)
        extra = (
            f"gnss outage {layer.n_dropped_gnss}, gnss rejected {layer.degrader.n_rejected}, "
            f"odom errors {dict(layer.quantizer.errors)}"
            if layer is not None
            else "not started (waiting for /ground_truth and /tf_static)"
        )
        self.get_logger().info(
            f"live noise: in {c.n_in}; out {c.n_out}; start-up overflow {c.n_overflow}; "
            f"gt not increasing {c.n_gt_not_increasing}, gt clamped {c.n_gt_clamped}; {extra}; "
            f"scan wall ms p50 {np.percentile(ms, 50):.1f} p95 {np.percentile(ms, 95):.1f} "
            f"max {ms.max():.1f}"
        )


def main(argv: list[str] | None = None) -> int:
    args_all = sys.argv if argv is None else argv
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("scenario", type=Path)
    ap.add_argument("--seed", type=int, default=None, help="override the scenario seed")
    ap.add_argument("--no-skew", action="store_true", help="do not inject motion skew")
    ap.add_argument("--raw-prefix", default="/sim/raw", help="namespace of the raw topics")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(args_all)[1:])
    rclpy.init(args=args_all)
    node = LiveNoiseNode(args.scenario, args.seed, not args.no_skew, args.raw_prefix)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.log_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
