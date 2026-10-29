"""Online noise layer: the offline bridge's ``NoiseLayer`` fed message by message (no ROS here).

``LiveNoise.push(topic, cdr)`` takes one raw message as serialised CDR bytes (what an rclpy
``raw=True`` subscription delivers and a bag stores) and returns the noisy messages to publish
as ``(topic, cdr)``. ``mss_sim.live_node`` wraps it in rclpy (ADR-011, 2026-10-29 amendment).

What differs from the offline bridge, and why the output still matches it:
- The ground truth grows: a scan is held until ``/ground_truth`` covers
  ``[stamp, stamp + scan period]`` (skew needs the motion during the sweep), then processed
  against a window of the last ``GT_WINDOW_S`` of samples. Interpolation is per interval
  (linear position, slerp keyed by integer ns), so the window gives the offline numbers.
- ``t0`` (GNSS segment times) is the first ground-truth stamp and ``T_b_l`` / the body frame
  come from ``/tf_static``; GNSS and joint states that arrive before them wait in a queue.
- Every sensor has its own rng stream, so only each sensor's own order matters. Fed the raw
  bag's messages in log-time order, the per-topic output equals the offline bag's bytes
  (``tests/python/test_live.py``). Live, Gazebo is not deterministic: only the seed is fixed.
- ``/ground_truth`` and ``/tf_static`` are not republished (they are already on the bus) and
  log times do not exist (the publisher stamps nothing; header stamps carry the time).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from mss_sim.bag_io import TYPESTORE, stamp_ns
from mss_sim.bridge import NoiseLayer, shift_stamp, t_b_l_from_tf
from mss_sim.gt import GroundTruth
from mss_sim.profile import NoiseProfile
from mss_sim.scenario import Scenario

GT, TF, IMU = "/ground_truth", "/tf_static", "/imu/data"
LIDAR, GNSS, JOINTS, ODOM = "/lidar/points", "/gnss/fix", "/joint_states", "/wheel/odom"
IN_TYPES = {
    GT: "nav_msgs/msg/Odometry",
    TF: "tf2_msgs/msg/TFMessage",
    IMU: "sensor_msgs/msg/Imu",
    LIDAR: "sensor_msgs/msg/PointCloud2",
    GNSS: "sensor_msgs/msg/NavSatFix",
    JOINTS: "sensor_msgs/msg/JointState",
}
OUT_TYPES = {
    IMU: "sensor_msgs/msg/Imu",
    LIDAR: "sensor_msgs/msg/PointCloud2",
    GNSS: "sensor_msgs/msg/NavSatFix",
    ODOM: "nav_msgs/msg/Odometry",
}
GT_WINDOW_S = 2.0  # s of ground truth kept for skew; scans older than this clamp (counted)
MAX_PENDING = {LIDAR: 20, GNSS: 50, JOINTS: 5000}  # messages held before start-up data, each
_NS_PER_S = 1_000_000_000


@dataclass
class LiveCounters:
    n_in: dict[str, int] = field(default_factory=lambda: dict.fromkeys(IN_TYPES, 0))
    n_out: dict[str, int] = field(default_factory=lambda: dict.fromkeys(OUT_TYPES, 0))
    n_overflow: dict[str, int] = field(default_factory=lambda: dict.fromkeys(MAX_PENDING, 0))
    n_gt_not_increasing: int = 0
    n_gt_clamped: int = 0


class LiveNoise:
    """Raw CDR in, noisy CDR out; one instance per live run."""

    def __init__(
        self,
        scenario: Scenario,
        profile: NoiseProfile,
        *,
        seed: int | None = None,
        skew: bool = True,
    ) -> None:
        self._scenario = scenario
        self._profile = profile
        self._seed = seed
        self._skew = skew
        self._layer: NoiseLayer | None = None
        self._t0_ns: int | None = None
        self._tf: tuple[np.ndarray, str] | None = None
        self._scan_ns = int(round(profile.lidar.scan_period_s * _NS_PER_S))
        self._window_ns = int(round(GT_WINDOW_S * _NS_PER_S))
        self._gt_t: deque[int] = deque()
        self._gt_p: deque[tuple[float, float, float]] = deque()
        self._gt_q: deque[tuple[float, float, float, float]] = deque()
        self._pending: dict[str, deque[Any]] = {k: deque() for k in MAX_PENDING}
        self.counters = LiveCounters()

    @property
    def started(self) -> bool:
        return self._layer is not None

    @property
    def layer(self) -> NoiseLayer | None:
        return self._layer

    def push(self, topic: str, cdr: bytes) -> list[tuple[str, bytes]]:
        """Process one raw message; return the noisy messages it releases, in order."""
        self.counters.n_in[topic] += 1
        msg = TYPESTORE.deserialize_cdr(cdr, IN_TYPES[topic])
        out: list[tuple[str, Any]] = []
        if topic == IMU:
            # IMU needs neither ground truth nor geometry; before start the offsets still apply.
            out.append((IMU, shift_stamp(msg, self._profile.clock_offset_ns["imu"])))
        elif topic == GT:
            self._add_gt(msg)
        elif topic == TF:
            if self._tf is None:
                try:
                    self._tf = t_b_l_from_tf([msg])
                except ValueError:
                    pass  # another static transform without lidar_link: keep waiting
        else:
            self._hold(topic, msg)
        self._maybe_start()
        if self._layer is not None:
            out.extend(self._release())
        for t, _ in out:
            self.counters.n_out[t] += 1
        return [(t, bytes(TYPESTORE.serialize_cdr(m, OUT_TYPES[t]))) for t, m in out]

    def _add_gt(self, msg: Any) -> None:
        t = stamp_ns(msg.header)
        if self._gt_t and t <= self._gt_t[-1]:
            self.counters.n_gt_not_increasing += 1
            return
        if self._t0_ns is None:
            self._t0_ns = t
        pose = msg.pose.pose
        o = pose.orientation
        self._gt_t.append(t)
        self._gt_p.append((pose.position.x, pose.position.y, pose.position.z))
        self._gt_q.append((o.w, o.x, o.y, o.z))  # Hamilton order (w, x, y, z)
        while len(self._gt_t) > 2 and self._gt_t[1] < t - self._window_ns:
            self._gt_t.popleft()
            self._gt_p.popleft()
            self._gt_q.popleft()

    def _hold(self, topic: str, msg: Any) -> None:
        queue = self._pending[topic]
        if len(queue) >= MAX_PENDING[topic]:
            queue.popleft()
            self.counters.n_overflow[topic] += 1
        queue.append(msg)

    def _maybe_start(self) -> None:
        if self._layer is None and self._t0_ns is not None and self._tf is not None:
            t_b_l, body = self._tf
            self._layer = NoiseLayer(
                self._scenario,
                self._profile,
                self._t0_ns,
                t_b_l,
                body,
                seed=self._seed,
                skew=self._skew,
            )

    def _release(self) -> list[tuple[str, Any]]:
        layer = self._layer
        assert layer is not None
        out: list[tuple[str, Any]] = []
        for msg in _drain(self._pending[GNSS]):
            fix = layer.gnss(msg)
            if fix is not None:
                out.append((GNSS, fix))
        for msg in _drain(self._pending[JOINTS]):
            odom = layer.joints(msg)
            if odom is not None:
                out.append((ODOM, odom))
        scans = self._pending[LIDAR]
        if scans and len(self._gt_t) >= 2:
            gt: GroundTruth | None = None
            while scans and stamp_ns(scans[0].header) + self._scan_ns <= self._gt_t[-1]:
                if gt is None:
                    gt = GroundTruth(
                        np.array(self._gt_t), np.array(self._gt_p), np.array(self._gt_q)
                    )
                out.append((LIDAR, layer.lidar(scans.popleft(), gt)))
            if gt is not None:
                self.counters.n_gt_clamped += gt.n_clamped
        return out


def _drain(queue: deque[Any]) -> list[Any]:
    items = list(queue)
    queue.clear()
    return items
