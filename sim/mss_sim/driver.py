"""rclpy node that drives the simulated robot along the scenario route (ADR-011 section 5).

    python -m mss_sim.driver <scenario.yaml> --ros-args -p use_sim_time:=true

Subscribes ``/ground_truth`` (nav_msgs/Odometry, body pose in world), publishes ``/cmd_vel``
(geometry_msgs/Twist) at ``CMD_RATE_HZ`` on sim time from ``mss_sim.pursuit.PurePursuit`` and
exits once the route is done and zero has been sent for ``EXIT_AFTER_DONE_S``. Limits come from
``sim/models/robot.yaml``. Needs ROS 2 (rclpy); not unit-tested, the launch run covers it.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mss_sim.pursuit import Limits, PurePursuit
from mss_sim.scenario import load_scenario

CMD_RATE_HZ = 50.0
EXIT_AFTER_DONE_S = 1.0  # keep sending zero this long (sim time) after the route is done
_REPO = Path(__file__).resolve().parents[2]
_ROBOT_YAML = _REPO / "sim" / "models" / "robot.yaml"


def _yaw(q: object) -> float:
    """Yaw of a geometry_msgs Quaternion (fields x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w  # type: ignore[attr-defined]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Driver(Node):
    def __init__(self, scenario: Path) -> None:
        super().__init__("mss_sim_driver")
        sc = load_scenario(scenario)
        if sc.route is None:  # pragma: no cover - load_scenario always loads it
            raise ValueError(f"{scenario}: no route")
        robot = yaml.safe_load(_ROBOT_YAML.read_text(encoding="utf-8"))
        self._pursuit = PurePursuit(sc.route, Limits.from_robot(robot), sc.seed)
        self._pose: tuple[float, float, float] | None = None
        self._done_at_s: float | None = None
        self.finished = False
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/ground_truth", self._on_gt, 50)
        self.create_timer(1.0 / CMD_RATE_HZ, self._on_timer)
        self.get_logger().info(
            f"driver: {sc.id} route {sc.trajectory.name}, {self._pursuit.length_m:.1f} m, "
            f"seed {sc.seed}"
        )

    def _on_gt(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_timer(self) -> None:
        if self._pose is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        t_s = now_ns / 1e9  # seconds only for the controller's dt and still window
        v, w = self._pursuit.command(t_s, *self._pose)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self._pub.publish(cmd)
        if self._pursuit.done:
            if self._done_at_s is None:
                self._done_at_s = t_s
                self.get_logger().info(f"driver: route done at sim t = {t_s:.2f} s")
            elif t_s - self._done_at_s >= EXIT_AFTER_DONE_S:
                self.finished = True


def main(argv: list[str] | None = None) -> int:
    args_ros = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("scenario", type=Path)
    args = ap.parse_args(args_ros[1:])
    rclpy.init(args=sys.argv if argv is None else argv)
    node = Driver(args.scenario)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
