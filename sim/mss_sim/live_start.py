"""Start and stop a live run: Gazebo starts paused; this node unpauses it once every consumer
listens, at a fixed real-time factor, and pauses it again at the end (ADR-011 amendment).

    python -m mss_sim.live_start <world> --rtf 1.0 --stop-at-s 62 \\
        --wait-subs /lidar/points /imu/data ... --ros-args -p use_sim_time:=true

1. Wait (wall clock) until each ``--wait-subs`` topic has a subscriber (mss_node, the live noise
   node), so nothing published after the unpause is lost.
2. Set the real-time factor (``/world/<world>/set_physics``, step size kept at ``--step-s``).
3. Step the paused world 1 ms at a time until ``/ground_truth`` arrives (the robot exists).
   The sim clock must then be within ``--max-start-clock-s`` of 0, else it is not this run's
   Gazebo (a left-over or foreign server on the same domain): exit 2.
4. Unpause; at ``--stop-at-s`` of sim time, pause the world, log the measured real-time factor
   (sim s / wall s) and exit 0. Exit 2 when a wait times out, or when the sim clock does not
   advance for ``--stall-timeout-s`` of wall time or goes back.

The sequence is ``mss_sim.live_guard.run_live`` (no ROS, unit-tested with a fake node); this
module is its rclpy node. World services go through the ``gz service`` CLI, as
``tools/record_scenario.sh`` does.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from mss_sim.live_guard import run_live

SERVICE_TIMEOUT_MS = 5000


def _world_call(world: str, service: str, reqtype: str, req: str) -> bool:
    cmd = [
        "gz", "service", "-s", f"/world/{world}/{service}", "--reqtype", reqtype,
        "--reptype", "gz.msgs.Boolean", "--timeout", str(SERVICE_TIMEOUT_MS), "--req", req,
    ]  # fmt: skip
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return proc.returncode == 0 and "true" in proc.stdout


class LiveStart(Node):
    """The rclpy side of ``live_guard.LiveNode``."""

    def __init__(self) -> None:
        super().__init__("mss_sim_live_start")
        self.n_gt = 0
        self.create_subscription(Odometry, "/ground_truth", self._on_gt, 10)

    def _on_gt(self, _msg: Odometry) -> None:
        self.n_gt += 1

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, end - time.monotonic()))

    def sim_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9  # seconds only for checks and the log

    def info(self, msg: str) -> None:
        self.get_logger().info(msg)

    def error(self, msg: str) -> None:
        self.get_logger().error(msg)


def main(argv: list[str] | None = None) -> int:
    args_all = sys.argv if argv is None else argv
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("world")
    ap.add_argument("--rtf", type=float, required=True, help="real-time factor (> 0)")
    ap.add_argument("--step-s", type=float, default=0.001, help="physics step, s (the world's)")
    ap.add_argument("--stop-at-s", type=float, required=True, help="sim time to pause at, s")
    ap.add_argument("--wait-subs", nargs="*", default=[], help="topics that need a subscriber")
    ap.add_argument(
        "--wait-timeout-s",
        type=float,
        default=120.0,
        help="wall s for the subscribers to appear and the robot to spawn",
    )
    ap.add_argument(
        "--max-start-clock-s",
        type=float,
        default=1.0,
        help="largest sim time accepted when the robot first appears, s (else not our Gazebo)",
    )
    ap.add_argument(
        "--stall-timeout-s",
        type=float,
        default=10.0,
        help="wall s without sim-clock progress before the run fails",
    )
    args = ap.parse_args(rclpy.utilities.remove_ros_args(args_all)[1:])
    for name in ("rtf", "wait_timeout_s", "max_start_clock_s", "stall_timeout_s"):
        if not getattr(args, name) > 0:
            ap.error(f"--{name.replace('_', '-')} must be > 0")
    rclpy.init(args=args_all)
    node = LiveStart()

    def world_call(service: str, reqtype: str, req: str) -> bool:
        return _world_call(args.world, service, reqtype, req)

    try:
        return run_live(args, node, world_call, time.monotonic)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
