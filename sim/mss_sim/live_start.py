"""Start and stop a live run: Gazebo starts paused; this node unpauses it once every consumer
listens, at a fixed real-time factor, and pauses it again at the end (ADR-011 amendment).

    python -m mss_sim.live_start <world> --rtf 1.0 --stop-at-s 62 \\
        --wait-subs /lidar/points /imu/data ... --ros-args -p use_sim_time:=true

1. Wait (wall clock) until each ``--wait-subs`` topic has a subscriber (mss_node, the live noise
   node), so nothing published after the unpause is lost.
2. Set the real-time factor (``/world/<world>/set_physics``, step size kept at ``--step-s``).
3. Step the paused world 1 ms at a time until ``/ground_truth`` arrives (the robot exists).
4. Unpause; at ``--stop-at-s`` of sim time, pause the world, log the measured real-time factor
   (sim s / wall s) and exit 0. Exit 2 when a step times out.

The world services are called with the ``gz service`` CLI, as ``tools/record_scenario.sh`` does.
Needs ROS 2 (rclpy) and Gazebo; covered by the live run, not unit-tested.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

WAIT_TIMEOUT_S = 120.0  # wall s for the subscribers to appear and the robot to spawn
SERVICE_TIMEOUT_MS = 5000
POLL_S = 0.1  # wall s between checks


def _world_call(world: str, service: str, reqtype: str, req: str) -> bool:
    cmd = [
        "gz", "service", "-s", f"/world/{world}/{service}", "--reqtype", reqtype,
        "--reptype", "gz.msgs.Boolean", "--timeout", str(SERVICE_TIMEOUT_MS), "--req", req,
    ]  # fmt: skip
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return proc.returncode == 0 and "true" in proc.stdout


class LiveStart(Node):
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
        return self.get_clock().now().nanoseconds / 1e9  # seconds only for the log and stop


def run(args: argparse.Namespace, node: LiveStart) -> int:
    log = node.get_logger()
    deadline = time.monotonic() + WAIT_TIMEOUT_S
    while missing := [t for t in args.wait_subs if node.count_subscribers(t) == 0]:
        if time.monotonic() > deadline:
            log.error(f"live_start: no subscriber on {missing} after {WAIT_TIMEOUT_S} s")
            return 2
        node.spin_for(POLL_S)
    physics = f"max_step_size: {args.step_s}, real_time_factor: {args.rtf}"
    if not _world_call(args.world, "set_physics", "gz.msgs.Physics", physics):
        log.error(f"live_start: set_physics ({physics}) failed")
        return 2
    n_steps = 0
    while node.n_gt == 0:
        if time.monotonic() > deadline:
            log.error("live_start: no /ground_truth while stepping the paused world")
            return 2
        _world_call(args.world, "control", "gz.msgs.WorldControl", "multi_step: 1")
        n_steps += 1
        node.spin_for(POLL_S)
    if not _world_call(args.world, "control", "gz.msgs.WorldControl", "pause: false"):
        log.error("live_start: unpause failed")
        return 2
    node.spin_for(POLL_S)
    sim0, wall0 = node.sim_s(), time.monotonic()
    log.info(
        f"live_start: running at real-time factor {args.rtf} after {n_steps} start-up steps; "
        f"pause at sim t = {args.stop_at_s} s"
    )
    while node.sim_s() < args.stop_at_s:
        node.spin_for(POLL_S)
    _world_call(args.world, "control", "gz.msgs.WorldControl", "pause: true")
    sim_s, wall_s = node.sim_s() - sim0, time.monotonic() - wall0
    log.info(
        f"live_start: done, {sim_s:.1f} sim s in {wall_s:.1f} wall s, "
        f"real-time factor {sim_s / wall_s:.3f} (requested {args.rtf})"
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    args_all = sys.argv if argv is None else argv
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("world")
    ap.add_argument("--rtf", type=float, required=True, help="real-time factor (> 0)")
    ap.add_argument("--step-s", type=float, default=0.001, help="physics step, s (the world's)")
    ap.add_argument("--stop-at-s", type=float, required=True, help="sim time to pause at, s")
    ap.add_argument("--wait-subs", nargs="*", default=[], help="topics that need a subscriber")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(args_all)[1:])
    if not args.rtf > 0:
        ap.error("--rtf must be > 0")
    rclpy.init(args=args_all)
    node = LiveStart()
    try:
        return run(args, node)
    except KeyboardInterrupt:
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
