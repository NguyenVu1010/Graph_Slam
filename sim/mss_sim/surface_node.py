"""rclpy node that sets the robot's WheelSlip compliances from the ground surface (backlog L3).

    python -m mss_sim.surface_node <world.planes.yaml> <world> <model> \\
        --ros-args -p use_sim_time:=true

Subscribes ``/ground_truth`` (body pose in world) and feeds every sample to
``mss_sim.surface.SurfaceCommander``, which sends ``gz.msgs.WheelSlipParametersCmd`` for the whole
model once per surface change, retrying every ``RESEND_NS`` of sim time until acknowledged. The
request goes to the ``UserCommands`` service ``/world/<world>/wheel_slip/blocking`` through the
``gz service`` CLI (``mss_sim.gz_cli``, as ``live_start``), so no Gazebo Python bindings are
needed. The call blocks this node's only callback for its wall duration (at most
``REQUEST_TIMEOUT_MS``); ground-truth samples that arrive meanwhile queue (depth 50) or are
dropped, which costs nothing here: only the next position matters. Logs every surface change
with its sim time and every failed request. This module is the untested glue; the decisions are
tested in ``tests/python/test_surface.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mss_sim.gz_cli import world_call
from mss_sim.surface import Surface, SurfaceCommander, load_surfaces, wheel_slip_request

REQUEST_TIMEOUT_MS = 2000  # wall ms per gz service request
SERVICE = "wheel_slip/blocking"
REQTYPE = "gz.msgs.WheelSlipParametersCmd"


class SurfaceNode(Node):
    def __init__(self, sidecar: Path, world: str, model: str) -> None:
        super().__init__("mss_sim_surface")
        surfaces = load_surfaces(sidecar)
        if surfaces is None:
            raise ValueError(f"{sidecar}: no surfaces")
        self._world = world
        self._model = model
        self.cmd = SurfaceCommander(surfaces, self._send)
        self.create_subscription(Odometry, "/ground_truth", self._on_gt, 50)
        self.get_logger().info(
            f"surface: {len(surfaces.patches)} patches, default {surfaces.default.name}, "
            f"service /world/{world}/{SERVICE}"
        )

    def _send(self, s: Surface) -> bool:
        req = wheel_slip_request(self._model, s)
        ok = world_call(self._world, SERVICE, REQTYPE, req, REQUEST_TIMEOUT_MS)
        if not ok:
            self.get_logger().warning(f"surface: /world/{self._world}/{SERVICE} failed: {req}")
        return ok

    def _on_gt(self, msg: Odometry) -> None:
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        s = self.cmd.update(t_ns, p.x, p.y)
        if s is not None:
            self.get_logger().info(
                f"surface: t = {t_ns} ns at ({p.x:.2f}, {p.y:.2f}) m -> {s.name} "
                f"(long {s.slip_compliance_longitudinal}, lat {s.slip_compliance_lateral})"
            )


def main(argv: list[str] | None = None) -> int:
    args_ros = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("sidecar", type=Path)
    ap.add_argument("world")
    ap.add_argument("model")
    args = ap.parse_args(args_ros[1:])
    rclpy.init(args=sys.argv if argv is None else argv)
    node = SurfaceNode(args.sidecar, args.world, args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"surface: {node.cmd.n_sent} requests, {node.cmd.n_failed} failed")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
