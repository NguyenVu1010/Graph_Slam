"""Gamepad buttons -> SLAM commands (ADR-031).

    ros2 run mss_teleop joy_commands --ros-args --params-file <share>/config/joy.yaml

Listens on ``/joy`` (sensor_msgs/Joy). A press (rising edge) of ``save_button`` calls
``/mss/save`` (std_srvs/Trigger); of ``pause_button`` toggles ``/mss/pause`` (std_srvs/SetBool:
pause when the world runs, resume when it is paused). The state is not remembered here: it is
mss_node's latched ``/mss/paused`` (std_msgs/Bool), which the node infers from the sim clock, so
a pause from the CLI, the Gazebo GUI or ``/mss/save`` stays in sync (review M-1, N-7). Calls are
asynchronous; the result is logged. A press while a call of the same kind is pending is ignored,
unless that call is older than ``call_timeout_s`` (mss_node died mid-call, review N-6). Button
indices come from the parameters (``config/joy.yaml``, Xbox layout by default); ``-1`` disables
a button.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.client import Client
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from mss_teleop.teleop_logic import ButtonEdges, PendingCalls, pause_request


class JoyCommands(Node):
    def __init__(self) -> None:
        super().__init__("mss_joy_commands")
        self.save_button = int(self.declare_parameter("save_button", -1).value)
        self.pause_button = int(self.declare_parameter("pause_button", -1).value)
        call_timeout_s = float(self.declare_parameter("call_timeout_s", 15.0).value)
        self.edges = ButtonEdges([b for b in (self.save_button, self.pause_button) if b >= 0])
        self.pending = PendingCalls(call_timeout_s)
        self.paused: bool | None = None  # last /mss/paused; None before the first one
        self.save_cli = self.create_client(Trigger, "/mss/save")
        self.pause_cli = self.create_client(SetBool, "/mss/pause")
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/mss/paused", self.on_paused, latched)
        self.get_logger().info(
            f"joy_commands: save = button {self.save_button}, pause = button {self.pause_button}"
        )

    def on_paused(self, msg: Bool) -> None:
        self.paused = bool(msg.data)

    def on_joy(self, msg: Joy) -> None:
        for b in self.edges.update(msg.buttons):
            if b == self.save_button:
                self.call_save()
            elif b == self.pause_button:
                self.call_pause()

    def _ready(self, kind: str, client: Client) -> float | None:
        """The call's token, or None when the press is ignored."""
        token = self.pending.start(kind, time.monotonic())
        if token is None:
            self.get_logger().info(f"{kind}: previous call still pending, press ignored")
            return None
        if not client.service_is_ready():
            self.pending.finish(kind, token)
            self.get_logger().warning(f"{kind}: service not available (mss_node with viz.enabled?)")
            return None
        return token

    def call_save(self) -> None:
        token = self._ready("save", self.save_cli)
        if token is None:
            return
        future = self.save_cli.call_async(Trigger.Request())

        def done(f: Future) -> None:
            self.pending.finish("save", token)
            res = f.result()
            if res is not None and res.success:
                self.get_logger().info(f"saved: {res.message}")
            else:
                self.get_logger().error(f"save failed: {res.message if res else f.exception()}")

        future.add_done_callback(done)

    def call_pause(self) -> None:
        want = pause_request(self.paused)
        if want is None:
            self.get_logger().warning("pause: world state unknown (no /mss/paused yet), ignored")
            return
        token = self._ready("pause", self.pause_cli)
        if token is None:
            return
        future = self.pause_cli.call_async(SetBool.Request(data=want))

        def done(f: Future) -> None:
            self.pending.finish("pause", token)
            res = f.result()
            if res is not None and res.success:
                self.get_logger().info(res.message)
            else:
                self.get_logger().error(f"pause failed: {res.message if res else f.exception()}")

        future.add_done_callback(done)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=sys.argv if argv is None else argv)
    node = JoyCommands()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
