"""rclpy node: all transforms of a ``tf_static.yaml`` in ONE latched ``/tf_static`` message.

    python -m mss_sim.tf_static <assets>/tf_static.yaml --ros-args -p use_sim_time:=true

``tools/gen_sim_assets.py`` writes the YAML (``transforms``: ``frame_id``, ``child_frame_id``,
``translation_m``, ``quaternion_xyzw``). One message instead of one ``static_transform_publisher``
per transform: two publishers stamp two messages with the same time, and the noise report requires
strictly increasing stamps per topic. QoS is transient local (latched), as tf2's static broadcaster.
Needs ROS 2 (rclpy); covered by the launch run, not unit-tested.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StaticTf(Node):
    def __init__(self, path: Path) -> None:
        super().__init__("mss_sim_tf_static")
        transforms = yaml.safe_load(path.read_text(encoding="utf-8"))["transforms"]
        stamp = self.get_clock().now().to_msg()
        msg = TFMessage()
        for tf in transforms:
            ts = TransformStamped()
            ts.header.stamp = stamp
            ts.header.frame_id = tf["frame_id"]
            ts.child_frame_id = tf["child_frame_id"]
            tr = ts.transform.translation
            tr.x, tr.y, tr.z = (float(v) for v in tf["translation_m"])
            q = ts.transform.rotation  # field order x, y, z, w
            q.x, q.y, q.z, q.w = (float(v) for v in tf["quaternion_xyzw"])
            msg.transforms.append(ts)
        self._pub = self.create_publisher(TFMessage, "/tf_static", _QOS)
        self._pub.publish(msg)
        names = ", ".join(f"{t['frame_id']}->{t['child_frame_id']}" for t in transforms)
        self.get_logger().info(f"tf_static: {names}")


def main(argv: list[str] | None = None) -> int:
    args_all = sys.argv if argv is None else argv
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tf_static_yaml", type=Path)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(args_all)[1:])
    rclpy.init(args=args_all)
    node = StaticTf(args.tf_static_yaml)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
