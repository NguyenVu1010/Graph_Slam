"""Launch mss_node with the shared config/default.yaml.

The YAML is read by the core config loader (ADR-008), so the node gets only its path as the
string parameter `config_path`; override it with `config:=<file>`.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mss_ros2"))
    default_cfg = str(share / "config" / "default.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_cfg),
            Node(
                package="mss_ros2",
                executable="mss_node",
                name="mss_node",
                output="screen",
                emulate_tty=True,
                parameters=[{"config_path": LaunchConfiguration("config")}],
            ),
        ]
    )
