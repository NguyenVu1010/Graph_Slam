"""Play a bridged bag into mss_node, optionally with RViz (ADR-024).

    ros2 launch mss_ros2 mss_bag.launch.py bag:=<bag dir> [rate:=1.0] [rviz:=true]
        [config_path:=<default.yaml>] [overlay:=<overlay.yaml>] [body_frame:=<frames.body>]
        [origin_lla:="lat_deg lon_deg alt_m"] [delay_s:=5.0] [exit_on_end:=false]

The bag is the output of the noise bridge (``sim/mss_sim/bridge.py``: per-point ``t`` and
``ring``). Everything runs on sim time: ``ros2 bag play --clock`` publishes /clock and the
node and RViz use it. ``overlay`` is the scenario's pipeline overlay (E-01:
``config/sim/warehouse.yaml``, installed under ``share/mss_ros2/config/sim``); the default
``""`` runs config/default.yaml alone. ``origin_lla`` fixes the GNSS ENU origin (the scenario's
``origin_lla``, as mss_bench's data conversion uses for recorded bags); empty takes the first
valid fix. ``body_frame`` empty (default) publishes TF to ``frames.body`` of the config
(``imu_link``), the parent of ``lidar_link`` in the sim bags' ``/tf_static``.
``exit_on_end:=true`` shuts the launch down when the bag ends.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context: LaunchContext) -> list:
    share = Path(get_package_share_directory("mss_ros2"))
    arg = {
        k: LaunchConfiguration(k).perform(context)
        for k in (
            "bag",
            "rate",
            "config_path",
            "overlay",
            "body_frame",
            "origin_lla",
            "delay_s",
            "exit_on_end",
        )
    }
    if not arg["bag"]:
        raise RuntimeError("mss_bag.launch.py: bag:=<bridged bag dir> is required")
    params: dict = {
        "use_sim_time": True,
        "config_path": arg["config_path"],
        "config_overlay": arg["overlay"],
        "tf.body_frame": arg["body_frame"],
    }
    origin = arg["origin_lla"].split()
    if origin:
        if len(origin) != 3:
            raise RuntimeError("origin_lla must be 'lat_deg lon_deg alt_m'")
        params["gnss_origin.from_first_fix"] = False
        params["gnss_origin.latitude_deg"] = float(origin[0])
        params["gnss_origin.longitude_deg"] = float(origin[1])
        params["gnss_origin.altitude_m"] = float(origin[2])
    node = Node(
        package="mss_ros2",
        executable="mss_node",
        name="mss_node",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )
    play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            arg["bag"],
            "--clock",
            "--rate",
            arg["rate"],
            "--delay",
            arg["delay_s"],
            "--qos-profile-overrides-path",
            str(share / "config" / "bag_qos.yaml"),
        ],
        output="screen",
    )
    actions = [node, play]
    if arg["exit_on_end"].lower() in ("true", "1"):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=play, on_exit=[EmitEvent(event=Shutdown())])
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mss_ros2"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value="", description="bridged bag directory"),
            DeclareLaunchArgument("rate", default_value="1.0", description="playback rate"),
            DeclareLaunchArgument("rviz", default_value="false", description="start RViz"),
            DeclareLaunchArgument(
                "config_path", default_value=str(share / "config" / "default.yaml")
            ),
            DeclareLaunchArgument("overlay", default_value="", description="pipeline overlay"),
            DeclareLaunchArgument(
                "body_frame", default_value="", description="default: frames.body of the config"
            ),
            DeclareLaunchArgument("origin_lla", default_value=""),
            DeclareLaunchArgument(
                "delay_s",
                default_value="5.0",
                description="play delay so the node subscribes first, s",
            ),
            DeclareLaunchArgument("exit_on_end", default_value="false"),
            OpaqueFunction(function=_setup),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(share / "rviz" / "mss.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
