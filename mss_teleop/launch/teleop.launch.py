"""Drive the simulated robot yourself and watch SLAM live in RViz2 (ADR-031).

    cd <repo>    # or pass repo:=<path>
    ros2 launch mss_teleop teleop.launch.py [world:=warehouse|outdoor] [input:=keyboard|joy]
        [rviz:=true] [headless:=true] [rtf:=1.0] [repo:=<cwd>] [keyboard_prefix:=]

One launch (``mss_ros2/launch/mss_live.launch.py`` underneath): Gazebo with the world of the
reused scenario (``warehouse`` -> E-07W, ``outdoor`` -> E-07: noise, seed, GNSS, wheels, spawn
pose), no route driver and no stop time, the live noise layer, ``mss_node`` with that scenario's
loops-on overlay plus ``viz.enabled`` (``config/slam_viz.yaml``), RViz2 with
``rviz/teleop.rviz``, and the input:

- ``input:=joy``: ``joy_node`` + ``teleop_twist_joy`` (hold LB, left stick) and
  ``joy_commands`` (Y saves, Start pauses / resumes), mapping in ``config/joy.yaml``;
- ``input:=keyboard`` (default; on WSL a USB pad needs ``usbipd``): ``teleop_twist_keyboard``
  needs a terminal of its own. With ``keyboard_prefix:="xterm -e"`` (or another terminal
  command) the launch opens it; otherwise it prints the command to run in a second terminal.

Speed limits for both inputs are the robot's maxima from ``sim/models/robot.yaml``. Services:
``ros2 service call /mss/save std_srvs/srv/Trigger`` writes ``kf_opt.tum``, ``loops.csv`` and
``map.pcd`` to ``results/teleop/<run id>/save-<n>/``; ``/mss/pause`` (``std_srvs/srv/SetBool``)
pauses the whole Gazebo world, so the robot cannot be driven while paused; ``/mss/paused`` tells
the world state mss_node reads from the sim clock.
"""

import os
import shlex
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mss_teleop.teleop_logic import drive_limits, find_repo, world_scenario

INPUTS = ("keyboard", "joy")


def _setup(context: LaunchContext) -> list:
    arg = {
        k: LaunchConfiguration(k).perform(context)
        for k in ("world", "input", "rviz", "headless", "rtf", "repo", "keyboard_prefix")
    }
    if arg["input"] not in INPUTS:
        raise RuntimeError(f"teleop.launch.py: input:={arg['input']} must be one of {INPUTS}")
    repo = find_repo(Path(arg["repo"] or os.getcwd()))
    run = world_scenario(repo, arg["world"])
    limits = drive_limits(repo / "sim" / "models" / "robot.yaml")
    share = Path(get_package_share_directory("mss_teleop"))
    live = Path(get_package_share_directory("mss_ros2")) / "launch" / "mss_live.launch.py"

    actions: list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(live)),
            launch_arguments={
                "scenario": str(run.scenario),
                "overlay": str(run.overlay),
                "driver": "false",
                "stop_at_end": "false",
                "exit_on_end": "false",
                "viz": "true",
                "node_params": str(share / "config" / "slam_viz.yaml"),
                "rviz": arg["rviz"],
                "rviz_config": str(share / "rviz" / "teleop.rviz"),
                "headless": arg["headless"],
                "rtf": arg["rtf"],
            }.items(),
        )
    ]
    if arg["input"] == "joy":
        joy_yaml = str(share / "config" / "joy.yaml")
        scales = {
            "scale_linear.x": limits.linear_mps,
            "scale_angular.yaw": limits.angular_radps,
        }
        actions += [
            Node(package="joy", executable="joy_node", name="joy_node", parameters=[joy_yaml]),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[joy_yaml, scales],
            ),
            Node(
                package="mss_teleop",
                executable="joy_commands",
                name="mss_joy_commands",
                output="screen",
                parameters=[joy_yaml],
            ),
        ]
    else:
        kb_params = {"speed": limits.linear_mps, "turn": limits.angular_radps}
        if arg["keyboard_prefix"]:
            actions.append(
                Node(
                    package="teleop_twist_keyboard",
                    executable="teleop_twist_keyboard",
                    name="teleop_twist_keyboard",
                    prefix=arg["keyboard_prefix"],
                    parameters=[kb_params],
                )
            )
        else:
            cmd = (
                "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args"
                f" -p speed:={limits.linear_mps} -p turn:={limits.angular_radps}"
            )
            env = f"export ROS_DOMAIN_ID={shlex.quote(os.environ.get('ROS_DOMAIN_ID', '0'))}"
            actions.append(
                LogInfo(
                    msg=(
                        "teleop: keyboard input. In a second terminal (same ROS environment):\n"
                        f"  {env}\n  {cmd}\n"
                        "Save: ros2 service call /mss/save std_srvs/srv/Trigger\n"
                        "Pause/resume: ros2 service call /mss/pause std_srvs/srv/SetBool "
                        "'{data: true}' (false to resume)"
                    )
                )
            )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="warehouse", description="or outdoor"),
            DeclareLaunchArgument("input", default_value="keyboard", description="or joy"),
            DeclareLaunchArgument("rviz", default_value="true", description="start RViz2"),
            DeclareLaunchArgument("headless", default_value="true", description="no Gazebo GUI"),
            DeclareLaunchArgument("rtf", default_value="1.0", description="real-time factor"),
            DeclareLaunchArgument("repo", default_value="", description="default: cwd"),
            DeclareLaunchArgument(
                "keyboard_prefix", default_value="", description='e.g. "xterm -e"'
            ),
            OpaqueFunction(function=_setup),
        ]
    )
