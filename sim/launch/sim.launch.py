"""Gazebo Harmonic simulation of one scenario (ADR-011 sections 2, 5 and 6).

    ros2 launch sim/launch/sim.launch.py scenario:=sim/scenarios/E-01.yaml \\
        [world:=<name>] [assets_dir:=results/sim-assets] [headless:=true] \\
        [render_engine:=ogre2] [paused:=false] [driver:=true] [raw_prefix:=]

Starts the gz server (``ros_gz_sim``, ``-s -r --headless-rendering`` by default, gz seed = the
scenario seed), spawns the generated robot (``tools/gen_sim_assets.py``) at the world's ``spawn``
pose (``sim/worlds/<world>.planes.yaml``), runs ``ros_gz_bridge parameter_bridge`` with the
generated ``bridge.yaml``, the ``/tf_static`` publisher (``python3 -m mss_sim.tf_static``) and
the route driver (``python3 -m mss_sim.driver``). Every node uses sim time.

``paused:=true`` starts the server paused (no ``-r``) so a recorder can subscribe first;
``tools/record_scenario.sh`` then unpauses through the world control service. ``world``
defaults to the scenario's world. Relative paths resolve against the current directory.
``raw_prefix:=/sim/raw`` moves the raw sensor topics that share a name with the noisy ones
(``RAW_SENSOR_TOPICS``) under that namespace, for the live noise node
(``ros2_node/launch/mss_live.launch.py``); empty (default) keeps the recording names.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REPO = Path(__file__).resolve().parents[2]
SIM_TIME = {"use_sim_time": True}
RAW_SENSOR_TOPICS = ("/lidar/points", "/imu/data", "/gnss/fix", "/joint_states")


def _flag(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes")


def _setup(context: LaunchContext) -> list:
    scenario = Path(LaunchConfiguration("scenario").perform(context)).resolve()
    sc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    world = LaunchConfiguration("world").perform(context) or sc["world"]
    world_sdf = REPO / "sim" / "worlds" / f"{world}.sdf"
    side = yaml.safe_load(world_sdf.with_suffix(".planes.yaml").read_text(encoding="utf-8"))
    assets = Path(LaunchConfiguration("assets_dir").perform(context)).resolve()
    meta = yaml.safe_load((assets / "assets.yaml").read_text(encoding="utf-8"))
    engine = LaunchConfiguration("render_engine").perform(context)

    gz_args = ["-s"] if _flag(context, "headless") else []
    if not _flag(context, "paused"):
        gz_args.append("-r")
    gz_args += ["--headless-rendering"] if _flag(context, "headless") else []
    gz_args += ["--render-engine-server", engine, "--seed", str(int(sc["seed"])), str(world_sdf)]

    spawn = side["spawn"]
    prefix = LaunchConfiguration("raw_prefix").perform(context).rstrip("/")
    remaps = [(t, prefix + t) for t in RAW_SENSOR_TOPICS] if prefix else []
    actions: list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": " ".join(gz_args), "on_exit_shutdown": "true"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_robot",
            output="screen",
            parameters=[SIM_TIME],
            arguments=[
                "-world",
                world,
                "-file",
                str(assets / meta["robot_sdf"]),
                "-name",
                meta["model_name"],
                "-x",
                str(spawn["x_m"]),
                "-y",
                str(spawn["y_m"]),
                "-z",
                str(meta["spawn_z_m"]),
                "-Y",
                str(spawn["yaw_rad"]),
            ],
        ),  # fmt: skip
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{"config_file": str(assets / meta["bridge_yaml"]), **SIM_TIME}],
            remappings=remaps,
        ),
    ]
    env_path = os.pathsep.join(
        p for p in (str(REPO / "sim"), os.environ.get("PYTHONPATH", "")) if p
    )
    ros_sim_time = ["--ros-args", "-p", "use_sim_time:=true"]
    actions.append(
        ExecuteProcess(
            cmd=[
                "python3",
                "-m",
                "mss_sim.tf_static",
                str(assets / "tf_static.yaml"),
                *ros_sim_time,
            ],
            name="tf_static",
            output="screen",
            additional_env={"PYTHONPATH": env_path},
        )
    )
    if _flag(context, "driver"):
        actions.append(
            ExecuteProcess(
                cmd=["python3", "-m", "mss_sim.driver", str(scenario), *ros_sim_time],
                name="driver",
                output="screen",
                additional_env={"PYTHONPATH": env_path},
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", description="scenario YAML, sim/scenarios/*.yaml"),
            DeclareLaunchArgument("world", default_value="", description="default: scenario"),
            DeclareLaunchArgument(
                "assets_dir",
                default_value=str(REPO / "results" / "sim-assets"),
                description="output dir of tools/gen_sim_assets.py",
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value="ogre2"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("driver", default_value="true"),
            DeclareLaunchArgument("raw_prefix", default_value="", description="e.g. /sim/raw"),
            OpaqueFunction(function=_setup),
        ]
    )
