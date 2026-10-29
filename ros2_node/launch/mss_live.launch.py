"""Live Gazebo run into mss_node, optionally with RViz (ADR-011 2026-10-29 amendment, ADR-024).

    ros2 launch mss_ros2 mss_live.launch.py scenario:=sim/scenarios/E-01.yaml [rviz:=true]
        [rtf:=1.0] [headless:=true] [render_engine:=ogre2] [seed:=<scenario seed>]
        [assets_dir:=results/live/<id>/assets] [config_path:=<default.yaml>]
        [overlay:=<scenario pipeline_config>] [body_frame:=<frames.body>] [exit_on_end:=false]

One process tree:
1. ``tools/gen_sim_assets.py`` (synchronously, at launch) writes the robot and bridge assets.
2. ``sim/launch/sim.launch.py``: Gazebo, paused, with the raw sensor topics under ``/sim/raw``,
   ``/tf_static`` and the route driver.
3. ``python3 -m mss_sim.live_node``: the online noise layer (the offline bridge's models) from
   ``/sim/raw/*`` to ``/lidar/points``, ``/imu/data``, ``/gnss/fix``, ``/wheel/odom``.
4. ``mss_node`` with the scenario's overlay and ENU origin (``origin_lla``).
5. ``python3 -m mss_sim.live_start``: once mss_node and the noise node subscribe, sets the
   real-time factor ``rtf``, unpauses, and pauses again at ``duration_s`` + 2 s of sim time.
   ``exit_on_end:=true`` then shuts the launch down; otherwise everything stays up for viewing.
6. RViz (``rviz:=true``).

All nodes use sim time (Gazebo ``/clock``). The repo is the ancestor of the scenario file that
holds ``sim/mss_sim``. Python needs the ``sim/`` dependencies (rosbags, shapely, scipy) on
``PYTHONPATH`` as for the offline bridge (``sim/README.md``).
"""

import os
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RAW_PREFIX = "/sim/raw"
STOP_MARGIN_S = 2.0  # sim s after duration_s before the world is paused, as record_scenario.sh
NOISY_TOPICS = ["/lidar/points", "/imu/data", "/gnss/fix", "/wheel/odom"]
RAW_TOPICS = [RAW_PREFIX + t for t in ("/lidar/points", "/imu/data", "/gnss/fix", "/joint_states")]
ROS_SIM_TIME = ["--ros-args", "-p", "use_sim_time:=true"]


def _repo_of(scenario: Path) -> Path:
    for parent in scenario.parents:
        if (parent / "sim" / "mss_sim" / "__init__.py").is_file():
            return parent
    raise RuntimeError(f"mss_live.launch.py: no repo (sim/mss_sim) above {scenario}")


def _flag(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes")


def _setup(context: LaunchContext) -> list:
    arg = {
        k: LaunchConfiguration(k).perform(context)
        for k in (
            "scenario",
            "rtf",
            "headless",
            "render_engine",
            "seed",
            "assets_dir",
            "config_path",
            "overlay",
            "body_frame",
            "exit_on_end",
        )
    }
    if not arg["scenario"]:
        raise RuntimeError("mss_live.launch.py: scenario:=<sim/scenarios/*.yaml> is required")
    scenario = Path(arg["scenario"]).resolve()
    repo = _repo_of(scenario)
    sc = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    assets = Path(arg["assets_dir"] or repo / "results" / "live" / sc["id"] / "assets").resolve()
    env_path = os.pathsep.join(
        p for p in (str(repo / "sim"), os.environ.get("PYTHONPATH", "")) if p
    )
    gen = subprocess.run(
        [
            "python3",
            str(repo / "tools" / "gen_sim_assets.py"),
            "--profile",
            str(repo / sc["profile"]),
            "--pipeline-config",
            str(repo / sc["pipeline_config"]),
            "--out",
            str(assets),
        ],
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "PYTHONPATH": env_path},
    )
    if gen.returncode != 0:
        raise RuntimeError(f"gen_sim_assets.py failed:\n{gen.stdout}{gen.stderr}")

    lat, lon, alt = (float(v) for v in sc["origin_lla"])
    params: dict = {
        "use_sim_time": True,
        "config_path": arg["config_path"],
        "config_overlay": arg["overlay"] or str(repo / sc["pipeline_config"]),
        "tf.body_frame": arg["body_frame"],
        "gnss_origin.from_first_fix": False,
        "gnss_origin.latitude_deg": lat,
        "gnss_origin.longitude_deg": lon,
        "gnss_origin.altitude_m": alt,
    }
    seed = ["--seed", arg["seed"]] if arg["seed"] else []
    py_env = {"PYTHONPATH": env_path}
    starter = ExecuteProcess(
        cmd=[
            "python3", "-m", "mss_sim.live_start", sc["world"], "--rtf", arg["rtf"],
            "--stop-at-s", str(float(sc["duration_s"]) + STOP_MARGIN_S),
            "--wait-subs", *NOISY_TOPICS, *RAW_TOPICS, *ROS_SIM_TIME,
        ],
        name="live_start",
        output="screen",
        additional_env=py_env,
    )  # fmt: skip
    actions: list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(repo / "sim" / "launch" / "sim.launch.py")),
            launch_arguments={
                "scenario": str(scenario),
                "assets_dir": str(assets),
                "headless": arg["headless"],
                "render_engine": arg["render_engine"],
                "paused": "true",
                "raw_prefix": RAW_PREFIX,
            }.items(),
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                "-m",
                "mss_sim.live_node",
                str(scenario),
                "--raw-prefix",
                RAW_PREFIX,
                *seed,
                *ROS_SIM_TIME,
            ],
            name="live_noise",
            output="screen",
            additional_env=py_env,
        ),  # fmt: skip
        Node(
            package="mss_ros2",
            executable="mss_node",
            name="mss_node",
            output="screen",
            emulate_tty=True,
            parameters=[params],
        ),
        starter,
    ]
    if _flag(arg["exit_on_end"]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=starter, on_exit=[EmitEvent(event=Shutdown())])
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mss_ros2"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="", description="scenario YAML"),
            DeclareLaunchArgument("rviz", default_value="false", description="start RViz"),
            DeclareLaunchArgument("rtf", default_value="1.0", description="real-time factor"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value="ogre2"),
            DeclareLaunchArgument("seed", default_value="", description="default: scenario"),
            DeclareLaunchArgument(
                "assets_dir", default_value="", description="default: results/live/<id>/assets"
            ),
            DeclareLaunchArgument(
                "config_path", default_value=str(share / "config" / "default.yaml")
            ),
            DeclareLaunchArgument(
                "overlay", default_value="", description="default: scenario pipeline_config"
            ),
            DeclareLaunchArgument(
                "body_frame", default_value="", description="default: frames.body of the config"
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
