"""Generate the Gazebo assets of a scenario (ADR-011 sections 2 and 5).

    python tools/gen_sim_assets.py --profile sim/noise/default.yaml \\
        --pipeline-config config/sim/warehouse.yaml [--out results/sim-assets]
    python tools/gen_sim_assets.py --check-worlds   # every world vs its planes sidecar

Writes under ``--out`` (a build artefact, never committed):
- ``robot.sdf``: ``sim/models/robot.sdf.j2`` rendered with ``sim/models/robot.yaml``, the noise
  profile (true kinematics, IMU/LiDAR noise, rates) and the pipeline overlay (``T_b_l``, ``p_b_g``,
  frame names).
- ``bridge.yaml``: ``ros_gz_bridge parameter_bridge`` config of the raw topics.
- ``tf_static.yaml``: ``T_b_l`` and ``p_b_g`` for ``mss_sim.tf_static`` (one ``/tf_static`` msg).
- ``assets.yaml``: what the launch file needs (model name, spawn height, frames).

The world helpers (``world_box_faces``, ``check_world``) compare a hand-written world SDF with the
reference planes of its ``<world>.planes.yaml`` sidecar: every plane must lie on a box face.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import jinja2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from mss_sim.config_overlay import load_pipeline_config
from mss_sim.profile import NoiseProfile, load_profile

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / "config" / "default.yaml"
ROBOT_YAML = REPO / "sim" / "models" / "robot.yaml"
TEMPLATE = REPO / "sim" / "models" / "robot.sdf.j2"
WORLDS = REPO / "sim" / "worlds"
DEFAULT_OUT = REPO / "results" / "sim-assets"

GRAVITY_MPS2 = 9.80665  # docs/conventions.md
PLANE_TOL_M = 1e-3  # a sidecar plane must lie on a box face within this distance
_UNIT_TOL = 1e-9  # plane normals must be axis-aligned unit vectors within this

LEFT_JOINT, RIGHT_JOINT = "left_wheel_joint", "right_wheel_joint"  # mss_sim.odom.WheelQuantizer

# Raw topics bridged from Gazebo (ADR-011 section 6 plus /joint_states and /cmd_vel).
BRIDGE_TOPICS: list[tuple[str, str, str, str]] = [
    # (topic, ROS type, gz type, direction)
    ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
    ("/lidar/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", "GZ_TO_ROS"),
    ("/imu/data", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
    ("/gnss/fix", "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat", "GZ_TO_ROS"),
    ("/joint_states", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS"),
    ("/ground_truth", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
    ("/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
]


class AssetError(ValueError):
    """Invalid robot description, pipeline overlay or world. The message names the key."""


# ---------------------------------------------------------------- robot


def _positive(robot: dict, dotted: str) -> float:
    node: Any = robot
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise AssetError(f"robot.yaml: {dotted}: missing")
        node = node[part]
    if isinstance(node, bool) or not isinstance(node, (int, float)) or not node > 0:
        raise AssetError(f"robot.yaml: {dotted}: must be a number > 0, got {node!r}")
    return float(node)


def load_robot(path: Path = ROBOT_YAML) -> dict:
    """Load ``robot.yaml`` and validate every value the template uses."""
    robot = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(robot, dict):
        raise AssetError(f"{path}: expected a mapping")
    for key in (
        "body.mass_kg",
        "body.center_z_m",
        "wheel.mass_kg",
        "wheel.width_m",
        "wheel.mu",
        "wheel.joint_damping_nms_per_rad",
        "wheel.joint_friction_nm",
        "wheel.motor_torque_limit_nm",
        "caster.radius_m",
        "caster.offset_x_m",
        "drive.max_linear_velocity_mps",
        "drive.max_linear_acceleration_mps2",
        "drive.max_angular_velocity_radps",
        "drive.max_angular_acceleration_radps2",
        "lidar.beams",
        "lidar.vertical_fov_deg",
        "lidar.horizontal_samples",
        "lidar.max_range_m",
        "lidar.range_resolution_m",
        "ground_truth_rate_hz",
    ):
        _positive(robot, key)
    size = robot["body"].get("size_m")
    if not (isinstance(size, list) and len(size) == 3 and all(_is_pos(v) for v in size)):
        raise AssetError(f"robot.yaml: body.size_m: expected [x_m, y_m, z_m] > 0, got {size!r}")
    for key in ("slip_compliance_longitudinal", "slip_compliance_lateral"):
        v = robot.get("wheel_slip", {}).get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v < 0:
            raise AssetError(f"robot.yaml: wheel_slip.{key}: must be a number >= 0, got {v!r}")
    shape = robot["wheel"].get("collision_shape")
    if shape not in ("cylinder", "sphere"):
        raise AssetError(
            f"robot.yaml: wheel.collision_shape: expected 'cylinder' or 'sphere', got {shape!r}"
        )
    if not isinstance(robot.get("model_name"), str) or not robot["model_name"]:
        raise AssetError("robot.yaml: model_name: expected a non-empty string")
    return robot


def _is_pos(v: Any) -> bool:
    return not isinstance(v, bool) and isinstance(v, (int, float)) and v > 0


def box_inertia(mass_kg: float, x_m: float, y_m: float, z_m: float) -> dict[str, float]:
    """Principal inertia of a solid box about its centre, kg m^2."""
    return {
        "ixx": mass_kg * (y_m**2 + z_m**2) / 12.0,
        "iyy": mass_kg * (x_m**2 + z_m**2) / 12.0,
        "izz": mass_kg * (x_m**2 + y_m**2) / 12.0,
    }


def cylinder_inertia(mass_kg: float, radius_m: float, length_m: float) -> dict[str, float]:
    """Solid cylinder about its centre: ``axial`` (symmetry axis) and ``radial``, kg m^2."""
    return {
        "axial": mass_kg * radius_m**2 / 2.0,
        "radial": mass_kg * (3.0 * radius_m**2 + length_m**2) / 12.0,
    }


def quat_wxyz_to_rpy(q_wxyz: list[float]) -> tuple[float, float, float]:
    """SDF ``roll pitch yaw`` (R = Rz(yaw) Ry(pitch) Rx(roll)) of a Hamilton (w, x, y, z) quat.

    The quaternion is normalised first (the overlay stores 8 decimals).
    """
    w, x, y, z = (float(v) for v in q_wxyz)
    rot = Rotation.from_quat([x, y, z, w])  # scipy order (x, y, z, w); normalises
    roll, pitch, yaw = rot.as_euler("xyz")  # lowercase = extrinsic = Rz Ry Rx
    return float(roll), float(pitch), float(yaw)


def rpy_to_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    """3x3 rotation of an SDF ``roll pitch yaw`` triple."""
    return Rotation.from_euler("xyz", rpy).as_matrix()


def imu_discrete_noise(profile: NoiseProfile) -> dict[str, float]:
    """Gazebo IMU noise elements from the continuous-time profile (template comment)."""
    imu = profile.imu
    sqrt_f = math.sqrt(imu.rate_hz)
    return {
        "gyro_stddev": imu.gyro_noise_density * sqrt_f,
        "acc_stddev": imu.acc_noise_density * sqrt_f,
        "gyro_bias_stddev": imu.gyro_bias_sigma_radps,
        "acc_bias_stddev": imu.acc_bias_sigma_mps2,
        "gyro_dynamic_bias_stddev": imu.gyro_random_walk,
        "acc_dynamic_bias_stddev": imu.acc_random_walk,
        "bias_correlation_time_s": imu.bias_corr_time_s,
    }


def gz_range_sigma(profile: NoiseProfile, override_m: float | None = None) -> float:
    """Gazebo LiDAR range stddev = the declared ``lidar.range_sigma_m``; the renderer's own error
    is measured per world (``sim/noise/render/<world>.yaml``), not compensated here.
    ``override_m`` (render calibration runs: 0) replaces it."""
    if override_m is not None:
        if not (math.isfinite(override_m) and override_m >= 0):
            raise AssetError(f"--lidar-gz-sigma must be a finite number >= 0, got {override_m}")
        return override_m
    return profile.lidar.range_sigma_m


@dataclass(frozen=True)
class Extrinsics:
    t_b_l_m: tuple[float, float, float]
    q_b_l_wxyz: tuple[float, float, float, float]  # Hamilton order (w, x, y, z)
    p_b_g_m: tuple[float, float, float]
    frames: dict[str, str]  # world, body, lidar, gnss


def load_extrinsics(pipeline_config: Path, default: Path = DEFAULT_CONFIG) -> Extrinsics:
    """``T_b_l``, ``p_b_g`` and frame names of ``default`` with the overlay merged on top."""
    cfg = load_pipeline_config(default, pipeline_config)
    try:
        fr = cfg["frames"]
        t = [float(v) for v in fr["T_b_l"]["translation_m"]]
        q = [float(v) for v in fr["T_b_l"]["quaternion_wxyz"]]
        p = [float(v) for v in fr["p_b_g_m"]]
        names = {k: str(fr[k]) for k in ("world", "body", "lidar", "gnss")}
    except (KeyError, TypeError, ValueError) as exc:
        raise AssetError(f"{pipeline_config}: frames: {exc}") from None
    if len(t) != 3 or len(q) != 4 or len(p) != 3:
        raise AssetError(f"{pipeline_config}: frames: T_b_l or p_b_g_m has the wrong length")
    if abs(math.sqrt(sum(v * v for v in q)) - 1.0) > 1e-6:
        raise AssetError(f"{pipeline_config}: frames.T_b_l.quaternion_wxyz: norm != 1 within 1e-6")
    return Extrinsics((t[0], t[1], t[2]), (q[0], q[1], q[2], q[3]), (p[0], p[1], p[2]), names)


def _fmt(v: float) -> str:
    return repr(float(v))  # shortest round-trip text, exact in the SDF


def robot_context(
    profile: NoiseProfile,
    ext: Extrinsics,
    robot: dict,
    profile_path: str,
    pipeline_path: str,
    lidar_gz_sigma_m: float | None = None,
) -> dict[str, Any]:
    """Template variables; numbers are pre-formatted so the SDF is exact and reproducible."""
    kin = profile.odom.kinematics_true
    body = robot["body"]
    wheel = robot["wheel"]
    rpy = quat_wxyz_to_rpy(list(ext.q_b_l_wxyz))
    v_half = math.radians(robot["lidar"]["vertical_fov_deg"]) / 2.0
    n_h = int(robot["lidar"]["horizontal_samples"])
    caster_z = -(kin.wheel_radius_m - robot["caster"]["radius_m"])
    n_wheels = 2
    total_mass = body["mass_kg"] + n_wheels * wheel["mass_kg"]
    return {
        "profile_path": profile_path,
        "pipeline_path": pipeline_path,
        "robot": robot,
        "frames": ext.frames,
        "kinematics": {"wheel_radius_m": _fmt(kin.wheel_radius_m), "track_m": _fmt(kin.track_m)},
        "imu": profile.imu,
        "imu_noise": {k: _fmt(v) for k, v in imu_discrete_noise(profile).items()},
        "body_inertia": {
            k: _fmt(v) for k, v in box_inertia(body["mass_kg"], *body["size_m"]).items()
        },
        "wheel_inertia": {
            k: _fmt(v)
            for k, v in cylinder_inertia(
                wheel["mass_kg"], kin.wheel_radius_m, wheel["width_m"]
            ).items()
        },
        "wheels": [
            {"name": "left_wheel", "joint": LEFT_JOINT, "y": _fmt(kin.track_m / 2.0)},
            {"name": "right_wheel", "joint": RIGHT_JOINT, "y": _fmt(-kin.track_m / 2.0)},
        ],
        "casters": [
            {"name": "front_caster", "x": _fmt(robot["caster"]["offset_x_m"]), "z": _fmt(caster_z)},
            {"name": "rear_caster", "x": _fmt(-robot["caster"]["offset_x_m"]), "z": _fmt(caster_z)},
        ],
        "half_pi": _fmt(math.pi / 2.0),
        "lidar_pose": [_fmt(v) for v in (*ext.t_b_l_m, *rpy)],
        "lidar": {
            "rate_hz": _fmt(1.0 / profile.lidar.scan_period_s),
            "min_range_m": _fmt(profile.lidar.min_range_m),
            "range_sigma_m": _fmt(gz_range_sigma(profile, lidar_gz_sigma_m)),
            "h_min_rad": _fmt(-math.pi),
            "h_max_rad": _fmt(math.pi - 2.0 * math.pi / n_h),  # no duplicate column at +-pi
            "v_min_rad": _fmt(-v_half),
            "v_max_rad": _fmt(v_half),
        },
        "p_b_g": [_fmt(v) for v in ext.p_b_g_m],
        "gnss_rate_hz": _fmt(profile.gnss.rate_hz),
        "wheel_normal_force_n": _fmt(total_mass * GRAVITY_MPS2 / n_wheels),
    }


def render_robot(context: dict[str, Any], template: Path = TEMPLATE) -> str:
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(str(template.parent)),
        undefined=jinja2.StrictUndefined,
        trim_blocks=True,
        lstrip_blocks=True,
        keep_trailing_newline=True,
        autoescape=False,  # noqa: S701 - SDF from trusted repo files, not HTML
    )
    return env.get_template(template.name).render(**context)


def bridge_config() -> list[dict[str, str]]:
    """``parameter_bridge`` YAML entries; the gz and ROS topic names are the same."""
    return [
        {
            "ros_topic_name": topic,
            "gz_topic_name": topic,
            "ros_type_name": ros_type,
            "gz_type_name": gz_type,
            "direction": direction,
        }
        for topic, ros_type, gz_type, direction in BRIDGE_TOPICS
    ]


def tf_static_transforms(ext: Extrinsics) -> list[dict[str, Any]]:
    """``T_b_l`` and ``p_b_g`` (no rotation) for ``mss_sim.tf_static``: one ``/tf_static`` message.

    Quaternions in ROS field order (x, y, z, w), normalised.
    """
    w, x, y, z = ext.q_b_l_wxyz  # Hamilton order (w, x, y, z)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    body = ext.frames["body"]
    return [
        {
            "frame_id": body,
            "child_frame_id": ext.frames["lidar"],
            "translation_m": [float(v) for v in ext.t_b_l_m],
            "quaternion_xyzw": [x / norm, y / norm, z / norm, w / norm],
        },
        {
            "frame_id": body,
            "child_frame_id": ext.frames["gnss"],
            "translation_m": [float(v) for v in ext.p_b_g_m],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
    ]


def generate(
    profile_path: Path,
    pipeline_config: Path,
    out: Path,
    lidar_gz_sigma_m: float | None = None,
    robot_yaml: Path = ROBOT_YAML,
) -> dict[str, Path]:
    """Write robot.sdf, bridge.yaml, tf_static.yaml and assets.yaml under ``out``.

    ``lidar_gz_sigma_m`` overrides the Gazebo LiDAR range noise (render calibration: 0).
    """
    profile = load_profile(profile_path)
    ext = load_extrinsics(pipeline_config)
    robot = load_robot(robot_yaml)
    ctx = robot_context(
        profile, ext, robot, _rel(profile_path), _rel(pipeline_config), lidar_gz_sigma_m
    )
    kin_true = profile.odom.kinematics_true
    out.mkdir(parents=True, exist_ok=True)
    paths = {
        "robot": out / "robot.sdf",
        "bridge": out / "bridge.yaml",
        "tf_static": out / "tf_static.yaml",
        "assets": out / "assets.yaml",
    }
    paths["robot"].write_text(render_robot(ctx), encoding="utf-8", newline="\n")
    _dump(paths["bridge"], bridge_config())
    _dump(paths["tf_static"], {"transforms": tf_static_transforms(ext)})
    meta = {
        "model_name": robot["model_name"],
        "lidar_gz_range_sigma_m": float(ctx["lidar"]["range_sigma_m"]),
        "robot_sdf": "robot.sdf",
        "bridge_yaml": "bridge.yaml",
        # The model frame is the body frame at axle height: spawn it one wheel radius up, wheels
        # touching the floor (a spawn gap makes the robot drop and spoils the static window).
        "spawn_z_m": kin_true.wheel_radius_m,
        "frames": ext.frames,
        "profile": _rel(profile_path),
        "pipeline_config": _rel(pipeline_config),
    }
    _dump(paths["assets"], meta)
    return paths


def _dump(path: Path, data: Any) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8", newline="\n")


def _rel(path: Path) -> str:
    p = Path(path).resolve()
    try:
        return p.relative_to(REPO).as_posix()
    except ValueError:
        return p.name  # never write absolute machine paths into generated files


# ---------------------------------------------------------------- worlds


def _floats(text: str | None, n: int, where: str) -> list[float]:
    vals = [float(v) for v in (text or "").split()]
    if len(vals) != n:
        raise AssetError(f"{where}: expected {n} numbers, got {text!r}")
    return vals


def _pose_xyz(elem: ET.Element, where: str) -> np.ndarray:
    """Translation of an element's ``<pose>``; rotation must be zero (axis-aligned worlds)."""
    pose = elem.find("pose")
    if pose is None:
        return np.zeros(3)
    v = _floats(pose.text, 6, f"{where}/pose")
    if any(abs(a) > 0.0 for a in v[3:]):
        raise AssetError(f"{where}/pose: rotated boxes are not supported, got {pose.text!r}")
    return np.array(v[:3])


@dataclass(frozen=True)
class BoxFace:
    model: str
    axis: int  # 0 x, 1 y, 2 z
    coord_m: float  # the face lies on the plane p[axis] = coord_m


def world_box_faces(world_sdf: Path) -> list[BoxFace]:
    """All faces of the axis-aligned collision boxes of the world's models."""
    root = ET.parse(world_sdf).getroot()
    world = root.find("world")
    if world is None:
        raise AssetError(f"{world_sdf}: no <world>")
    faces: list[BoxFace] = []
    for model in world.findall("model"):
        name = model.get("name", "?")
        p_model = _pose_xyz(model, name)
        for link in model.findall("link"):
            p_link = p_model + _pose_xyz(link, f"{name}/{link.get('name')}")
            for col in link.findall("collision"):
                where = f"{name}/{link.get('name')}/{col.get('name')}"
                size_el = col.find("geometry/box/size")
                if size_el is None:
                    continue
                size = np.array(_floats(size_el.text, 3, where))
                c = p_link + _pose_xyz(col, where)
                for axis in range(3):
                    for sign in (-1.0, 1.0):
                        faces.append(BoxFace(name, axis, float(c[axis] + sign * size[axis] / 2.0)))
    return faces


def load_sidecar(path: Path) -> dict:
    """``<world>.planes.yaml``: ``world``, ``enu_origin_w_m``, ``spawn``, ``planes``."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise AssetError(f"{path}: expected a mapping")
    for key in ("world", "enu_origin_w_m", "spawn", "planes"):
        if key not in data:
            raise AssetError(f"{path}: {key}: missing")
    for key in ("x_m", "y_m", "yaw_rad"):
        v = data["spawn"].get(key) if isinstance(data["spawn"], dict) else None
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise AssetError(f"{path}: spawn.{key}: expected a number, got {v!r}")
    if not isinstance(data["planes"], list) or not data["planes"]:
        raise AssetError(f"{path}: planes: expected a non-empty list")
    return data


def _plane_axis(n: list[float], where: str) -> tuple[int, float]:
    """Axis index and sign of an axis-aligned unit normal."""
    v = np.asarray(n, dtype=float)
    if v.shape != (3,):
        raise AssetError(f"{where}.n: expected [x, y, z], got {n!r}")
    axis = int(np.argmax(np.abs(v)))
    if abs(abs(v[axis]) - 1.0) > _UNIT_TOL or np.sum(np.abs(v)) - abs(v[axis]) > _UNIT_TOL:
        raise AssetError(f"{where}.n: must be an axis-aligned unit vector, got {n!r}")
    return axis, float(np.sign(v[axis]))


def check_world(world_sdf: Path, sidecar: Path) -> list[str]:
    """Errors (empty when consistent): each plane lies on a box face within ``PLANE_TOL_M``."""
    faces = world_box_faces(world_sdf)
    data = load_sidecar(sidecar)
    errors = []
    if data["world"] != world_sdf.stem:
        errors.append(f"{sidecar.name}: world {data['world']!r} != {world_sdf.stem!r}")
    if [float(v) for v in data["enu_origin_w_m"]] != [0.0, 0.0, 0.0]:
        errors.append(f"{sidecar.name}: enu_origin_w_m must be [0, 0, 0] (ENU frame = world frame)")
    for i, plane in enumerate(data["planes"]):
        where = f"{sidecar.name}: planes[{i}]"
        axis, sign = _plane_axis(plane["n"], where)
        coord = sign * float(plane["d"])  # n . p = d with n = sign * e_axis
        name = str(plane.get("name", ""))
        # The face must belong to the named model (or its parts, e.g. room_wall_west_north).
        hits = [
            f
            for f in faces
            if f.axis == axis
            and abs(f.coord_m - coord) <= PLANE_TOL_M
            and (f.model == name or f.model.startswith(name + "_"))
        ]
        if not hits:
            errors.append(
                f"{where} ({name or '?'}): no face of model {name!r} at {'xyz'[axis]} = {coord}"
            )
    return errors


def world_spherical_lla(world_sdf: Path) -> tuple[float, float, float]:
    """``(lat_deg, lon_deg, alt_m)`` of the world's ``<spherical_coordinates>``."""
    world = ET.parse(world_sdf).getroot().find("world")
    sc = world.find("spherical_coordinates") if world is not None else None
    if sc is None:
        raise AssetError(f"{world_sdf}: no <spherical_coordinates>")
    vals = []
    for tag in ("latitude_deg", "longitude_deg", "elevation"):
        el = sc.find(tag)
        if el is None or el.text is None:
            raise AssetError(f"{world_sdf}: spherical_coordinates/{tag}: missing")
        vals.append(float(el.text))
    return vals[0], vals[1], vals[2]


def check_worlds(worlds_dir: Path = WORLDS) -> list[str]:
    errors = []
    sdfs = sorted(worlds_dir.glob("*.sdf"))
    if not sdfs:
        errors.append(f"no worlds in {worlds_dir}")
    for sdf in sdfs:
        sidecar = sdf.with_suffix(".planes.yaml")
        if not sidecar.exists():
            errors.append(f"{sdf.name}: missing sidecar {sidecar.name}")
            continue
        errors += check_world(sdf, sidecar)
    return errors


# ---------------------------------------------------------------- CLI


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--profile", type=Path, help="noise profile, sim/noise/<profile>.yaml")
    ap.add_argument(
        "--pipeline-config", type=Path, help="pipeline overlay, config/sim/<world>.yaml"
    )
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output dir (build artefact)")
    ap.add_argument("--check-worlds", action="store_true", help="check worlds vs planes sidecars")
    ap.add_argument("--robot", type=Path, default=ROBOT_YAML, help="robot description YAML")
    ap.add_argument(
        "--lidar-gz-sigma",
        type=float,
        default=None,
        help="Gazebo LiDAR range noise, m (render-model calibration: 0)",
    )
    args = ap.parse_args(argv)
    if args.check_worlds:
        errors = check_worlds()
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        if errors:
            return 1
        print("worlds OK")
        if args.profile is None:
            return 0
    if args.profile is None or args.pipeline_config is None:
        ap.error("--profile and --pipeline-config are required to generate assets")
    try:
        paths = generate(
            args.profile, args.pipeline_config, args.out, args.lidar_gz_sigma, args.robot
        )
    except AssetError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    for p in paths.values():
        print(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
