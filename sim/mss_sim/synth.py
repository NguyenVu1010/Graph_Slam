"""Synthetic "raw Gazebo" MCAP bag: clean sensors plus ground truth, no Gazebo or ROS needed.

A robot drives a circle of radius ``speed / omega`` starting at the world origin (body x
forward, z up) inside a closed box. Optionally it stands still for ``static_s`` and then ramps
its speed up over ``ramp_s`` on the same circle (``docs/derivations/pipeline.md`` section 7).
The LiDAR sits at ``T_b_l`` (identity by default) and ray-casts the six box planes from the
pose at the scan stamp. Only the IMU carries noise (white, from the default profile, like
Gazebo's own IMU); everything else is exact.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from mss_sim.bag_io import TYPESTORE, BagWriter, make_header
from mss_sim.geodesy import Lla, enu_to_lla
from mss_sim.profile import load_profile

T0_NS = 1_700_000_000_000_000_000  # epoch scale on purpose: catches float-seconds bugs
GRAVITY_W = np.array([0.0, 0.0, -9.80665])
P_B_G = np.array([-0.20, 0.10, 0.60])  # GNSS antenna in body frame [m]

_NS_PER_S = 1_000_000_000
_GT_HZ, _IMU_HZ, _LIDAR_HZ, _GNSS_HZ, _JOINT_HZ = 200, 200, 10, 5, 50
_RINGS, _COLUMNS = 16, 720
_ELEVATION_DEG = (-15.0, 15.0)  # ring 0 lowest
_DEFAULT_MAX_RANGE_M = 100.0
_INTENSITY = 100.0
_F32 = 7  # sensor_msgs/PointField FLOAT32
_RAW_FIELDS = ("x", "y", "z", "intensity")
_DEFAULT_PROFILE = Path(__file__).resolve().parents[1] / "noise" / "default.yaml"
_N_SPAWN, _SPAWN_IMU = 5, 3  # SeedSequence children: [lidar, gnss, odom, imu, mutate]
_FRAME_WORLD, _FRAME_BODY, _FRAME_LIDAR = "map", "base_link", "lidar_link"


# (translation_m, quaternion_wxyz): T_b_l, LiDAR in body, Hamilton order (w, x, y, z).
_TBL = tuple[tuple[float, float, float], tuple[float, float, float, float]]


@dataclass(frozen=True)
class SynthInfo:
    """What the generator wrote: planes ``(n_w unit normal, d)`` with ``n . p = d``."""

    planes: list[tuple[np.ndarray, float]]
    t0_ns: int
    origin_lla: Lla
    duration_s: float


def _box_planes(half: tuple[float, float, float]) -> list[tuple[np.ndarray, float]]:
    planes = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            n = np.zeros(3)
            n[axis] = sign
            planes.append((n, float(half[axis])))
    return planes


@dataclass(frozen=True)
class _Motion:
    static_s: float
    ramp_s: float
    speed: float  # V, cruise speed [m/s]
    omega: float  # W, cruise yaw rate [rad/s]


def _profile(t_s: np.ndarray, m: _Motion) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Speed profile of pipeline.md section 7 at times ``t_s`` (seconds after T0).

    Returns ``(u, r, a_t)``: ``u = s / V`` (arc length in cruise-seconds), ``r = v / V`` and the
    tangential acceleration ``a_t`` [m/s^2]. Then ``s = V u``, ``psi = W u``, ``v = V r`` and
    ``omega = W r``. With ``static_s = ramp_s = 0`` this gives ``u = t_s`` and ``r = 1`` exactly,
    so the constant circle is reproduced bit for bit.
    """
    tau = t_s - m.static_s
    u = np.zeros_like(tau)
    r = np.zeros_like(tau)
    a_t = np.zeros_like(tau)
    cruise = tau >= m.ramp_s
    u[cruise] = m.ramp_s / 2 + (tau[cruise] - m.ramp_s)
    r[cruise] = 1.0
    ramp = (tau >= 0.0) & ~cruise
    if ramp.any():
        x = np.pi * tau[ramp] / m.ramp_s
        u[ramp] = (tau[ramp] - (m.ramp_s / np.pi) * np.sin(x)) / 2
        r[ramp] = (1.0 - np.cos(x)) / 2
        a_t[ramp] = m.speed * np.pi / (2 * m.ramp_s) * np.sin(x)
    return u, r, a_t


def _pose(t_s: np.ndarray, m: _Motion) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Circle through the origin heading +x: returns (yaw, p_w_b, R_w_b) for times ``t_s``."""
    u, _, _ = _profile(t_s, m)
    yaw = m.omega * u
    radius = m.speed / m.omega
    p = np.stack([radius * np.sin(yaw), radius * (1.0 - np.cos(yaw)), np.zeros_like(t_s)], axis=1)
    rot = Rotation.from_rotvec(np.stack([0 * yaw, 0 * yaw, yaw], axis=1)).as_matrix()
    return yaw, p, rot


def _times(duration_s: float, rate_hz: int, inclusive: bool = False) -> np.ndarray:
    n = int(round(duration_s * rate_hz)) + (1 if inclusive else 0)
    return T0_NS + np.arange(n, dtype=np.int64) * (_NS_PER_S // rate_hz)


def _rel_s(stamps_ns: np.ndarray) -> np.ndarray:
    return (stamps_ns - T0_NS) * 1e-9


def _scan_dirs() -> np.ndarray:
    """Unit ray directions in the LiDAR frame, shape (rings * columns, 3), ring-major."""
    el = np.deg2rad(np.linspace(_ELEVATION_DEG[0], _ELEVATION_DEG[1], _RINGS))[:, None]
    az = np.deg2rad(np.arange(_COLUMNS) * (360.0 / _COLUMNS))[None, :]
    dirs = np.stack(
        [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el) + 0 * az], axis=-1
    )
    return dirs.reshape(-1, 3)


def _raycast(
    origin: np.ndarray,
    rot: np.ndarray,
    planes: list[tuple[np.ndarray, float]],
    dirs_l: np.ndarray,
    max_range_m: float,
) -> np.ndarray:
    """Nearest positive plane hit per ray in the LiDAR frame; ``inf`` x, y, z for non-returns."""
    dirs_w = dirs_l @ rot.T
    best = np.full(len(dirs_w), np.inf)
    for n, d in planes:
        denom = dirs_w @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (d - origin @ n) / denom
        best = np.where((denom != 0) & (t > 0) & (t < best), t, best)
    best[best > max_range_m] = np.inf
    # Only scale rays that hit: inf * 0 would give nan for an exactly-zero direction component.
    xyz = np.full(dirs_l.shape, np.inf)
    hit = np.isfinite(best)
    xyz[hit] = best[hit, None] * dirs_l[hit]
    return xyz


def _raw_cloud(stamp: int, xyz: np.ndarray) -> object:
    rec = np.empty((len(xyz), 4), dtype="<f4")
    rec[:, :3] = xyz
    rec[:, 3] = _INTENSITY
    types = TYPESTORE.types
    fields = [
        types["sensor_msgs/msg/PointField"](name=name, offset=4 * i, datatype=_F32, count=1)
        for i, name in enumerate(_RAW_FIELDS)
    ]
    return types["sensor_msgs/msg/PointCloud2"](
        header=make_header(stamp, _FRAME_LIDAR),
        height=_RINGS,
        width=_COLUMNS,
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * _COLUMNS,
        data=np.frombuffer(rec.tobytes(), dtype=np.uint8),
        is_dense=False,
    )


def _vec3(v: np.ndarray) -> object:
    return TYPESTORE.types["geometry_msgs/msg/Vector3"](x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _identity_quat() -> object:
    # Hamilton (w, x, y, z) = (1, 0, 0, 0).
    return TYPESTORE.types["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0)


def _odometry(stamp: int, yaw: float, p: np.ndarray, speed: float, omega: float) -> object:
    """Ground truth ``T_w_b`` with the body twist (``speed`` along x, ``omega`` about z)."""
    ty = TYPESTORE.types
    point = ty["geometry_msgs/msg/Point"](x=float(p[0]), y=float(p[1]), z=float(p[2]))
    # Pure yaw, Hamilton (w, x, y, z) = (cos(yaw/2), 0, 0, sin(yaw/2)).
    quat = ty["geometry_msgs/msg/Quaternion"](
        x=0.0, y=0.0, z=float(np.sin(yaw / 2)), w=float(np.cos(yaw / 2))
    )
    pose = ty["geometry_msgs/msg/Pose"](position=point, orientation=quat)
    zeros = np.zeros(36)
    twist = ty["geometry_msgs/msg/Twist"](
        linear=_vec3(np.array([speed, 0.0, 0.0])), angular=_vec3(np.array([0.0, 0.0, omega]))
    )
    return ty["nav_msgs/msg/Odometry"](
        header=make_header(stamp, _FRAME_WORLD),
        child_frame_id=_FRAME_BODY,
        pose=ty["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=zeros),
        twist=ty["geometry_msgs/msg/TwistWithCovariance"](twist=twist, covariance=zeros),
    )


def _imu(stamp: int, gyro: np.ndarray, acc: np.ndarray) -> object:
    orientation_cov = np.zeros(9)
    orientation_cov[0] = -1.0  # REP-145: orientation not provided
    return TYPESTORE.types["sensor_msgs/msg/Imu"](
        header=make_header(stamp, _FRAME_BODY),
        orientation=_identity_quat(),
        orientation_covariance=orientation_cov,
        angular_velocity=_vec3(gyro),
        angular_velocity_covariance=np.zeros(9),
        linear_acceleration=_vec3(acc),
        linear_acceleration_covariance=np.zeros(9),
    )


def _gnss(stamp: int, lla: np.ndarray) -> object:
    ty = TYPESTORE.types
    return ty["sensor_msgs/msg/NavSatFix"](
        header=make_header(stamp, "gnss_link"),
        status=ty["sensor_msgs/msg/NavSatStatus"](status=0, service=1),
        latitude=float(lla[0]),
        longitude=float(lla[1]),
        altitude=float(lla[2]),
        position_covariance=np.zeros(9),
        position_covariance_type=0,
    )


def _joint_state(stamp: int, vel: np.ndarray, pos: np.ndarray) -> object:
    return TYPESTORE.types["sensor_msgs/msg/JointState"](
        header=make_header(stamp, ""),
        name=["left_wheel_joint", "right_wheel_joint"],
        position=pos,
        velocity=vel,
        effort=np.zeros(0),
    )


def _tf_static(stamp: int, t_b_l: _TBL | None) -> object:
    ty = TYPESTORE.types
    if t_b_l is None:
        rotation = _identity_quat()  # T_b_l = identity
        translation = np.zeros(3)
    else:
        translation = np.asarray(t_b_l[0], dtype=float)
        w, x, y, z = t_b_l[1]  # Hamilton (w, x, y, z); ROS fields are x, y, z, w
        rotation = ty["geometry_msgs/msg/Quaternion"](x=x, y=y, z=z, w=w)
    transform = ty["geometry_msgs/msg/Transform"](translation=_vec3(translation), rotation=rotation)
    stamped = ty["geometry_msgs/msg/TransformStamped"](
        header=make_header(stamp, _FRAME_BODY), child_frame_id=_FRAME_LIDAR, transform=transform
    )
    return ty["tf2_msgs/msg/TFMessage"](transforms=[stamped])


def write_synthetic_raw_bag(
    out_dir: Path,
    *,
    duration_s: float = 6.0,
    seed: int = 0,
    organized: bool = True,
    box_half_m: tuple[float, float, float] = (10.0, 6.0, 3.0),
    omega_radps: float = 0.3,
    speed_mps: float = 0.5,
    max_range_m: float = _DEFAULT_MAX_RANGE_M,
    origin_lla: Lla,
    static_s: float = 0.0,
    ramp_s: float = 0.0,
    t_b_l: _TBL | None = None,
) -> SynthInfo:
    """Write a raw bag into ``out_dir / "raw"`` and describe it.

    ``static_s`` seconds still, then a speed ramp over ``ramp_s`` seconds to ``speed_mps`` on a
    circle of curvature ``omega_radps / speed_mps`` (pipeline.md section 7). ``t_b_l`` is
    ``(translation_m, quaternion_wxyz)``, Hamilton order; ``None`` means identity. The defaults
    reproduce the constant circle with an identity ``T_b_l`` byte for byte.
    """
    if not organized:
        raise NotImplementedError("only organized (16 x 720) clouds are generated")
    if static_s < 0.0 or ramp_s < 0.0:
        raise ValueError(f"static_s and ramp_s must be >= 0, got {static_s}, {ramp_s}")
    motion = _Motion(static_s, ramp_s, speed_mps, omega_radps)
    profile = load_profile(_DEFAULT_PROFILE)
    kin = profile.odom.kinematics_true
    rng_imu = np.random.default_rng(np.random.SeedSequence(seed).spawn(_N_SPAWN)[_SPAWN_IMU])
    planes = _box_planes(box_half_m)
    dirs_l = _scan_dirs()
    half_track = omega_radps * kin.track_m / 2
    # Cruise wheel speeds [rad/s]; the profile scales them by r = v / V, positions by u = s / V.
    wheel_vel = np.array([speed_mps - half_track, speed_mps + half_track]) / kin.wheel_radius_m
    r_b_l, p_b_l = np.eye(3), np.zeros(3)
    if t_b_l is not None:
        w, x, y, z = t_b_l[1]  # Hamilton (w, x, y, z); scipy wants (x, y, z, w)
        r_b_l = Rotation.from_quat([x, y, z, w]).as_matrix()
        p_b_l = np.asarray(t_b_l[0], dtype=float)

    with BagWriter(out_dir / "raw") as bag:
        for name, typename in (
            ("/ground_truth", "nav_msgs/msg/Odometry"),
            ("/imu/data", "sensor_msgs/msg/Imu"),
            ("/lidar/points", "sensor_msgs/msg/PointCloud2"),
            ("/gnss/fix", "sensor_msgs/msg/NavSatFix"),
            ("/joint_states", "sensor_msgs/msg/JointState"),
            ("/tf_static", "tf2_msgs/msg/TFMessage"),
        ):
            bag.add_topic(name, typename)
        bag.write("/tf_static", T0_NS, _tf_static(T0_NS, t_b_l))

        stamps = _times(duration_s, _GT_HZ, inclusive=True)
        yaw, p, _ = _pose(_rel_s(stamps), motion)
        _, r, _ = _profile(_rel_s(stamps), motion)
        for i, s in enumerate(stamps):
            v, w = speed_mps * r[i], omega_radps * r[i]
            bag.write("/ground_truth", int(s), _odometry(int(s), yaw[i], p[i], v, w))

        stamps = _times(duration_s, _IMU_HZ)
        yaw, _, rot = _pose(_rel_s(stamps), motion)
        _, r, a_t = _profile(_rel_s(stamps), motion)
        # Centripetal kappa v^2 = V W r^2 [-sin, cos, 0], plus a_t [cos, sin, 0] in the ramp.
        a_w = speed_mps * omega_radps * np.stack([-np.sin(yaw), np.cos(yaw), 0 * yaw], axis=1)
        if ramp_s > 0.0 or static_s > 0.0:
            a_w = a_w * (r * r)[:, None]
            a_w += a_t[:, None] * np.stack([np.cos(yaw), np.sin(yaw), 0 * yaw], axis=1)
        acc_b = np.einsum("nji,nj->ni", rot, a_w - GRAVITY_W)  # R_w_b^T (a_w - g_w)
        gyro_b = np.zeros((len(stamps), 3))
        gyro_b[:, 2] = omega_radps * r
        sqrt_rate = np.sqrt(profile.imu.rate_hz)  # discrete sigma = density * sqrt(rate)
        gyro_b += rng_imu.normal(0.0, profile.imu.gyro_noise_density * sqrt_rate, gyro_b.shape)
        acc_b += rng_imu.normal(0.0, profile.imu.acc_noise_density * sqrt_rate, acc_b.shape)
        for i, s in enumerate(stamps):
            bag.write("/imu/data", int(s), _imu(int(s), gyro_b[i], acc_b[i]))

        for s in _times(duration_s, _LIDAR_HZ):
            _, p1, rot1 = _pose(_rel_s(np.array([s])), motion)
            origin, rot_l = p1[0], rot1[0]
            if t_b_l is not None:  # T_w_l = T_w_b * T_b_l
                origin, rot_l = origin + rot_l @ p_b_l, rot_l @ r_b_l
            bag.write(
                "/lidar/points",
                int(s),
                _raw_cloud(int(s), _raycast(origin, rot_l, planes, dirs_l, max_range_m)),
            )

        stamps = _times(duration_s, _GNSS_HZ)
        _, p, rot = _pose(_rel_s(stamps), motion)
        lla = np.asarray(enu_to_lla(p + rot @ P_B_G, origin_lla))  # p_w_g = p_w_b + R_w_b p_b_g
        for i, s in enumerate(stamps):
            bag.write("/gnss/fix", int(s), _gnss(int(s), lla[i]))

        stamps = _times(duration_s, _JOINT_HZ)
        u, r, _ = _profile(_rel_s(stamps), motion)
        for i, s in enumerate(stamps):
            bag.write(
                "/joint_states", int(s), _joint_state(int(s), wheel_vel * r[i], wheel_vel * u[i])
            )

    return SynthInfo(planes=planes, t0_ns=T0_NS, origin_lla=origin_lla, duration_s=duration_s)
