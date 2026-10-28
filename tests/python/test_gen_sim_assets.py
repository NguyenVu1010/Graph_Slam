"""tools/gen_sim_assets.py: robot SDF from the template, bridge config, world/planes consistency."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

import gen_sim_assets as gsa
from mss_sim.profile import load_profile

REPO = Path(__file__).resolve().parents[2]
PROFILE = REPO / "sim/noise/default.yaml"
OVERLAY = REPO / "config/sim/warehouse.yaml"


@pytest.fixture(scope="module")
def assets(tmp_path_factory: pytest.TempPathFactory) -> dict[str, Path]:
    return gsa.generate(PROFILE, OVERLAY, tmp_path_factory.mktemp("assets"))


@pytest.fixture(scope="module")
def model(assets: dict[str, Path]) -> ET.Element:
    root = ET.parse(assets["robot"]).getroot()
    m = root.find("model")
    assert m is not None
    return m


def _f(elem: ET.Element, path: str) -> float:
    found = elem.find(path)
    assert found is not None and found.text is not None, path
    return float(found.text)


def _sensor(model: ET.Element, sensor_type: str) -> ET.Element:
    found = model.find(f"link/sensor[@type='{sensor_type}']")
    assert found is not None, sensor_type
    return found


def test_renders_and_parses(model: ET.Element) -> None:
    assert model.get("name") == "robot"
    assert {link.get("name") for link in model.findall("link")} == {
        "imu_link",
        "left_wheel",
        "right_wheel",
    }
    assert {j.get("name") for j in model.findall("joint")} == {
        "left_wheel_joint",
        "right_wheel_joint",
    }


def test_kinematics_from_kinematics_true(model: ET.Element) -> None:
    kin = load_profile(PROFILE).odom.kinematics_true
    believed = load_profile(PROFILE).odom.kinematics_believed
    dd = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    assert dd is not None
    assert _f(dd, "wheel_radius") == kin.wheel_radius_m != believed.wheel_radius_m
    assert _f(dd, "wheel_separation") == kin.track_m != believed.track_m
    assert _f(dd, "max_linear_velocity") == 1.0
    assert _f(dd, "max_linear_acceleration") == 0.5
    assert _f(dd, "max_angular_velocity") == 1.0
    ys = sorted(_floats(link.find("pose"))[1] for link in model.findall("link")[1:])
    assert ys == [-kin.track_m / 2, kin.track_m / 2]
    for link in model.findall("link")[1:]:
        # Sphere contact: one point in the wheel mid-plane, so the contact track is the true track.
        assert _f(link, "collision/geometry/sphere/radius") == kin.wheel_radius_m
        assert _f(link, "visual/geometry/cylinder/radius") == kin.wheel_radius_m
    for joint in model.findall("joint"):
        assert _f(joint, "axis/dynamics/damping") == 0.1
        assert _f(joint, "axis/dynamics/friction") == 0.2
        assert _f(joint, "axis/limit/effort") > 0


def _floats(elem: ET.Element | None) -> list[float]:
    assert elem is not None and elem.text is not None
    return [float(v) for v in elem.text.split()]


def test_lidar_pose_equals_overlay_t_b_l(model: ET.Element) -> None:
    cfg = yaml.safe_load(OVERLAY.read_text())["mss_node"]["ros__parameters"]["frames"]
    w, x, y, z = cfg["T_b_l"]["quaternion_wxyz"]  # Hamilton order (w, x, y, z)
    r_expected = Rotation.from_quat([x, y, z, w]).as_matrix()  # scipy (x, y, z, w), normalises
    pose = _floats(_sensor(model, "gpu_lidar").find("pose"))
    assert pose[:3] == cfg["T_b_l"]["translation_m"]
    r_sdf = gsa.rpy_to_matrix((pose[3], pose[4], pose[5]))
    np.testing.assert_allclose(r_sdf, r_expected, atol=1e-9, rtol=0)
    assert math.degrees(pose[4]) == pytest.approx(2.0, abs=1e-5)


def test_imu_discrete_stddev_is_density_times_sqrt_rate(model: ET.Element) -> None:
    imu = load_profile(PROFILE).imu
    sensor = _sensor(model, "imu")
    assert _f(sensor, "update_rate") == imu.rate_hz == 200.0
    for axis in "xyz":
        gyro = sensor.find(f"imu/angular_velocity/{axis}/noise")
        acc = sensor.find(f"imu/linear_acceleration/{axis}/noise")
        assert gyro is not None and acc is not None
        assert _f(gyro, "stddev") == pytest.approx(
            imu.gyro_noise_density * math.sqrt(200), rel=1e-12
        )
        assert _f(acc, "stddev") == pytest.approx(imu.acc_noise_density * math.sqrt(200), rel=1e-12)
        assert _f(gyro, "dynamic_bias_stddev") == imu.gyro_random_walk
        assert _f(acc, "dynamic_bias_stddev") == imu.acc_random_walk
        assert _f(gyro, "dynamic_bias_correlation_time") == imu.bias_corr_time_s
        assert _f(gyro, "bias_stddev") == imu.gyro_bias_sigma_radps
        assert _f(acc, "bias_stddev") == imu.acc_bias_sigma_mps2


def test_lidar_and_gnss_sensors(model: ET.Element) -> None:
    prof = load_profile(PROFILE)
    lidar = _sensor(model, "gpu_lidar")
    assert _f(lidar, "update_rate") == 10.0
    assert _f(lidar, "lidar/scan/horizontal/samples") == 720
    assert _f(lidar, "lidar/scan/vertical/samples") == 16
    assert math.degrees(_f(lidar, "lidar/scan/vertical/max_angle")) == pytest.approx(15.0)
    assert math.degrees(_f(lidar, "lidar/scan/vertical/min_angle")) == pytest.approx(-15.0)
    step = _f(lidar, "lidar/scan/horizontal/max_angle") - _f(
        lidar, "lidar/scan/horizontal/min_angle"
    )
    assert math.degrees(step / 719) == pytest.approx(0.5)
    assert _f(lidar, "lidar/range/min") == prof.lidar.min_range_m == 0.5
    assert _f(lidar, "lidar/range/max") == 100.0
    # The declared sigma exactly: the renderer's own error is modelled by the noise report.
    assert _f(lidar, "lidar/noise/stddev") == prof.lidar.range_sigma_m
    gnss = _sensor(model, "navsat")
    assert _f(gnss, "update_rate") == prof.gnss.rate_hz == 5.0
    assert _floats(gnss.find("pose"))[:3] == [-0.20, 0.10, 0.60]
    assert gnss.find("navsat") is None  # no noise: the offline degrader adds it
    assert lidar.findtext("gz_frame_id") == "lidar_link"
    assert gnss.findtext("gz_frame_id") == "gnss_link"
    assert _sensor(model, "imu").findtext("gz_frame_id") == "imu_link"


def test_body_inertia_matches_box_formula(model: ET.Element) -> None:
    robot = gsa.load_robot()
    m = robot["body"]["mass_kg"]
    x, y, z = robot["body"]["size_m"]
    inertial = model.find("link[@name='imu_link']/inertial")
    assert inertial is not None
    assert _f(inertial, "mass") == m == 20.0
    assert _f(inertial, "inertia/ixx") == pytest.approx(m * (y * y + z * z) / 12, rel=1e-12)
    assert _f(inertial, "inertia/iyy") == pytest.approx(m * (x * x + z * z) / 12, rel=1e-12)
    assert _f(inertial, "inertia/izz") == pytest.approx(m * (x * x + y * y) / 12, rel=1e-12)


def test_plugins_present(model: ET.Element) -> None:
    names = {p.get("name") for p in model.findall("plugin")}
    assert names == {
        "gz::sim::systems::DiffDrive",
        "gz::sim::systems::JointStatePublisher",
        "gz::sim::systems::WheelSlip",
        "gz::sim::systems::OdometryPublisher",
    }
    odom = model.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert odom is not None
    assert odom.findtext("odom_topic") == "/ground_truth"
    assert _f(odom, "odom_publish_frequency") == 200.0
    jsp = model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    assert jsp is not None and jsp.findtext("topic") == "/joint_states"


def test_bridge_yaml_lists_exactly_the_raw_topics(assets: dict[str, Path]) -> None:
    entries = yaml.safe_load(assets["bridge"].read_text())
    got = {e["ros_topic_name"]: (e["ros_type_name"], e["direction"]) for e in entries}
    assert got == {
        "/clock": ("rosgraph_msgs/msg/Clock", "GZ_TO_ROS"),
        "/lidar/points": ("sensor_msgs/msg/PointCloud2", "GZ_TO_ROS"),
        "/imu/data": ("sensor_msgs/msg/Imu", "GZ_TO_ROS"),
        "/gnss/fix": ("sensor_msgs/msg/NavSatFix", "GZ_TO_ROS"),
        "/joint_states": ("sensor_msgs/msg/JointState", "GZ_TO_ROS"),
        "/ground_truth": ("nav_msgs/msg/Odometry", "GZ_TO_ROS"),
        "/cmd_vel": ("geometry_msgs/msg/Twist", "ROS_TO_GZ"),
    }
    assert len(entries) == len(got)


def test_tf_static_transforms(assets: dict[str, Path]) -> None:
    tfs = yaml.safe_load(assets["tf_static"].read_text())["transforms"]
    by_child = {t["child_frame_id"]: t for t in tfs}
    assert sorted(by_child) == ["gnss_link", "lidar_link"]
    assert {t["frame_id"] for t in tfs} == {"imu_link"}
    lidar = by_child["lidar_link"]
    assert lidar["translation_m"] == [0.10, 0.0, 0.50]
    q = lidar["quaternion_xyzw"]  # ROS field order (x, y, z, w)
    assert math.sqrt(sum(v * v for v in q)) == pytest.approx(1.0, abs=1e-15)
    assert q[1] == pytest.approx(0.01745241, abs=1e-6) and q[3] == pytest.approx(
        0.9998477, abs=1e-6
    )
    assert by_child["gnss_link"]["translation_m"] == [-0.2, 0.1, 0.6]
    assert by_child["gnss_link"]["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]


def test_assets_meta_has_no_absolute_paths(assets: dict[str, Path]) -> None:
    meta = yaml.safe_load(assets["assets"].read_text())
    assert meta["spawn_z_m"] == 0.10  # wheels touch the floor: no drop at start
    assert meta["profile"] == "sim/noise/default.yaml"
    text = assets["robot"].read_text()
    assert str(REPO) not in text and str(REPO.as_posix()) not in text


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _quat_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation of a unit Hamilton quaternion (w, x, y, z), written out by hand."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def test_quat_rpy_round_trip_general() -> None:
    """SDF convention, independent of scipy: R(q) == Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    rng = np.random.default_rng(7)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)  # Hamilton (w, x, y, z)
        roll, pitch, yaw = gsa.quat_wxyz_to_rpy(list(q))
        expected = _quat_matrix(*q)
        np.testing.assert_allclose(_rz(yaw) @ _ry(pitch) @ _rx(roll), expected, atol=1e-9, rtol=0)
        np.testing.assert_allclose(gsa.rpy_to_matrix((roll, pitch, yaw)), expected, atol=1e-9)


def test_bad_robot_value_rejected(tmp_path: Path) -> None:
    robot = yaml.safe_load(gsa.ROBOT_YAML.read_text())
    robot["wheel"]["mass_kg"] = -1
    f = tmp_path / "robot.yaml"
    f.write_text(yaml.safe_dump(robot))
    with pytest.raises(gsa.AssetError, match=r"wheel\.mass_kg"):
        gsa.load_robot(f)


def test_cli_writes_default_names(tmp_path: Path) -> None:
    out = tmp_path / "a"
    argv = ["--profile", str(PROFILE), "--pipeline-config", str(OVERLAY), "--out", str(out)]
    assert gsa.main(argv) == 0
    assert sorted(p.name for p in out.iterdir()) == [
        "assets.yaml",
        "bridge.yaml",
        "robot.sdf",
        "tf_static.yaml",
    ]


def test_lidar_gz_sigma_override(tmp_path: Path) -> None:
    paths = gsa.generate(PROFILE, OVERLAY, tmp_path, lidar_gz_sigma_m=0.0)
    lidar = ET.parse(paths["robot"]).getroot().find("model/link/sensor[@type='gpu_lidar']")
    assert lidar is not None and _f(lidar, "lidar/noise/stddev") == 0.0
    assert yaml.safe_load(paths["assets"].read_text())["lidar_gz_range_sigma_m"] == 0.0
    with pytest.raises(gsa.AssetError, match="lidar-gz-sigma"):
        gsa.gz_range_sigma(load_profile(PROFILE), -1.0)


def test_wheel_collision_shape_validated(tmp_path: Path) -> None:
    robot = yaml.safe_load(gsa.ROBOT_YAML.read_text())
    robot["wheel"]["collision_shape"] = "box"
    f = tmp_path / "robot.yaml"
    f.write_text(yaml.safe_dump(robot))
    with pytest.raises(gsa.AssetError, match="collision_shape"):
        gsa.load_robot(f)
    robot["wheel"]["collision_shape"] = "cylinder"
    f.write_text(yaml.safe_dump(robot))
    paths = gsa.generate(PROFILE, OVERLAY, tmp_path / "out", robot_yaml=f)
    root = ET.parse(paths["robot"]).getroot()
    assert root.find("model/link[@name='left_wheel']/collision/geometry/cylinder") is not None
