"""bag_to_mssdata: bridged MCAP bag -> flat binary dataset for the C++ bench."""

import json
import os
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from conftest import _ORIGIN_LLA, _REPO

from bag_to_mssdata import (
    GNSS_DTYPE,
    GT_DTYPE,
    IMU_DTYPE,
    LIDAR_HEADER_DTYPE,
    LIDAR_POINT_DTYPE,
    ODOM_DTYPE,
    ConvertReport,
    _sorted_unique,
    convert,
)
from mss_sim import synth
from mss_sim.bag_io import TYPESTORE, BagWriter, CloudArrays, arrays_to_cloud, make_header
from mss_sim.bridge import BridgeReport
from mss_sim.geodesy import lla_to_enu
from mss_sim.synth import SynthInfo

_WAREHOUSE = _REPO / "config/sim/warehouse.yaml"
_N_POINTS = 16 * 720
_CLI = _REPO / "tools/bag_to_mssdata.py"


@pytest.fixture(scope="module")
def converted(
    bridged_bag: tuple[Path, BridgeReport], tmp_path_factory: pytest.TempPathFactory
) -> tuple[Path, ConvertReport]:
    out = tmp_path_factory.mktemp("mssdata") / "ds"
    report = convert(bridged_bag[0] / "bag", out, pipeline_config=_WAREHOUSE)
    return out, report


def test_record_sizes_match_declared_layout() -> None:
    assert IMU_DTYPE.itemsize == struct.calcsize("<q3d3d") == 56
    assert GNSS_DTYPE.itemsize == struct.calcsize("<q3d9db") == 105
    assert ODOM_DTYPE.itemsize == struct.calcsize("<qdd") == 24
    assert GT_DTYPE.itemsize == struct.calcsize("<q3d4d") == 64
    assert LIDAR_HEADER_DTYPE.itemsize == struct.calcsize("<qI") == 12
    assert LIDAR_POINT_DTYPE.itemsize == struct.calcsize("<4fIH") == 22


def test_index_keys_and_counts(
    converted: tuple[Path, ConvertReport], bridged_bag: tuple[Path, BridgeReport]
) -> None:
    out, report = converted
    index = json.loads((out / "index.json").read_text(encoding="utf-8"))
    assert set(index) == {
        "version",
        "topics",
        "origin_lla",
        "T_b_l",
        "p_b_g_m",
        "body_frame",
        "lidar_frame",
        "gnss_cov_filled_from_sigma",
    }
    assert index["version"] == 1
    assert set(index["topics"]) == {"imu", "gnss", "odom", "gt", "lidar"}
    assert index["topics"]["lidar"] == {"dir": "lidar", "count": 60}
    assert index["topics"]["imu"] == {
        "file": "imu.bin",
        "count": bridged_bag[1].n_msgs["/imu/data"],
    }
    assert set(index["T_b_l"]) == {"translation_m", "quaternion_wxyz"}
    assert index["p_b_g_m"] == [-0.20, 0.10, 0.60]
    assert index["body_frame"] == "base_link"
    assert index["lidar_frame"] == "lidar_link"
    assert index["gnss_cov_filled_from_sigma"] is False
    assert report.n == {k: v["count"] for k, v in index["topics"].items()}


def test_file_sizes_match_counts(converted: tuple[Path, ConvertReport]) -> None:
    out, report = converted
    for name, dtype in (
        ("imu", IMU_DTYPE),
        ("gnss", GNSS_DTYPE),
        ("odom", ODOM_DTYPE),
        ("gt", GT_DTYPE),
    ):
        assert (out / f"{name}.bin").stat().st_size == report.n[name] * dtype.itemsize


def test_gt_count_and_quaternion_order(
    converted: tuple[Path, ConvertReport], bridged_bag: tuple[Path, BridgeReport]
) -> None:
    out, _ = converted
    gt = np.fromfile(out / "gt.bin", dtype=GT_DTYPE)
    assert len(gt) == bridged_bag[1].n_msgs["/ground_truth"] == 1201  # 6 s at 200 Hz + 1
    q = gt["q_wxyz"]
    assert np.allclose(np.linalg.norm(q, axis=1), 1.0)
    assert np.all(q[:, 0] > 0.0)  # synthetic yaw stays small: w = cos(yaw/2) > 0
    assert np.allclose(q[:, 1:3], 0.0)  # pure yaw: x = y = 0, z carries the rotation


def test_odom_fields(converted: tuple[Path, ConvertReport]) -> None:
    out, _ = converted
    odom = np.fromfile(out / "odom.bin", dtype=ODOM_DTYPE)
    assert len(odom) > 0
    assert np.all(np.isfinite(odom["v"])) and np.all(np.isfinite(odom["omega"]))


def test_t_strictly_increasing(converted: tuple[Path, ConvertReport]) -> None:
    out, _ = converted
    for name, dtype in (
        ("imu", IMU_DTYPE),
        ("gnss", GNSS_DTYPE),
        ("odom", ODOM_DTYPE),
        ("gt", GT_DTYPE),
    ):
        t = np.fromfile(out / f"{name}.bin", dtype=dtype)["t"]
        assert np.all(np.diff(t) > 0), name
    starts = [
        np.fromfile(f, dtype=LIDAR_HEADER_DTYPE, count=1)["t_start"][0]
        for f in sorted((out / "lidar").glob("*.bin"))
    ]
    assert np.all(np.diff(starts) > 0)


def test_lidar_files(converted: tuple[Path, ConvertReport]) -> None:
    out, _ = converted
    files = sorted((out / "lidar").glob("*.bin"))
    assert [f.name for f in files] == [f"{i:06d}.bin" for i in range(60)]
    raw = files[0].read_bytes()
    header = np.frombuffer(raw, dtype=LIDAR_HEADER_DTYPE, count=1)
    assert int(header["n"][0]) == _N_POINTS
    assert len(raw) == 12 + 22 * _N_POINTS
    pts = np.frombuffer(raw, dtype=LIDAR_POINT_DTYPE, offset=12)
    row0 = pts[:720]
    assert np.isfinite(row0["xyz"][:, 0]).sum() > 10
    assert np.all(np.diff(row0["t_offset"].astype(np.int64)) > 0)
    assert np.array_equal(pts["ring"][::720], np.arange(16))  # column 0 down the rings


def test_gnss_origin_defaults_to_first_fix(converted: tuple[Path, ConvertReport]) -> None:
    out, report = converted
    gnss = np.fromfile(out / "gnss.bin", dtype=GNSS_DTYPE)
    assert np.allclose(gnss["p_e_g"][0], 0.0, atol=1e-6)
    index = json.loads((out / "index.json").read_text(encoding="utf-8"))
    assert index["origin_lla"] == list(report.origin_lla)
    assert gnss["cov"][0].reshape(3, 3)[0, 0] > 0.0


def test_gnss_explicit_origin(
    converted: tuple[Path, ConvertReport],
    bridged_bag: tuple[Path, BridgeReport],
    tmp_path: Path,
) -> None:
    out_default, report = converted
    explicit = (_ORIGIN_LLA[0], _ORIGIN_LLA[1], _ORIGIN_LLA[2])
    out = tmp_path / "ds"
    rep = convert(bridged_bag[0] / "bag", out, origin_lla=explicit, pipeline_config=_WAREHOUSE)
    assert rep.origin_lla == explicit
    first_default = np.fromfile(out_default / "gnss.bin", dtype=GNSS_DTYPE)["p_e_g"][0]
    first = np.fromfile(out / "gnss.bin", dtype=GNSS_DTYPE)["p_e_g"][0]
    offset = lla_to_enu(np.array(report.origin_lla), explicit)
    assert np.allclose(first, first_default + offset, atol=1e-6)
    assert np.linalg.norm(first) > 1.0  # the first fix is noisy and not at the explicit origin


def test_non_empty_out_dir_is_rejected(
    bridged_bag: tuple[Path, BridgeReport], tmp_path: Path
) -> None:
    (tmp_path / "junk.txt").write_text("x", encoding="utf-8")
    with pytest.raises(ValueError, match="not empty"):
        convert(bridged_bag[0] / "bag", tmp_path, pipeline_config=_WAREHOUSE)


def test_raw_bag_is_rejected(raw_bag: tuple[Path, SynthInfo], tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="run the bridge first"):
        convert(raw_bag[0], tmp_path / "ds", pipeline_config=_WAREHOUSE)


def test_cli_prints_json_report(bridged_bag: tuple[Path, BridgeReport], tmp_path: Path) -> None:
    out = tmp_path / "ds"
    proc = subprocess.run(
        [
            sys.executable,
            str(_CLI),
            str(bridged_bag[0] / "bag"),
            str(out),
            "--pipeline-config",
            str(_WAREHOUSE),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    report = json.loads(proc.stdout.strip().splitlines()[-1])
    assert set(report) == {"n", "origin_lla", "gnss_cov_filled_from_sigma"}
    assert report["n"]["lidar"] == 60
    assert (out / "index.json").is_file()


def test_cli_error_exit_code(raw_bag: tuple[Path, SynthInfo], tmp_path: Path) -> None:
    proc = subprocess.run(
        [sys.executable, str(_CLI), str(raw_bag[0]), str(tmp_path / "d")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 2
    assert "run the bridge first" in proc.stderr


def _bag_with_unknown_cov(path: Path) -> Path:
    """Minimal bag: one NavSatFix with ``position_covariance_type == 0`` and a /tf_static."""
    ty = TYPESTORE.types
    with BagWriter(path) as w:
        w.add_topic("/gnss/fix", "sensor_msgs/msg/NavSatFix")
        w.add_topic("/tf_static", "tf2_msgs/msg/TFMessage")
        w.write("/gnss/fix", 1_000, synth._gnss(1_000, np.array(_ORIGIN_LLA)))
        tf = ty["geometry_msgs/msg/TransformStamped"](
            header=make_header(0, "base_link"),
            child_frame_id="lidar_link",
            transform=ty["geometry_msgs/msg/Transform"](
                translation=ty["geometry_msgs/msg/Vector3"](x=0.1, y=0.0, z=0.5),
                rotation=ty["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        w.write("/tf_static", 0, ty["tf2_msgs/msg/TFMessage"](transforms=[tf]))
    return path


def test_cov_filled_from_sigma_when_type_unknown(tmp_path: Path) -> None:
    bag = _bag_with_unknown_cov(tmp_path / "bag")
    out = tmp_path / "ds"
    report = convert(bag, out, pipeline_config=_WAREHOUSE)
    assert report.gnss_cov_filled_from_sigma is True
    cov = np.fromfile(out / "gnss.bin", dtype=GNSS_DTYPE)["cov"][0].reshape(3, 3)
    assert np.allclose(cov, np.diag([1.5**2, 1.5**2, 3.0**2]))
    index = json.loads((out / "index.json").read_text(encoding="utf-8"))
    assert index["gnss_cov_filled_from_sigma"] is True
    assert index["T_b_l"]["translation_m"] == [0.1, 0.0, 0.5]
    assert index["T_b_l"]["quaternion_wxyz"] == [1.0, 0.0, 0.0, 0.0]


def test_unknown_cov_without_pipeline_config_is_an_error(tmp_path: Path) -> None:
    bag = _bag_with_unknown_cov(tmp_path / "bag")
    with pytest.raises(ValueError, match="pipeline-config"):
        convert(bag, tmp_path / "ds")


def test_missing_pipeline_config_warns_and_defaults_p_b_g(
    bridged_bag: tuple[Path, BridgeReport], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "ds"
    convert(bridged_bag[0] / "bag", out, origin_lla=_ORIGIN_LLA)
    assert "p_b_g_m defaults to [0, 0, 0]" in capsys.readouterr().err
    index = json.loads((out / "index.json").read_text(encoding="utf-8"))
    assert index["p_b_g_m"] == [0.0, 0.0, 0.0]


def test_sorted_unique_orders_and_keeps_first_of_ties() -> None:
    rec = np.zeros(6, dtype=ODOM_DTYPE)
    rec["t"] = [30, 10, 20, 10, 30, 5]
    rec["v"] = [0, 1, 2, 3, 4, 5]
    out = _sorted_unique(rec)
    assert out["t"].tolist() == [5, 10, 20, 30]
    assert out["v"].tolist() == [5, 1, 2, 0]  # first occurrence of each tied stamp wins


def _scan(stamp: int, marker: float) -> object:
    n = 4
    arrays = CloudArrays(
        xyz=np.full((n, 3), marker, dtype=np.float32),
        intensity=np.zeros(n, np.float32),
        t_ns=np.arange(n, dtype=np.uint32),
        ring=np.zeros(n, np.uint16),
        height=1,
        width=n,
    )
    return arrays_to_cloud(arrays, stamp, "lidar_link")


def test_lidar_reorder_and_duplicate_stamps(tmp_path: Path) -> None:
    ty = TYPESTORE.types
    bag = tmp_path / "bag"
    tf = ty["geometry_msgs/msg/TransformStamped"](
        header=make_header(0, "base_link"),
        child_frame_id="lidar_link",
        transform=ty["geometry_msgs/msg/Transform"](
            translation=ty["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0),
            rotation=ty["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    # Log order: stamp 200 (marker 2), stamp 100 (marker 1), stamp 200 again (marker 9, duplicate).
    with BagWriter(bag) as w:
        w.add_topic("/lidar/points", "sensor_msgs/msg/PointCloud2")
        w.add_topic("/tf_static", "tf2_msgs/msg/TFMessage")
        w.write("/tf_static", 0, ty["tf2_msgs/msg/TFMessage"](transforms=[tf]))
        w.write("/lidar/points", 1_000, _scan(200, 2.0))
        w.write("/lidar/points", 2_000, _scan(100, 1.0))
        w.write("/lidar/points", 3_000, _scan(200, 9.0))
    out = tmp_path / "ds"
    report = convert(bag, out)
    assert report.n["lidar"] == 2
    files = sorted((out / "lidar").iterdir())
    assert [f.name for f in files] == ["000000.bin", "000001.bin"]
    starts, markers = [], []
    for f in files:
        raw = f.read_bytes()
        starts.append(int(np.frombuffer(raw, LIDAR_HEADER_DTYPE, count=1)["t_start"][0]))
        markers.append(float(np.frombuffer(raw, LIDAR_POINT_DTYPE, offset=12)["xyz"][0, 0]))
    assert starts == [100, 200]
    assert markers == [1.0, 2.0]  # the first-logged scan of the duplicated stamp survives


def test_cpp_reader_agrees(converted: tuple[Path, ConvertReport], tmp_path: Path) -> None:
    """Cross-check: the C++ DatasetReader (via mss_bench) replays what Python wrote.

    Runs only when MSS_BENCH_BIN points at a built mss_bench binary.
    """
    bench = os.environ.get("MSS_BENCH_BIN")
    if not bench:
        pytest.skip("MSS_BENCH_BIN is not set (path to mss_bench)")
    assert Path(bench).is_file(), f"MSS_BENCH_BIN={bench} is not a file"
    data, report = converted
    out = tmp_path / "bench"
    proc = subprocess.run(
        [
            bench,
            str(_REPO / "config/default.yaml"),
            str(data),
            "--overlay",
            str(_WAREHOUSE),
            "--out",
            str(out),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    result = json.loads((out / "bench.json").read_text(encoding="utf-8"))
    assert result["records"] == report.n
    assert result["n_imu"] == report.n["imu"]
    assert result["n_gnss"] == report.n["gnss"]
    assert result["n_odom"] == report.n["odom"]
    assert result["n_scans"] + result["n_dropped_scans"] == report.n["lidar"]
    assert result["n_rejected_imu"] == 0
    assert result["n_dropped_imu"] == 0
    assert result["n_dropped_aux"] == 0
    est = (out / "est.tum").read_text(encoding="utf-8").splitlines()
    assert len(est) == result["n_scans"]
