"""Noise report (ADR-011 section 7) on the bridged fixture bag."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from conftest import _REPO, copy_bag_without
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

from mss_eval.noise_measure import expected_dropout, imu_section, lidar_section, ray_residuals
from mss_eval.noise_report import (
    Report,
    Row,
    estimate_gnss_sigma,
    estimate_range_sigma,
    load_planes,
    run,
)
from mss_sim.bridge import BridgeReport
from mss_sim.synth import SynthInfo

_PROFILE = _REPO / "sim/noise/default.yaml"
_BAG_VERSION = 9


def _section(report: Report, name: str):  # noqa: ANN202
    return next(s for s in report.sections if s.name == name)


def _row(report: Report, section: str, name: str) -> Row:
    return next(r for r in _section(report, section).rows if r.name == name)


@pytest.fixture(scope="module")
def planes_file(raw_bag: tuple[Path, SynthInfo], tmp_path_factory: pytest.TempPathFactory) -> Path:
    _, info = raw_bag
    path = tmp_path_factory.mktemp("planes") / "planes.yaml"
    planes = [{"n": [float(v) for v in n], "d": float(d)} for n, d in info.planes]
    path.write_text(yaml.safe_dump(planes))
    return path


@pytest.fixture(scope="module")
def report_dir(
    bridged_bag: tuple[Path, BridgeReport],
    planes_file: Path,
    tmp_path_factory: pytest.TempPathFactory,
) -> tuple[Report, Path]:
    out_dir, _ = bridged_bag
    out = tmp_path_factory.mktemp("noise_report")
    return run(out_dir, _PROFILE, out, planes=planes_file), out


@pytest.fixture(scope="module")
def no_gt_dir(
    bridged_bag: tuple[Path, BridgeReport], tmp_path_factory: pytest.TempPathFactory
) -> Path:
    """Copy of the bridged output without ``/ground_truth`` (and without ``gt.tum``)."""
    src, _ = bridged_bag
    dst = tmp_path_factory.mktemp("no_gt") / "out"
    dst.mkdir()
    shutil.copy(src / "bridge_report.json", dst / "bridge_report.json")
    store = get_typestore(Stores.ROS2_JAZZY)
    with Reader(src / "bag") as reader, Writer(dst / "bag", version=_BAG_VERSION) as writer:
        conns = {}
        for c in reader.connections:
            if c.topic != "/ground_truth":
                conns[c.id] = writer.add_connection(c.topic, c.msgtype, typestore=store)
        keep = [c for c in reader.connections if c.id in conns]
        for conn, t_ns, raw in reader.messages(connections=keep):
            writer.write(conns[conn.id], t_ns, raw)
    return dst


def test_report_writes_html_and_json(report_dir: tuple[Report, Path]) -> None:
    report, out = report_dir
    html = (out / "noise_report.html").read_text(encoding="utf-8")
    assert "<html" in html and "data:image/png;base64," in html
    saved = json.loads((out / "noise_report.json").read_text())
    assert saved["passed"] == report.passed
    assert {s["name"] for s in saved["sections"]} >= {"IMU", "GNSS", "LiDAR", "Wheel odom", "Bag"}
    assert all("figures" not in s for s in saved["sections"])


def test_bag_section_passes(report_dir: tuple[Report, Path]) -> None:
    report, _ = report_dir
    rows = _section(report, "Bag").rows
    gated = [r for r in rows if r.ok is not None]
    assert gated and all(r.ok for r in gated), [r for r in gated if not r.ok]
    assert _row(report, "Bag", "/ground_truth present").ok is True


def test_gnss_sigma_h_within_15_percent(report_dir: tuple[Report, Path]) -> None:
    report, _ = report_dir
    row = _row(report, "GNSS", "sigma_h_m")
    assert row.declared == pytest.approx(1.0)
    assert isinstance(row.measured, float)
    assert abs(row.measured - 1.0) <= 0.15
    assert row.ok is True


def test_imu_without_static_segment_is_informational(report_dir: tuple[Report, Path]) -> None:
    report, _ = report_dir
    rows = [r for r in _section(report, "IMU").rows if r.name.startswith("gyro_sigma")]
    assert len(rows) == 3
    assert all(r.ok is None and "no static segment" in r.note for r in rows)
    assert _section(report, "IMU").figures  # Allan figure is always drawn


def test_lidar_section_on_noise_free_synthetic_bag(report_dir: tuple[Report, Path]) -> None:
    """The synthetic bag has no range noise: measured sigma ~0 is a genuine mismatch."""
    report, _ = report_dir
    dropout = _row(report, "LiDAR", "dropout_fraction")
    assert dropout.ok is True, dropout
    sigma = _row(report, "LiDAR", "range_sigma_m")
    assert sigma.declared == pytest.approx(0.02)
    assert isinstance(sigma.measured, float) and sigma.measured < 0.005
    assert sigma.ok is False
    assert report.passed is False  # the LiDAR mismatch fails the report


def test_range_sigma_estimator_recovers_declared_sigma() -> None:
    rng = np.random.default_rng(3)
    d = rng.normal(0.0, 0.02, 50_000)
    d[:200] = rng.uniform(1.0, 3.0, 200)  # outliers (other planes, body returns)
    assert abs(estimate_range_sigma(d, 0.02) / 0.02 - 1.0) <= 0.15


def test_report_without_planes_passes(
    bridged_bag: tuple[Path, BridgeReport], tmp_path: Path
) -> None:
    out_dir, _ = bridged_bag
    report = run(out_dir / "bag", _PROFILE, tmp_path)  # the bag directory itself also works
    assert "LiDAR" not in {s.name for s in report.sections}
    assert report.passed is True


def test_missing_ground_truth_fails(no_gt_dir: Path, tmp_path: Path) -> None:
    report = run(no_gt_dir, _PROFILE, tmp_path)
    assert report.passed is False
    assert "ground_truth missing" in (tmp_path / "noise_report.json").read_text()


def _cli(bag_dir: Path, out: Path) -> int:
    cmd = [sys.executable, "-m", "mss_eval.noise_report", str(bag_dir), str(_PROFILE)]
    proc = subprocess.run(
        [*cmd, "--out", str(out)],
        cwd=_REPO / "eval",
        capture_output=True,
        text=True,
        check=False,
    )
    return proc.returncode


def test_cli_exit_codes(
    bridged_bag: tuple[Path, BridgeReport], no_gt_dir: Path, tmp_path: Path
) -> None:
    out_dir, _ = bridged_bag
    assert _cli(out_dir, tmp_path / "ok") == 0
    assert _cli(no_gt_dir, tmp_path / "fail") == 1
    assert (tmp_path / "fail" / "noise_report.json").exists()


def test_imu_static_segment_gates_white_sigma() -> None:
    imu_cfg = yaml.safe_load(_PROFILE.read_text())["imu"]
    rate = imu_cfg["rate_hz"]
    rng = np.random.default_rng(5)
    n = int(10 * rate)
    t_ns = 1_700_000_000_000_000_000 + np.arange(n, dtype=np.int64) * int(1e9 / rate)
    gyro = rng.normal(0.0, imu_cfg["gyro_noise_density"] * np.sqrt(rate), (n, 3))
    acc = rng.normal(0.0, imu_cfg["acc_noise_density"] * np.sqrt(rate), (n, 3))
    acc[:, 2] += 9.80665
    sec = imu_section(
        t_ns, gyro, acc, imu_cfg, static=True, static_s=5.0, allan_gate=False, pipeline_imu=None
    )
    sigma_rows = [r for r in sec.rows if "_sigma_" in r.name]
    assert len(sigma_rows) == 6 and all(r.ok is True for r in sigma_rows), sigma_rows
    assert len(sec.figures) == 2  # Allan + static histogram
    acc[:, 0] *= 2.0  # twice the declared noise must fail
    sec = imu_section(
        t_ns, gyro, acc, imu_cfg, static=True, static_s=5.0, allan_gate=False, pipeline_imu=None
    )
    assert next(r for r in sec.rows if r.name == "acc_sigma_x").ok is False


def _room_scan(sigma_m: float, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray, tuple]:
    """Organized 16 x 720 scan from 0.5 m above the floor of a 8 x 6 x 3 m room.

    Gaussian range noise is applied along each ray. Returns (p_w, o_w, planes).
    """
    normals = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
    offsets = np.array([4.0, 4.0, 3.0, 3.0, 3.0, 0.0])  # n . p = d; floor z = 0, ceiling z = 3
    origin = np.array([0.7, -0.4, 0.5])
    el = np.deg2rad(np.linspace(-15.0, 15.0, 16))[:, None]
    az = np.deg2rad(np.arange(720) * 0.5)[None, :]
    u = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el) + 0 * az], -1)
    u = u.reshape(-1, 3)
    # Opposite normals bound the room: hit the first plane with n . u > 0 (ray leaving the room).
    nu = u @ normals.T
    t = np.where(nu > 1e-9, (offsets - origin @ normals.T) / np.where(nu > 1e-9, nu, 1.0), np.inf)
    rng_m = t.min(axis=1) + rng.normal(0.0, sigma_m, len(u))
    p_w = origin + rng_m[:, None] * u
    return p_w, np.broadcast_to(origin, p_w.shape), (normals, offsets)


def test_lidar_sigma_is_along_ray_at_mixed_incidence() -> None:
    lidar_cfg = yaml.safe_load(_PROFILE.read_text())["lidar"]
    p_w, o_w, (normals, offsets) = _room_scan(0.02, np.random.default_rng(7))
    res, cos = ray_residuals(p_w, o_w, normals, offsets)
    assert cos.min() < 0.3 and np.mean(cos < 0.9) > 0.3  # genuinely mixed incidence
    plane_dist = res * cos
    assert np.std(plane_dist) < 0.85 * 0.02  # the old point-to-plane measure under-reads
    sec = lidar_section(res, cos, 0, len(res), lidar_cfg, None)
    row = next(r for r in sec.rows if r.name == "range_sigma_m")
    assert abs(row.measured / 0.02 - 1.0) <= 0.15 and row.ok is True, row


def test_expected_dropout_accounts_for_grazing_and_nan() -> None:
    lidar_cfg = yaml.safe_load(_PROFILE.read_text())["lidar"]
    rng = np.random.default_rng(11)
    n, g_true = 400_000, 0.2
    cos_graze = np.cos(np.deg2rad(lidar_cfg["grazing_deg"]))
    grazing = rng.random(n) < g_true
    cos = np.where(grazing, rng.uniform(0.0, cos_graze, n), rng.uniform(cos_graze, 1.0, n))
    nan_inj = rng.random(n) < lidar_cfg["nan_fraction"]
    p_drop = np.where(grazing, lidar_cfg["dropout_p_grazing"], lidar_cfg["dropout_p"])
    dropped = nan_inj | (rng.random(n) < p_drop)
    p_exp, g = expected_dropout(cos[~dropped], lidar_cfg)
    assert g == pytest.approx(g_true, abs=0.01)
    assert p_exp == pytest.approx(dropped.mean(), rel=0.05)


def test_gnss_sigma_rejects_multipath_steps() -> None:
    rng = np.random.default_rng(13)
    r = rng.normal(0.0, 1.0, 5_000)
    r[: len(r) // 10] += 6.0  # 10 % of fixes carry a +6 m multipath step
    sigma, excluded = estimate_gnss_sigma(r, 1.0)
    assert abs(sigma - 1.0) <= 0.15
    assert excluded == pytest.approx(0.10, abs=0.02)


def test_load_planes_scales_d_with_normal(tmp_path: Path) -> None:
    path = tmp_path / "planes.yaml"
    path.write_text(yaml.safe_dump([{"n": [0, 0, 2], "d": 2}, {"n": [3, 4, 0], "d": -10}]))
    n, d = load_planes(path)
    np.testing.assert_allclose(n, [[0, 0, 1], [0.6, 0.8, 0]])
    np.testing.assert_allclose(d, [1.0, -2.0])
    path.write_text(yaml.safe_dump([{"n": [0, 0, 0], "d": 1}]))
    with pytest.raises(ValueError, match="zero-norm"):
        load_planes(path)


def test_missing_lidar_transform_skips_lidar(
    bridged_bag: tuple[Path, BridgeReport], planes_file: Path, tmp_path: Path
) -> None:
    src, _ = bridged_bag
    bag = copy_bag_without(src / "bag", tmp_path / "no_tf" / "bag", "/tf_static")
    report = run(bag, _PROFILE, tmp_path / "out", planes=planes_file)
    assert "LiDAR" not in {s.name for s in report.sections}
    row = _row(report, "Bag", "T_b_l")
    assert row.ok is None
    assert row.note == "no lidar transform in /tf_static; LiDAR rows skipped"


def test_gnss_lever_arm_assumed_zero_without_pipeline_config(
    report_dir: tuple[Report, Path],
) -> None:
    report, _ = report_dir
    row = _row(report, "GNSS", "p_b_g_m")
    assert row.measured == "assumed 0 (no --pipeline-config)"
    assert row.ok is None


def _floor_scan(height_m: float = 0.6, n_rings: int = 8, width: int = 360) -> np.ndarray:
    """Organized scan of the floor z = -height_m from rings at -15..-1 deg (all hit the floor)."""
    elev = np.radians(np.linspace(-15.0, -1.0, n_rings))
    az = np.linspace(-np.pi, np.pi, width, endpoint=False)
    r = height_m / np.sin(-elev)  # range to the floor along each ring
    e, a = np.meshgrid(elev, az, indexing="ij")
    rr = np.repeat(r[:, None], width, axis=1)
    xyz = np.stack([rr * np.cos(e) * np.cos(a), rr * np.cos(e) * np.sin(a), rr * np.sin(e)], -1)
    return xyz.reshape(-1, 3)


def test_neighbour_incidence_cos_on_a_floor_survives_dropout() -> None:
    from mss_eval.noise_measure import neighbour_incidence_cos

    n_rings, width = 8, 360
    xyz = _floor_scan(n_rings=n_rings, width=width)
    truth = np.abs(xyz[:, 2]) / np.linalg.norm(xyz, axis=1)  # |n . u| with n = z
    cos = neighbour_incidence_cos(xyz, n_rings, width)
    np.testing.assert_allclose(cos, truth, atol=1e-9)
    rng = np.random.default_rng(3)
    dropped = rng.random(len(xyz)) < 0.3  # heavy dropout, as at grazing incidence
    noisy = xyz.copy()
    noisy[dropped] = np.nan
    cos_d = neighbour_incidence_cos(noisy, n_rings, width)[~dropped]
    assert np.isfinite(cos_d).all()
    np.testing.assert_allclose(cos_d, truth[~dropped], atol=1e-6)
    inf_scan = xyz.copy()
    inf_scan[0] = np.inf  # a no-return point stays unknown, and so do its neighbours' normals
    assert np.isnan(neighbour_incidence_cos(inf_scan, n_rings, width)[0])
