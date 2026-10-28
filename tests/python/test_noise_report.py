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
    assert sigma.ok is None  # pooled row informational: the range sigma is gated per bin
    cells = [r for r in _section(report, "LiDAR").rows if r.name.startswith("range_sigma_m r ")]
    assert cells and any(r.ok is False for r in cells)  # no table: gated on declared sigma
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


def test_unknown_incidence_counts_as_non_grazing() -> None:
    """A NaN incidence (neighbour without return, e.g. sky) is non-grazing, as in the bridge."""
    lidar_cfg = {
        "range_sigma_m": 0.02,
        "dropout_p": 0.02,
        "dropout_p_grazing": 0.3,
        "grazing_deg": 80.0,
        "nan_fraction": 0.0,
    }
    res = np.zeros(100)
    cos = np.ones(100)
    inc = np.full(100, np.nan)
    inc[:10] = 0.05  # grazing
    sec = lidar_section(res, cos, 0, 100, lidar_cfg, None, inc)
    dropout = next(r for r in sec.rows if r.name == "dropout_fraction")
    known_only = lidar_section(res, cos, 0, 100, lidar_cfg, None, inc[:10])
    p_known = next(r for r in known_only.rows if r.name == "dropout_fraction").declared
    assert dropout.declared < p_known  # 90 unknown points dilute g instead of being dropped
    p_exp, g = expected_dropout(np.nan_to_num(inc, nan=1.0), lidar_cfg)
    assert dropout.declared == pytest.approx(p_exp) and 0.05 < g < 0.2


_LIDAR_CFG = {"range_sigma_m": 0.02, "edge_jump_m": 0.10, "min_incidence_cos": 0.3}
# Two cells with a known additive render error: r [10, 20) x |n.u| [0.9, 1] and [0.5, 0.7].
_CELLS = [(15.0, 0.95, 0.008), (15.0, 0.6, 0.025), (3.0, 0.95, 0.001)]


def _cell_rows(res: np.ndarray, cos: np.ndarray, ranges: np.ndarray, table: list | None) -> dict:
    from mss_eval.noise_measure import range_sigma_rows

    edge = np.zeros(len(res), dtype=bool)
    rows = range_sigma_rows(res, cos, ranges, edge, _LIDAR_CFG, table)
    return {r.name: r for r in rows if r.name.startswith("range_sigma_m r ")}


def _synthetic_cells(
    rng: np.random.Generator, declared: float, n: int = 20_000
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Residuals: declared sigma plus a per-cell additive render error (``_CELLS``)."""
    res, cos, ranges = [], [], []
    for r, c, s_render in _CELLS:
        res.append(rng.normal(0.0, np.hypot(declared, s_render), n))
        cos.append(np.full(n, c))
        ranges.append(np.full(n, r))
    return np.concatenate(res), np.concatenate(cos), np.concatenate(ranges)


def _table_from(cells: list[tuple[float, float, float]]) -> list:
    from mss_eval.noise_measure import build_render_table

    rng = np.random.default_rng(1)
    res, cos, ranges = [], [], []
    for r, c, s_render in cells:
        res.append(rng.normal(0.0, s_render, 20_000))
        cos.append(np.full(20_000, c))
        ranges.append(np.full(20_000, r))
    res_a, cos_a, rng_a = np.concatenate(res), np.concatenate(cos), np.concatenate(ranges)
    return build_render_table(res_a, cos_a, rng_a, np.ones(len(res_a), dtype=bool), 0.1)


def test_table_builder_recovers_a_known_additive_error() -> None:
    table = _table_from(_CELLS)
    by_cell = {(row[0], row[2]): row for row in table}
    assert len(table) == 3
    for r, c, s_render in _CELLS:
        key = (10.0 if r == 15.0 else 2.0, 0.9 if c == 0.95 else 0.5)
        row = by_cell[key]
        assert row[4] == 20_000
        assert row[5] == pytest.approx(s_render, rel=0.05)


def test_cells_declared_sigma_only_passes_without_render_error() -> None:
    res, cos, ranges = _synthetic_cells(np.random.default_rng(5), 0.02)
    zero = [[*row[:5], 0.0] for row in _table_from(_CELLS)]  # calibrated, no render error
    rng = np.random.default_rng(9)
    res = rng.normal(0.0, 0.02, len(res))
    rows = _cell_rows(res, cos, ranges, zero)
    assert len(rows) == 3 and all(r.ok is True for r in rows.values())


def test_cell_error_passes_only_with_the_matching_table() -> None:
    res, cos, ranges = _synthetic_cells(np.random.default_rng(6), 0.02)
    matching = _cell_rows(res, cos, ranges, _table_from(_CELLS))
    assert all(r.ok is True for r in matching.values())
    no_error = [[*row[:5], 0.0] for row in _table_from(_CELLS)]
    assert (
        _cell_rows(res, cos, ranges, no_error)["range_sigma_m r [10, 20) m |n.u| [0.5, 0.7]"].ok
        is False
    )
    wrong = _table_from([(15.0, 0.95, 0.008), (15.0, 0.6, 0.004), (3.0, 0.95, 0.001)])
    assert (
        _cell_rows(res, cos, ranges, wrong)["range_sigma_m r [10, 20) m |n.u| [0.5, 0.7]"].ok
        is False
    )


def test_uncalibrated_and_small_cells_shown_not_gated() -> None:
    rng = np.random.default_rng(7)
    cos = np.concatenate([np.full(5000, 0.95), np.full(500, 0.6), np.full(300, 0.1)])
    ranges = np.full(len(cos), 15.0)
    res = rng.normal(0, 0.02, len(cos))
    no_table = _cell_rows(res, cos, ranges, None)  # render error taken as 0, gated
    assert no_table["range_sigma_m r [10, 20) m |n.u| [0.9, 1]"].ok is True
    only_head_on = [[10.0, 20.0, 0.9, 1.0, 5000, 0.0]]
    rows = _cell_rows(res, cos, ranges, only_head_on)
    assert "no calibrated" in rows["range_sigma_m r [10, 20) m |n.u| [0.5, 0.7]"].tol
    table = [[10.0, 20.0, 0.9, 1.0, 5000, 0.0], [10.0, 20.0, 0.5, 0.7, 500, 0.0]]
    rows = _cell_rows(res, cos, ranges, table)
    assert rows["range_sigma_m r [10, 20) m |n.u| [0.9, 1]"].ok is True
    small = rows["range_sigma_m r [10, 20) m |n.u| [0.5, 0.7]"]
    assert small.ok is None and "points" in small.tol
    from mss_eval.noise_measure import range_sigma_rows

    excl = {
        r.name: r
        for r in range_sigma_rows(res, cos, ranges, np.zeros(len(res), bool), _LIDAR_CFG, table)
    }
    assert excl["low-incidence points excluded"].measured == pytest.approx(300 / 5800)


def _table_doc(**over: object) -> dict:
    from mss_eval.noise_measure import INCIDENCE_BINS_COS, RANGE_BINS_M

    doc = {
        "world": "warehouse",
        "range_bins_m": list(RANGE_BINS_M),
        "incidence_bins_cos": list(INCIDENCE_BINS_COS),
        "cells": [[10.0, 20.0, 0.9, 1.0, 5000, 0.01]],
    }
    doc.update(over)
    return doc


def test_load_render_table_valid(tmp_path: Path) -> None:
    from mss_eval.noise_report import load_render_table

    f = tmp_path / "t.yaml"
    f.write_text(yaml.safe_dump(_table_doc()))
    assert load_render_table(f, "warehouse") == [[10.0, 20.0, 0.9, 1.0, 5000, 0.01]]


@pytest.mark.parametrize(
    ("over", "match"),
    [
        ({"cells": []}, "non-empty"),
        ({"world": "corridor"}, "planes world"),
        ({"range_bins_m": [0.5, 2.0, 100.0]}, "range_bins_m"),
        ({"incidence_bins_cos": [0.3, 1.0]}, "incidence_bins_cos"),
        ({"cells": [[10.0, 15.0, 0.9, 1.0, 5000, 0.01]]}, "not a cell of the bin grid"),
        ({"cells": [[10.0, 20.0, 0.9, 1.0, 0, 0.01]]}, "n > 0"),
        ({"cells": [[10.0, 20.0, 0.9, 1.0, 10, -0.01]]}, "sigma >= 0"),
        ({"cells": [[1, 2, 3]]}, "expected"),
    ],
)
def test_load_render_table_rejects(tmp_path: Path, over: dict, match: str) -> None:
    from mss_eval.noise_report import load_render_table

    f = tmp_path / "t.yaml"
    f.write_text(yaml.safe_dump(_table_doc(**over)))
    with pytest.raises(ValueError, match=match):
        load_render_table(f, "warehouse")


def test_no_gated_cell_fails() -> None:
    """A table that calibrates none of the measured cells leaves the range noise unchecked."""
    from mss_eval.noise_measure import range_sigma_rows

    rng = np.random.default_rng(3)
    res = rng.normal(0, 0.02, 5000)
    cos, ranges = np.full(5000, 0.95), np.full(5000, 15.0)
    elsewhere = [[0.5, 2.0, 0.9, 1.0, 5000, 0.001]]
    rows = range_sigma_rows(res, cos, ranges, np.zeros(5000, bool), _LIDAR_CFG, elsewhere)
    check = next(r for r in rows if r.name == "range sigma cells gated")
    assert check.ok is False and check.measured == 0
    rows = range_sigma_rows(res, cos, ranges, np.zeros(5000, bool), _LIDAR_CFG, None)
    assert next(r for r in rows if r.name == "range sigma cells gated").ok is True


def test_noise_free_range_binning_unbiased_at_a_bin_edge() -> None:
    """Points crowding a range-bin edge: binning on the measured range truncates the cell's
    error one-sidedly; binning on the noise-free plane range recovers sigma within 2 %."""
    from mss_eval.noise_measure import cell_sigma, in_cell

    rng = np.random.default_rng(11)
    n = 200_000
    r_true = rng.uniform(1.9, 2.0, n)  # just below the 2 m edge, like corridor floor points
    err = rng.normal(0.0, 0.02, n)
    r_meas = r_true + err
    cos = np.full(n, 0.95)
    cell = (0.5, 2.0, 0.9, 1.0)
    biased, _ = cell_sigma(err[in_cell(r_meas, cos, cell)], 0.02, 0.0)
    clean, _ = cell_sigma(err[in_cell(r_meas - err, cos, cell)], 0.02, 0.0)
    assert biased < 0.95 * 0.02  # the one-sided cut is visible
    assert clean == pytest.approx(0.02, rel=0.02)


def test_plane_ranges_are_noise_free() -> None:
    from mss_eval.noise_measure import plane_ranges

    rng = np.random.default_rng(12)
    u = np.array([[1.0, 0.3, -0.1]])
    u = u / np.linalg.norm(u)
    o = np.array([[0.5, -0.2, 0.6]])
    normals, offsets = np.array([[1.0, 0.0, 0.0]]), np.array([10.0])
    r_star = (10.0 - o[0, 0]) / u[0, 0]
    r = r_star + rng.normal(0, 0.05, 50)
    p = o + r[:, None] * u
    np.testing.assert_allclose(plane_ranges(p, np.repeat(o, 50, 0), normals, offsets), r_star)


def test_oblique_plane_not_flagged_as_edge() -> None:
    """A wall at 65-70 deg incidence with noise: adjacent-beam ranges differ by ~0.2-0.5 m, yet
    fewer than 1 % of points are flagged (the rule uses the distance to the point's plane)."""
    from mss_eval.noise_measure import plane_edge_mask, ray_residuals

    rng = np.random.default_rng(13)
    height, width = 16, 720
    elev = np.radians(np.linspace(-15, 15, height))
    az = np.radians(np.linspace(-180, 180, width, endpoint=False))
    e, a = np.meshgrid(elev, az, indexing="ij")
    u = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1).reshape(-1, 3)
    normal = np.array([0.0, 1.0, 0.0])  # wall y = 5
    n_u = u @ normal
    hits = n_u > 0.05
    xyz = np.full((len(u), 3), np.inf)  # no return
    r = 5.0 / n_u[hits] + rng.normal(0, 0.02, hits.sum())
    xyz[hits] = u[hits] * r[:, None]
    ok = np.isfinite(xyz).all(axis=1)
    p = xyz[ok]
    edge = plane_edge_mask(xyz, ok, p, normal[None, :], np.array([5.0]), height, width, 0.10)
    _, cos = ray_residuals(p, np.zeros(3), normal[None, :], np.array([5.0]))
    oblique = (cos > np.cos(np.radians(70))) & (cos < np.cos(np.radians(65)))
    assert oblique.sum() > 200
    # Exclude the wall's own silhouette rows/columns next to no-return (inf) points.
    grid_ok = ok.reshape(height, width)
    interior = np.ones_like(grid_ok)
    for axis, step in ((0, 1), (0, -1), (1, 1), (1, -1)):
        nb = np.roll(grid_ok, step, axis=axis)
        if axis == 0:
            nb[0 if step == 1 else -1] = True
        interior &= nb
    interior = interior.reshape(-1)[ok]
    assert edge[oblique & interior].mean() < 0.01


def test_dropped_neighbour_counts_as_edge() -> None:
    from mss_eval.noise_measure import plane_edge_mask

    height, width = 3, 5
    xyz = np.zeros((height * width, 3))
    xyz[:, 0] = 10.0
    xyz[:, 1] = np.tile(np.arange(width), height) * 0.1
    xyz[:, 2] = np.repeat(np.arange(height), width) * 0.1
    xyz[7] = np.nan  # dropped (middle row, middle column)
    ok = np.isfinite(xyz).all(axis=1)
    normals, offsets = np.array([[1.0, 0.0, 0.0]]), np.array([10.0])
    edge = np.zeros(len(xyz), dtype=bool)
    edge[ok] = plane_edge_mask(xyz, ok, xyz[ok], normals, offsets, height, width, 0.10)
    assert edge[[2, 6, 8, 12]].all()  # the four grid neighbours of the dropped point
    assert not edge[[0, 4, 10, 14]].any()


def test_edge_points_excluded_plane_sigma_recovered() -> None:
    """A wall with a box in front: the wall points next to the box silhouette carry mixed depth
    (as gpu_lidar's texels do); excluding edges recovers the wall's sigma within 2 %."""
    from mss_eval.noise_measure import plane_edge_mask, ray_residuals

    rng = np.random.default_rng(8)
    height, width, sigma = 16, 360, 0.02
    elev = np.radians(np.linspace(-15, 15, height))
    az = np.radians(np.linspace(-40, 40, width))
    e, a = np.meshgrid(elev, az, indexing="ij")
    u = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], -1).reshape(-1, 3)
    r_wall = 10.0 / u[:, 0]  # wall x = 10
    box = (np.abs(a) < np.radians(8)).reshape(-1) & (e < np.radians(5)).reshape(-1)
    r = np.where(box, 8.0 / u[:, 0], r_wall)  # box face x = 8 in front of the wall
    r = r + rng.normal(0, sigma, len(r))
    xyz = u * r[:, None]
    grid_box = box.reshape(height, width)
    silhouette = np.zeros_like(grid_box)
    for axis, step in ((0, 1), (0, -1), (1, 1), (1, -1)):
        nb = np.roll(grid_box, step, axis=axis)
        if axis == 0:  # rings do not wrap
            nb[0 if step == 1 else -1] = False
        silhouette |= nb & ~grid_box
    mixed = silhouette.reshape(-1)
    xyz[mixed] -= u[mixed] * rng.uniform(0.02, 0.09, mixed.sum())[:, None]  # mixed depth texels
    normals, offsets = np.array([[1.0, 0.0, 0.0]]), np.array([10.0])
    ok = np.ones(len(xyz), dtype=bool)
    edge = plane_edge_mask(xyz, ok, xyz, normals, offsets, height, width, 0.10)
    res, cos = ray_residuals(xyz, np.zeros(3), normals, offsets)
    on_wall = (np.abs(res) <= 0.1) & ~box
    assert edge[mixed].all()
    # Clean wall points are excluded only next to a mixed point whose depth plus noise exceeds
    # edge_jump_m (the next ring of the silhouette): a small share.
    assert edge[~box & ~mixed & on_wall].mean() < 0.05
    contaminated = float(np.std(res[on_wall], ddof=1))
    clean = float(np.std(res[on_wall & ~edge], ddof=1))
    assert contaminated > 1.05 * sigma
    assert clean == pytest.approx(sigma, rel=0.02)
