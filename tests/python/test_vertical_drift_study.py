"""tools/vertical_drift_study.py on synthetic datasets with known answers."""

from pathlib import Path

import numpy as np
import pytest
from conftest import _REPO
from scipy.spatial.transform import Rotation

import vertical_drift_study as vds
from bag_to_mssdata import GT_DTYPE, IMU_DTYPE, LIDAR_HEADER_DTYPE, LIDAR_POINT_DTYPE

_G = 9.80665
_BA = np.array([0.5884, -0.41, 0.236])  # m/s^2
_BG = np.array([0.01, -0.02, 0.03])  # rad/s
_T0 = 1_000_000_000  # ns
_STILL_S = 3.0
_SPEED = 1.0  # m/s along x after the still start
_P_B_L = np.array([0.10, 0.0, 0.50])  # config/sim/corridor.yaml T_b_l
_R_B_L = Rotation.from_euler("y", 2.0, degrees=True)


def _gt_xyz(t_s: np.ndarray) -> np.ndarray:
    x = np.where(t_s > _STILL_S, _SPEED * (t_s - _STILL_S), 0.0)
    return np.c_[x, np.zeros_like(x), np.full_like(x, 0.1)]


def _write_dataset(d: Path, lidar_pitch_err_rad: float = 0.0) -> None:
    """Level body: still 3 s, then 1 m/s along x for 3 s; IMU with constant biases."""
    d.mkdir(parents=True)
    t_ns = _T0 + np.arange(0, 6_000_000_000, 5_000_000, dtype=np.int64)
    t_s = (t_ns - _T0) * 1e-9
    gt = np.zeros(len(t_ns), dtype=GT_DTYPE)
    gt["t"], gt["p"], gt["q_wxyz"] = t_ns, _gt_xyz(t_s), [1.0, 0.0, 0.0, 0.0]
    gt.tofile(d / "gt.bin")
    imu = np.zeros(len(t_ns), dtype=IMU_DTYPE)
    imu["t"], imu["acc"], imu["gyro"] = t_ns, [0.0, 0.0, _G] + _BA, _BG
    imu.tofile(d / "imu.bin")
    (d / "lidar").mkdir()
    # Floor grid z = 0 around the body, seen by a LiDAR whose true mount is pitched by
    # lidar_pitch_err_rad more than the overlay says.
    gx, gy = np.meshgrid(np.linspace(-6, 6, 41), np.linspace(-6, 6, 41))
    R_true = _R_B_L * Rotation.from_euler("y", lidar_pitch_err_rad)
    for k, ts in enumerate(np.arange(0.0, 6.0, 0.1)):
        p_w_b = _gt_xyz(np.array([ts]))[0]
        floor_w = np.c_[gx.ravel() + p_w_b[0], gy.ravel(), np.zeros(gx.size)]
        p_l = R_true.inv().apply(floor_w - p_w_b - _P_B_L)
        pts = np.zeros(len(p_l), dtype=LIDAR_POINT_DTYPE)
        pts["xyz"], pts["ring"] = p_l, 0
        head = np.array([(_T0 + int(round(ts * 1e9)), len(p_l))], dtype=LIDAR_HEADER_DTYPE)
        (d / "lidar" / f"{k:06d}.bin").write_bytes(head.tobytes() + pts.tobytes())


def _write_est(path: Path, pitch_rad: float, climb: float) -> None:
    t_s = np.arange(0.0, 6.0, 0.1)
    p = _gt_xyz(t_s)
    p[:, 2] += climb * p[:, 0]
    q = Rotation.from_euler("y", pitch_rad).as_quat()  # scipy (x, y, z, w) = TUM order
    q_txt = " ".join(f"{v:.9f}" for v in q)
    lines = [
        f"{(_T0 * 1e-9 + ts):.9f} {x:.9f} {y:.9f} {z:.9f} {q_txt}"
        for ts, (x, y, z) in zip(t_s, p, strict=True)
    ]
    path.write_text("\n".join(lines) + "\n")


@pytest.fixture(scope="module")
def data(tmp_path_factory: pytest.TempPathFactory) -> Path:
    d = tmp_path_factory.mktemp("vds") / "mssdata"
    _write_dataset(d)
    return d


def test_bias_recovers_the_constant_biases(data: Path) -> None:
    b = vds.true_bias(data)
    assert np.allclose(b["ba_mps2"], _BA, atol=1e-12)
    assert np.allclose(b["bg_radps"], _BG, atol=1e-12)
    assert b["still_s"] == pytest.approx(_STILL_S - 0.2, abs=0.01)
    assert b["tilt_from_bias_mrad"] == pytest.approx([1e3 * 0.41 / _G, 1e3 * 0.5884 / _G])


@pytest.mark.parametrize(
    ("mode", "acc_left", "gyro_left"),
    [
        ("acc", [0.0, 0.0, 0.0], _BG),
        ("accxy", [0.0, 0.0, _BA[2]], _BG),
        ("accz", [_BA[0], _BA[1], 0.0], _BG),
        ("gyro", _BA, [0.0, 0.0, 0.0]),
    ],
)
def test_debias_removes_the_selected_bias(
    data: Path, tmp_path: Path, mode: str, acc_left: list[float], gyro_left: np.ndarray
) -> None:
    out = tmp_path / mode
    vds.debias(data, out, mode)
    imu = np.fromfile(out / "imu.bin", dtype=IMU_DTYPE)
    assert np.allclose(imu["acc"] - [0.0, 0.0, _G], acc_left, atol=1e-12)
    assert np.allclose(imu["gyro"], gyro_left, atol=1e-12)
    assert (out / "gt.bin").read_bytes() == (data / "gt.bin").read_bytes()
    assert len(list((out / "lidar").glob("*.bin"))) == 60
    assert vds.true_bias(out)["ba_mps2"] == pytest.approx(list(acc_left), abs=1e-12)


def test_compare_reads_tilt_and_climb(data: Path, tmp_path: Path) -> None:
    est = tmp_path / "est.tum"
    _write_est(est, pitch_rad=0.03, climb=0.05)
    r = vds.compare(est, data)
    assert r["end_ez_m"] == pytest.approx(0.05 * _SPEED * 2.9, abs=1e-6)  # last estimate 5.9 s
    assert r["tilt_y_mrad_mean"] == pytest.approx(-1e3 * np.sin(0.03), abs=1e-6)
    assert r["tilt_x_mrad_mean"] == pytest.approx(0.0, abs=1e-9)
    assert r["end_eh_m"] == pytest.approx(0.0, abs=1e-9)
    # A straight climb is almost a rigid rotation: SE(3) alignment hides it (backlog A4).
    assert r["ate_se3_m"] < 0.05 * r["end_ez_m"]


def test_floor_oracle(data: Path, tmp_path: Path) -> None:
    overlay = _REPO / "config/sim/corridor.yaml"
    ok = vds.floor(data, overlay, z_max_m=0.08)
    assert ok["fit_z0_m"] == pytest.approx(0.0, abs=1e-5)
    assert ok["fit_slope_x_mrad"] == pytest.approx(0.0, abs=1e-3)
    tilted = tmp_path / "tilted"
    _write_dataset(tilted, lidar_pitch_err_rad=0.005)
    bad = vds.floor(tilted, overlay, z_max_m=0.08)
    assert abs(bad["fit_slope_x_mrad"]) == pytest.approx(5.0, rel=0.02)


def test_cli_errors_exit_2(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert vds.main(["bias", str(tmp_path / "missing")]) == 2
    assert "vertical_drift_study: error:" in capsys.readouterr().err
