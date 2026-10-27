"""ATE RMSE from TUM files: association, SE(3) alignment, EV-01, CLI."""

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mss_eval.ate import associate, ate, plot_xy, read_tum_stamps_ns, umeyama_se3

_T0_NS = 1_700_000_000_000_000_000  # epoch scale: float seconds would lose the nanoseconds
_DT_NS = 5_000_000  # 200 Hz
_REPO = Path(__file__).resolve().parents[2]


def _write_tum(
    path: Path, t_ns: np.ndarray, p: np.ndarray, q_xyzw: np.ndarray | None = None
) -> None:
    if q_xyzw is None:
        q_xyzw = np.tile([0.0, 0.0, 0.0, 1.0], (len(t_ns), 1))
    lines = ["# t x y z qx qy qz qw"]
    for t, pos, q in zip(t_ns.tolist(), p, q_xyzw, strict=True):
        stamp = f"{t // 1_000_000_000}.{t % 1_000_000_000:09d}"
        lines.append(" ".join([stamp, *(f"{v:.12f}" for v in (*pos, *q))]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _trajectory(n: int) -> tuple[np.ndarray, np.ndarray]:
    """A 3D curve (not planar, not collinear) sampled at 200 Hz."""
    t_ns = _T0_NS + np.arange(n, dtype=np.int64) * _DT_NS
    s = np.linspace(0.0, 4.0 * np.pi, n)
    p = np.stack([5.0 * np.cos(s), 3.0 * np.sin(s), 0.5 * s + 0.2 * np.sin(3 * s)], axis=1)
    return t_ns, p


def test_ev01_self_is_zero(tmp_path: Path) -> None:
    t_ns, p = _trajectory(500)
    _write_tum(tmp_path / "x.tum", t_ns, p)
    res = ate(tmp_path / "x.tum", tmp_path / "x.tum")
    assert res.rmse_m < 1e-12
    assert (res.n_pairs, res.n_unmatched) == (500, 0)


def test_rigid_offset_removed(tmp_path: Path) -> None:
    t_ns, p = _trajectory(500)
    rot = Rotation.from_euler("z", 30.0, degrees=True).as_matrix()
    est = p @ rot.T + np.array([5.0, -3.0, 1.0])
    _write_tum(tmp_path / "gt.tum", t_ns, p)
    _write_tum(tmp_path / "est.tum", t_ns, est)
    assert ate(tmp_path / "est.tum", tmp_path / "gt.tum").rmse_m < 1e-9


def test_known_noise(tmp_path: Path) -> None:
    t_ns, p = _trajectory(2000)
    rng = np.random.default_rng(0)
    _write_tum(tmp_path / "gt.tum", t_ns, p)
    _write_tum(tmp_path / "est.tum", t_ns, p + rng.normal(0.0, 0.1, p.shape))
    rmse = ate(tmp_path / "est.tum", tmp_path / "gt.tum").rmse_m
    assert rmse == pytest.approx(0.1 * np.sqrt(3.0), rel=0.05)


def test_umeyama_recovers_rotation_and_translation() -> None:
    _, p = _trajectory(200)
    rot = Rotation.from_euler("xyz", [10.0, -20.0, 30.0], degrees=True).as_matrix()
    t = np.array([1.0, 2.0, -3.0])
    r_est, t_est = umeyama_se3(p, p @ rot.T + t)
    np.testing.assert_allclose(r_est, rot, atol=1e-12)
    np.testing.assert_allclose(t_est, t, atol=1e-12)
    assert np.linalg.det(r_est) == pytest.approx(1.0)


def test_stamps_parsed_exactly(tmp_path: Path) -> None:
    path = tmp_path / "s.tum"
    path.write_text(
        "# t x y z qx qy qz qw\n"
        "1700000000.123456789 0 0 0 0 0 0 1\n"
        "1700000000.5 0 0 0 0 0 0 1\n"
        "1700000001 0 0 0 0 0 0 1\n",
        encoding="utf-8",
    )
    stamps = read_tum_stamps_ns(path)
    assert stamps.dtype == np.int64
    assert stamps.tolist() == [1700000000123456789, 1700000000500000000, 1700000001000000000]


def test_associate_nearest_within_tolerance() -> None:
    gt = _T0_NS + np.arange(10, dtype=np.int64) * 10_000_000  # 100 Hz: 3 ms off stays 3 ms off
    est = np.array([gt[2] + 1_000_000, gt[5] - 3_000_000, gt[7]], dtype=np.int64)
    i_est, i_gt = associate(est, gt, 2_500_000)
    assert i_est.tolist() == [0, 2]  # 1 ms off and exact: matched; 3 ms off: unmatched
    assert i_gt.tolist() == [2, 7]


def test_associate_picks_the_nearer_neighbour() -> None:
    gt = np.array([0, 10, 20], dtype=np.int64)
    i_est, i_gt = associate(np.array([4, 6, 25, -1], dtype=np.int64), gt, 5)
    assert i_est.tolist() == [0, 1, 2, 3]
    assert i_gt.tolist() == [0, 1, 2, 0]


def test_unmatched_estimates_are_counted(tmp_path: Path) -> None:
    t_ns, p = _trajectory(100)
    _write_tum(tmp_path / "gt.tum", t_ns, p)
    est_t = t_ns.copy()
    est_t[:5] = t_ns[0] - 3_000_000 - np.arange(5) * _DT_NS  # >= 3 ms before the first GT stamp
    _write_tum(tmp_path / "est.tum", est_t, p)
    res = ate(tmp_path / "est.tum", tmp_path / "gt.tum")
    assert (res.n_pairs, res.n_unmatched) == (95, 5)


def test_too_few_pairs_raises(tmp_path: Path) -> None:
    t_ns, p = _trajectory(9)
    _write_tum(tmp_path / "x.tum", t_ns, p)
    with pytest.raises(ValueError, match="9 pairs"):
        ate(tmp_path / "x.tum", tmp_path / "x.tum", min_pairs=10)


def test_plot_xy_writes_png(tmp_path: Path) -> None:
    t_ns, p = _trajectory(100)
    _write_tum(tmp_path / "x.tum", t_ns, p)
    plot_xy(tmp_path / "x.tum", tmp_path / "x.tum", tmp_path / "xy.png")
    assert (tmp_path / "xy.png").read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_cli_prints_json(tmp_path: Path) -> None:
    t_ns, p = _trajectory(50)
    _write_tum(tmp_path / "x.tum", t_ns, p)
    env = {**os.environ, "PYTHONPATH": str(_REPO / "eval")}
    cmd = [sys.executable, "-m", "mss_eval.ate", str(tmp_path / "x.tum"), str(tmp_path / "x.tum")]
    out = subprocess.run(cmd, capture_output=True, text=True, check=False, env=env)
    assert out.returncode == 0, out.stderr
    data = json.loads(out.stdout)
    assert set(data) == {"ate_rmse_m", "n_pairs", "n_unmatched"}
    assert data["n_pairs"] == 50
    bad = subprocess.run(
        [*cmd[:3], str(tmp_path / "missing.tum"), str(tmp_path / "x.tum")],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    assert bad.returncode == 2
