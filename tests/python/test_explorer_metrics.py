"""Explorer metrics: error over time, translation RPE, ATE change against a reference."""

from pathlib import Path

import numpy as np
import pytest

from mss_eval.ate import align
from mss_explorer.metrics import delta_ate_pct, error_series, rpe_t


def _tum(path: Path, t_s: np.ndarray, xyz: np.ndarray) -> Path:
    lines = [
        f"{t:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1" for t, (x, y, z) in zip(t_s, xyz, strict=True)
    ]
    path.write_text("\n".join(lines) + "\n")
    return path


def _line(n: int = 101) -> tuple[np.ndarray, np.ndarray]:
    t = 100.0 + np.arange(n, dtype=np.float64)
    return t, np.column_stack([np.arange(n, dtype=np.float64), np.zeros(n), np.zeros(n)])


def test_error_series_zero_for_identical(tmp_path: Path) -> None:
    t, xyz = _line()
    p = _tum(tmp_path / "a.tum", t, xyz)
    ts, err = error_series(p, p)
    np.testing.assert_allclose(ts, t - t[0])
    np.testing.assert_allclose(err, 0.0, atol=1e-9)


def test_alignment_reports_pair_stamps(tmp_path: Path) -> None:
    t, xyz = _line()
    gt = _tum(tmp_path / "gt.tum", t, xyz)
    est = _tum(tmp_path / "est.tum", t[10:], xyz[10:])
    al = align(est, gt)
    assert al.t_ns[0] == round(t[10] * 1e9) and len(al.t_ns) == len(al.gt)


def test_rpe_t_known_drift() -> None:
    _, g = _line()
    e = g * 1.01  # 1 % scale error: every 10 m segment is 0.1 m too long
    assert rpe_t(e, g, 10.0) == pytest.approx(0.1, abs=1e-6)


def test_rpe_t_none_when_short() -> None:
    _, g = _line(5)
    assert rpe_t(g, g, 50.0) is None


def test_delta_ate_pct() -> None:
    assert delta_ate_pct(0.5, 1.0) == pytest.approx(100.0)
    assert delta_ate_pct(2.0, 1.0) == pytest.approx(-50.0)
