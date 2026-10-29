"""map-png: PCD reading, height cut, density grid and image size on known synthetic data."""

import json
import os
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from mss_eval.map_png import (
    MapPngError,
    density_grid,
    figure_size_px,
    main,
    read_pcd,
    render_map_png,
)

_REPO = Path(__file__).resolve().parents[2]
_T0_NS = 1_700_000_000_000_000_000
_DT_NS = 5_000_000  # 200 Hz


def _write_pcd(path: Path, pts: np.ndarray, *, data: str = "binary") -> None:
    """PCD v0.7 ``x y z`` float32, the header ``io::write_pcd`` writes."""
    n = len(pts)
    head = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\n"
        f"DATA {data}\n"
    )
    if data == "binary":
        body = np.asarray(pts, dtype="<f4").tobytes()
    else:
        body = "".join(f"{x} {y} {z}\n" for x, y, z in pts).encode()
    path.write_bytes(head.encode() + body)


def _write_tum(path: Path, p: np.ndarray) -> None:
    lines = ["# t x y z qx qy qz qw"]
    for i, pos in enumerate(p):
        t = _T0_NS + i * _DT_NS
        stamp = f"{t // 1_000_000_000}.{t % 1_000_000_000:09d}"
        lines.append(" ".join([stamp, *(f"{v:.12f}" for v in pos), "0 0 0 1"]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _png_size(path: Path) -> tuple[int, int]:
    raw = path.read_bytes()
    assert raw[:8] == b"\x89PNG\r\n\x1a\n"
    assert raw[12:16] == b"IHDR"
    return struct.unpack(">II", raw[16:24])


def _cell(v: float, lo: float, res: float) -> int:
    return int((v - lo) // res)


def _scene() -> np.ndarray:
    """300 floor (z -0.2), 80 wall (z 1.0) and 200 ceiling (z 3.0) points in x [0, 10], y [0, 4].

    The wall points sit in two known cells: 50 at (2.1, 1.1) and 30 at (7.6, 3.6).
    """
    rng = np.random.default_rng(7)
    floor = np.column_stack([rng.uniform(0, 10, 300), rng.uniform(0, 4, 300), np.full(300, -0.2)])
    ceil = np.column_stack([rng.uniform(0, 10, 200), rng.uniform(0, 4, 200), np.full(200, 3.0)])
    wall = np.array([[2.1, 1.1, 1.0]] * 50 + [[7.6, 3.6, 1.0]] * 30)
    return np.vstack([floor, wall, ceil])


def test_read_pcd_binary_and_ascii(tmp_path: Path) -> None:
    pts = _scene()
    _write_pcd(tmp_path / "b.pcd", pts)
    _write_pcd(tmp_path / "a.pcd", pts, data="ascii")
    np.testing.assert_array_equal(read_pcd(tmp_path / "b.pcd"), pts.astype(np.float32))
    np.testing.assert_allclose(read_pcd(tmp_path / "a.pcd"), pts, atol=1e-12)


def test_density_grid_known_cells() -> None:
    xy = np.array([[2.1, 1.1]] * 5 + [[7.6, 3.6]] * 3 + [[20.0, 20.0]])  # last: outside
    g = density_grid(xy, 0.25, np.array([-1.0, -1.0]), np.array([11.0, 5.0]))
    assert g.counts.shape == (24, 48)  # 6 / 0.25, 12 / 0.25
    assert g.extent == (-1.0, 11.0, -1.0, 5.0)
    assert g.counts[_cell(1.1, -1, 0.25), _cell(2.1, -1, 0.25)] == 5
    assert g.counts[_cell(3.6, -1, 0.25), _cell(7.6, -1, 0.25)] == 3
    assert g.counts.sum() == 8


def test_density_grid_float_edge_gives_exact_cell_count() -> None:
    g = density_grid(np.zeros((0, 2)), 0.05, np.array([-1.0, -1.0]), np.array([11.0, 5.0]))
    assert g.counts.shape == (120, 240)


def test_height_cut_keeps_only_the_points_between_the_heights(tmp_path: Path) -> None:
    _write_pcd(tmp_path / "m.pcd", _scene())

    def n_cut(z_min: float, z_max: float) -> int:
        res = render_map_png(
            tmp_path / "m.pcd", tmp_path / "m.png", z_min_m=z_min, z_max_m=z_max, res_m=0.25
        )
        assert res.n_points == 580
        return res.n_in_cut

    assert n_cut(0.3, 2.0) == 80  # walls only
    assert n_cut(-1.0, 5.0) == 580
    assert n_cut(-0.3, -0.1) == 300  # floor only
    assert n_cut(2.5, 3.5) == 200  # ceiling only


def test_image_size_and_ate_with_trajectories(tmp_path: Path) -> None:
    _write_pcd(tmp_path / "m.pcd", _scene())
    s = np.linspace(0.0, 2.0 * np.pi, 400)
    est = np.column_stack([5 + 3 * np.cos(s), 2 + 1.5 * np.sin(s), 0.1 * np.sin(2 * s)])
    rot = Rotation.from_euler("z", 40.0, degrees=True).as_matrix()
    gt = est @ rot.T + np.array([100.0, -50.0, 3.0])  # ground-truth frame: rigid motion of w
    _write_tum(tmp_path / "est.tum", est)
    _write_tum(tmp_path / "gt.tum", gt)
    res = render_map_png(
        tmp_path / "m.pcd",
        tmp_path / "m.png",
        est_tum=tmp_path / "est.tum",
        gt_tum=tmp_path / "gt.tum",
        res_m=0.25,
        width_px=800,
    )
    # Wall points and trajectories span x [2, 8], y [0.5, 3.6], plus 1 m margin: x [1, 9],
    # y [-0.5, 4.6] -> 32 x 21 cells of 0.25 m; height round(800 * 21 / 32) + 90 = 615.
    assert (res.grid_nx, res.grid_ny) == (32, 21)
    assert (res.width_px, res.height_px) == figure_size_px(32, 21, 800) == (800, 615)
    assert _png_size(tmp_path / "m.png") == (800, 615)
    assert res.ate_rmse_m is not None
    assert res.ate_rmse_m < 1e-6
    assert res.n_pairs == 400


def test_narrow_map_gets_minimum_height() -> None:
    assert figure_size_px(1200, 50, 1600) == (1600, 480)


_HEAD = b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"


@pytest.mark.parametrize(
    ("content", "reason"),
    [
        (None, "cannot read"),
        (b"hello\nworld\n", "no DATA line"),
        (b"FIELDS a b c\nSIZE 4 4 4\nTYPE F F F\nPOINTS 1\nDATA binary\n", "no x, y, z"),
        (_HEAD + b"POINTS 3\nDATA binary\n\0\0", "truncated"),
        (_HEAD + b"POINTS 1\nDATA binary_compressed\n", "not supported"),
        (_HEAD + b"POINTS 2\nDATA ascii\n1 2 3\n", "ascii rows"),
    ],
)
def test_bad_pcd_is_a_clean_error(tmp_path: Path, content: bytes | None, reason: str) -> None:
    path = tmp_path / "bad.pcd"
    if content is not None:
        path.write_bytes(content)
    with pytest.raises(MapPngError, match=reason) as info:
        read_pcd(path)
    assert str(path) in str(info.value)


def test_cli_bad_inputs_exit_2_without_traceback(tmp_path: Path) -> None:
    _write_pcd(tmp_path / "m.pcd", _scene())
    (tmp_path / "bad.tum").write_text("1.0 2 3\n", encoding="utf-8")
    cases = [
        [str(tmp_path / "missing.pcd"), "--out", "x.png"],
        [str(tmp_path / "m.pcd"), "--est", str(tmp_path / "bad.tum"), "--out", "x.png"],
        [str(tmp_path / "m.pcd"), "--z-min-m", "2", "--z-max-m", "1", "--out", "x.png"],
    ]
    env = {**os.environ, "PYTHONPATH": str(_REPO / "eval"), "MPLBACKEND": "Agg"}
    for argv in cases:
        proc = subprocess.run(
            [sys.executable, "-m", "mss_eval", "map-png", *argv],
            capture_output=True,
            text=True,
            cwd=tmp_path,
            env=env,
            check=False,
        )
        assert proc.returncode == 2, proc.stderr
        assert proc.stderr.startswith("map-png: error:"), proc.stderr
        assert "Traceback" not in proc.stderr
    assert not (tmp_path / "x.png").exists()


def test_cli_prints_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    _write_pcd(tmp_path / "m.pcd", _scene())
    out = tmp_path / "m.png"
    assert main([str(tmp_path / "m.pcd"), "--out", str(out), "--res-m", "0.25"]) == 0
    line = json.loads(capsys.readouterr().out)
    assert (line["n_points"], line["n_in_cut"], line["ate_rmse_m"]) == (580, 80, None)
    assert _png_size(out) == (line["width_px"], line["height_px"])
