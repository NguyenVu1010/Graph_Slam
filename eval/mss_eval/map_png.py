"""Top-down map picture: point density between two heights, with the trajectories and ATE.

Usage: python -m mss_eval map-png <map.pcd> [--est est.tum] [--gt gt.tum] --out map.png
           [--z-min-m Z] [--z-max-m Z] [--res-m R] [--width-px W] [--title TEXT]

Reads the PCD that ``mss_bench --map-out`` writes (points in the map frame ``w``, metres) and
TUM files only (ADR-003: eval never links core). Points with ``z_min_m <= z <= z_max_m``
(``w`` frame) are counted per ``res_m`` x ``res_m`` cell; the image shows log(1 + count),
dark = many points. The estimate is drawn as written (frame ``w``). The ground truth is drawn
in ``w`` too: the SE(3) fit of ``mss_eval.ate`` (``gt ~= R est + t``) is inverted on it, so the
map and the estimate are not moved. With both TUM files the title carries the ATE RMSE.

The PNG is exactly ``figure_size_px`` pixels (no tight bounding box). Prints one JSON line
(point counts, grid, image size, ATE); exit 2 on a bad input file, with the reason on stderr.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from mss_eval.ate import align
from mss_eval.tum import read_tum

DEFAULT_Z_MIN_M = 0.3  # w frame; above the floor returns (body origin at axle height)
DEFAULT_Z_MAX_M = 2.0  # w frame; below the ceilings of the sim worlds (3 m and 6 m)
DEFAULT_RES_M = 0.05  # density cell edge
DEFAULT_WIDTH_PX = 1600
MARGIN_M = 1.0  # empty border around the points and trajectories
DPI = 100  # pixels per inch of the figure (only fixes the font size against the pixels)
HEADER_PX = 90  # room for the title and the x-axis labels
MIN_HEIGHT_PX = 480  # long narrow maps (the 60 x 2.5 m corridor) still get a readable figure
MAX_HEIGHT_PX = 4000

_CELL_EPS = 1e-9  # cells; 12 / 0.05 = 240.00000000000003 must give 240 cells, not 241
_PCD_TYPES = {"F": "f", "I": "i", "U": "u"}


class MapPngError(ValueError):
    """Bad input file or argument; the message names the file."""


@dataclass(frozen=True)
class Grid:
    counts: np.ndarray  # (ny, nx) points per cell, row 0 = lowest y
    x0_m: float  # lower-left corner of cell (0, 0), w frame
    y0_m: float
    res_m: float

    @property
    def extent(self) -> tuple[float, float, float, float]:
        ny, nx = self.counts.shape
        return (self.x0_m, self.x0_m + nx * self.res_m, self.y0_m, self.y0_m + ny * self.res_m)


@dataclass(frozen=True)
class MapPngResult:
    n_points: int
    n_in_cut: int
    grid_nx: int
    grid_ny: int
    width_px: int
    height_px: int
    ate_rmse_m: float | None
    n_pairs: int | None


def read_pcd(path: Path) -> np.ndarray:
    """``(N, 3)`` float64 x, y, z of a PCD v0.7 file, ``DATA ascii`` or ``binary``."""
    path = Path(path)
    try:
        raw = path.read_bytes()
    except OSError as e:
        raise MapPngError(f"{path}: cannot read: {e.strerror or e}") from e
    header: dict[str, list[str]] = {}
    pos = 0
    while True:
        end = raw.find(b"\n", pos)
        if end < 0:
            raise MapPngError(f"{path}: not a PCD file (no DATA line)")
        line = raw[pos:end].decode("ascii", errors="replace").strip()
        pos = end + 1
        if not line or line.startswith("#"):
            continue
        key, *vals = line.split()
        header[key.upper()] = vals
        if key.upper() == "DATA":
            break
    for key in ("FIELDS", "SIZE", "TYPE", "POINTS", "DATA"):
        if key not in header:
            raise MapPngError(f"{path}: PCD header has no {key} line")
    fields, sizes, types = header["FIELDS"], header["SIZE"], header["TYPE"]
    counts = header.get("COUNT", ["1"] * len(fields))
    if not len(fields) == len(sizes) == len(types) == len(counts):
        raise MapPngError(f"{path}: FIELDS, SIZE, TYPE and COUNT lengths differ")
    if not {"x", "y", "z"} <= set(fields):
        raise MapPngError(f"{path}: PCD has no x, y, z fields (has {fields})")
    try:
        n = int(header["POINTS"][0])
        dtype = np.dtype(
            [
                (name, "<" + _PCD_TYPES[t.upper()] + s, (int(c),) if int(c) > 1 else ())
                for name, s, t, c in zip(fields, sizes, types, counts, strict=True)
            ]
        )
    except (KeyError, ValueError, TypeError, IndexError) as e:
        raise MapPngError(f"{path}: bad PCD header ({e})") from e
    data = header["DATA"][0].lower() if header["DATA"] else ""
    if data == "binary":
        need = n * dtype.itemsize
        if len(raw) - pos < need:
            raise MapPngError(
                f"{path}: truncated: {len(raw) - pos} data bytes for {n} points ({need} needed)"
            )
        rec = np.frombuffer(raw, dtype=dtype, count=n, offset=pos)
        xyz = np.stack([rec[k].astype(np.float64) for k in ("x", "y", "z")], axis=1)
    elif data == "ascii":
        rows = raw[pos:].decode("ascii", errors="replace").split("\n")
        rows = [r.split() for r in rows if r.strip()]
        if len(rows) != n:
            raise MapPngError(f"{path}: {len(rows)} ascii rows for POINTS {n}")
        starts = np.cumsum([0] + [int(c) for c in counts])  # first column of each field
        cols = [int(starts[fields.index(k)]) for k in ("x", "y", "z")]
        try:
            xyz = np.array([[float(r[c]) for c in cols] for r in rows], dtype=np.float64)
        except (ValueError, IndexError) as e:
            raise MapPngError(f"{path}: bad ascii row ({e})") from e
        xyz = xyz.reshape(n, 3)
    else:
        raise MapPngError(f"{path}: DATA {data!r} not supported (ascii or binary)")
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def density_grid(xy: np.ndarray, res_m: float, lo_m: np.ndarray, hi_m: np.ndarray) -> Grid:
    """Count ``xy`` points per ``res_m`` cell over the box ``[lo_m, hi_m]`` (x, y, metres).

    The grid starts at ``floor(lo / res) * res``; points outside the box are not counted.
    """
    x0 = math.floor(float(lo_m[0]) / res_m) * res_m
    y0 = math.floor(float(lo_m[1]) / res_m) * res_m
    nx = max(1, math.ceil((float(hi_m[0]) - x0) / res_m - _CELL_EPS))
    ny = max(1, math.ceil((float(hi_m[1]) - y0) / res_m - _CELL_EPS))
    counts = np.zeros((ny, nx), dtype=np.int64)
    if len(xy):
        ix = np.floor((xy[:, 0] - x0) / res_m).astype(np.int64)
        iy = np.floor((xy[:, 1] - y0) / res_m).astype(np.int64)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        np.add.at(counts, (iy[ok], ix[ok]), 1)
    return Grid(counts, x0, y0, res_m)


def figure_size_px(nx: int, ny: int, width_px: int) -> tuple[int, int]:
    """PNG size for a grid of ``nx`` x ``ny`` cells: the given width, height from the aspect."""
    height = round(width_px * ny / nx) + HEADER_PX
    return width_px, min(MAX_HEIGHT_PX, max(MIN_HEIGHT_PX, height))


def render_map_png(
    pcd: Path,
    out_png: Path,
    *,
    est_tum: Path | None = None,
    gt_tum: Path | None = None,
    z_min_m: float = DEFAULT_Z_MIN_M,
    z_max_m: float = DEFAULT_Z_MAX_M,
    res_m: float = DEFAULT_RES_M,
    width_px: int = DEFAULT_WIDTH_PX,
    title: str = "",
) -> MapPngResult:
    """Write the top-down density PNG of ``pcd`` (see the module docstring)."""
    if not z_min_m < z_max_m:
        raise MapPngError(f"--z-min-m {z_min_m} must be below --z-max-m {z_max_m}")
    if not res_m > 0.0 or width_px < 100:
        raise MapPngError(f"--res-m must be > 0 (got {res_m}), --width-px >= 100 ({width_px})")
    if gt_tum is not None and est_tum is None:
        raise MapPngError("--gt needs --est (the ground truth is aligned to the estimate)")
    pts = read_pcd(pcd)
    cut = pts[(pts[:, 2] >= z_min_m) & (pts[:, 2] <= z_max_m)]
    paths: list[tuple[str, np.ndarray]] = []
    ate_rmse = n_pairs = None
    if est_tum is not None:
        try:
            est = read_tum(Path(est_tum)).p
            if gt_tum is not None:
                al = align(Path(est_tum), Path(gt_tum))
                err = np.linalg.norm(al.gt - al.est_aligned, axis=1)
                ate_rmse, n_pairs = float(np.sqrt(np.mean(err * err))), len(err)
                gt_w = (read_tum(Path(gt_tum)).p - al.t) @ al.rot  # R^T (p - t), row-wise
                paths.append(("ground truth (aligned to w)", gt_w))
        except (OSError, ValueError) as e:
            raise MapPngError(str(e)) from e
        paths.append(("estimate", est))
    every = [cut[:, :2], *(p[:, :2] for _, p in paths)]
    every = [a for a in every if len(a)]
    if not every:
        raise MapPngError(f"{pcd}: no points between z {z_min_m} and {z_max_m} m, no trajectory")
    allxy = np.concatenate(every)
    grid = density_grid(cut[:, :2], res_m, allxy.min(0) - MARGIN_M, allxy.max(0) + MARGIN_M)
    ny, nx = grid.counts.shape
    w_px, h_px = figure_size_px(nx, ny, width_px)
    _draw(
        grid,
        paths,
        Path(out_png),
        (w_px, h_px),
        _title(title, len(cut), z_min_m, z_max_m, ate_rmse, n_pairs),
    )
    return MapPngResult(len(pts), len(cut), nx, ny, w_px, h_px, ate_rmse, n_pairs)


def _title(
    title: str, n_cut: int, z_min_m: float, z_max_m: float, ate: float | None, n: int | None
) -> str:
    head = f"{title}: " if title else ""
    text = f"{head}top-down map, {n_cut} points with z in [{z_min_m:g}, {z_max_m:g}] m (w)"
    if ate is not None:
        text += f"; ATE RMSE {ate:.3f} m ({n} poses)"
    return text


def _draw(
    grid: Grid, paths: list[tuple[str, np.ndarray]], out: Path, size: tuple[int, int], title: str
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(size[0] / DPI, size[1] / DPI), dpi=DPI)
    ax = fig.add_axes((0.06, 0.08, 0.92, 0.84))
    ax.imshow(
        np.log1p(grid.counts),
        origin="lower",
        extent=grid.extent,
        cmap="gray_r",
        interpolation="nearest",
        aspect="equal",
    )
    styles = {"estimate": ("tab:orange", 1.2), "ground truth (aligned to w)": ("tab:blue", 2.0)}
    for label, p in paths:
        color, width = styles.get(label, ("tab:green", 1.0))
        ax.plot(p[:, 0], p[:, 1], color=color, linewidth=width, label=label)
    if paths:
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("x_w [m]")
    ax.set_ylabel("y_w [m]")
    ax.set_title(title, fontsize=10)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=DPI)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m mss_eval map-png", description=__doc__.split("\n")[0]
    )
    ap.add_argument("pcd", type=Path, help="map from mss_bench --map-out (w frame, m)")
    ap.add_argument("--est", type=Path, help="estimate TUM (w frame)")
    ap.add_argument("--gt", type=Path, help="ground truth TUM; needs --est, adds ATE")
    ap.add_argument("--out", type=Path, required=True, help="PNG to write")
    ap.add_argument("--z-min-m", type=float, default=DEFAULT_Z_MIN_M, help="lowest z kept (w)")
    ap.add_argument("--z-max-m", type=float, default=DEFAULT_Z_MAX_M, help="highest z kept (w)")
    ap.add_argument("--res-m", type=float, default=DEFAULT_RES_M, help="density cell edge")
    ap.add_argument("--width-px", type=int, default=DEFAULT_WIDTH_PX)
    ap.add_argument("--title", default="", help="prefix of the title, e.g. the scenario id")
    args = ap.parse_args(argv)
    try:
        res = render_map_png(
            args.pcd,
            args.out,
            est_tum=args.est,
            gt_tum=args.gt,
            z_min_m=args.z_min_m,
            z_max_m=args.z_max_m,
            res_m=args.res_m,
            width_px=args.width_px,
            title=args.title,
        )
    except MapPngError as e:
        print(f"map-png: error: {e}", file=sys.stderr)
        return 2
    print(json.dumps({"png": str(args.out), **asdict(res)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
