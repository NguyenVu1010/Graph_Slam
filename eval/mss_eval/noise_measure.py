"""Per-sensor measurements for the noise report (ADR-011 section 7): declared vs measured.

Pure functions on numpy arrays; bag reading and orchestration live in ``noise_report``.
Frames: ``w`` world (assumed ENU-aligned, as in the simulator), ``b`` body, ``l`` LiDAR,
``g`` GNSS antenna. Time is int64 ns; float seconds only for ``dt``.
"""

from __future__ import annotations

import io
from dataclasses import dataclass, field

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402
from scipy.stats import chi2  # noqa: E402

from mss_eval.allan import fit_white_and_walk, oadev, theory_curve  # noqa: E402

_NS_PER_S = 1_000_000_000
_SIGMA_TOL = 0.15  # +-15 % on white sigmas (IMU, GNSS, LiDAR)
_DROPOUT_TOL = 0.20  # +-20 % relative on the LiDAR dropout fraction
_ALLAN_TOL = 0.10
_ALLAN_TAU_S = (0.01, 100.0)
_GNSS_DETREND_S = 60.0
_CHI2_P = 0.99
_COV_MATCH_REL = 0.01  # a fix "reports nominal" when its variance is within 1 % of declared
_MIN_VARIANCE_M2 = 1e-12
_OUTLIER_SIGMAS = 5.0
_ODOM_DRIFT_PCT = (1.0, 3.0)
_ODOM_YAW_DEG_PER_REV = (0.3, 1.0)
_MIN_TURN_RAD = 0.5  # below this total rotation a yaw drift per revolution is meaningless
_WGS84_A_M, _WGS84_F = 6378137.0, 1.0 / 298.257223563
_AXES = "xyz"
# Categorical slots 1-3 of the reference palette (blue, orange, aqua), then neutral ink.
_C = ("#2a78d6", "#eb6834", "#1baf7a")
_INK, _MUTED, _GRID, _SHADE = "#1b1f24", "#5b6470", "#d3d9e0", "#e6e9ee"


@dataclass
class Row:
    name: str
    declared: float | str | None
    measured: float | str | None
    tol: str
    ok: bool | None  # None = informational, never gates
    note: str = ""
    pipeline: float | str | None = None  # what --pipeline-config believes, if given


@dataclass
class Section:
    name: str
    rows: list[Row]
    figures: list[bytes] = field(default_factory=list)  # PNG
    note: str = ""


@dataclass
class Report:
    sections: list[Section]
    passed: bool


class Trajectory:
    """``T_w_b(t)``: linear position and slerp rotation, clamped to the sample range."""

    def __init__(self, t_ns: np.ndarray, p_w_b: np.ndarray, q_xyzw: np.ndarray) -> None:
        self.t_ns = np.asarray(t_ns, dtype=np.int64)
        self.p_w_b = np.asarray(p_w_b, dtype=float)
        self._t_rel = (self.t_ns - self.t_ns[0]).astype(float)  # epoch ns lose precision
        self._slerp = Slerp(self._t_rel, Rotation.from_quat(q_xyzw))  # scipy order x, y, z, w

    def _rel(self, t_ns: np.ndarray) -> np.ndarray:
        rel = (np.asarray(t_ns, dtype=np.int64) - self.t_ns[0]).astype(float)
        return np.clip(rel, 0.0, self._t_rel[-1])

    def position(self, t_ns: np.ndarray) -> np.ndarray:
        rel = self._rel(t_ns)
        return np.stack([np.interp(rel, self._t_rel, self.p_w_b[:, i]) for i in range(3)], 1)

    def rotation(self, t_ns: np.ndarray) -> np.ndarray:
        return self._slerp(self._rel(t_ns)).as_matrix()

    def yaw(self, t_ns: np.ndarray) -> np.ndarray:
        r = self.rotation(t_ns)
        return np.arctan2(r[:, 1, 0], r[:, 0, 0])


def _rel_ok(measured: float, declared: float, tol: float) -> bool:
    return bool(np.isfinite(measured) and abs(measured - declared) <= tol * declared)


def _png(fig: plt.Figure) -> bytes:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, facecolor="white")
    plt.close(fig)
    return buf.getvalue()


def _axes(ncols: int, width_in: float = 10.0) -> tuple[plt.Figure, list[plt.Axes]]:
    fig, ax = plt.subplots(1, ncols, figsize=(width_in, 3.4), squeeze=False)
    for a in ax.flat:
        a.grid(True, color=_GRID, linewidth=0.6)
        a.tick_params(colors=_MUTED, labelsize=8)
        for s in a.spines.values():
            s.set_color(_GRID)
    fig.set_layout_engine("constrained")
    return fig, list(ax[0])


def _label(a: plt.Axes, title: str, xlabel: str, ylabel: str = "") -> None:
    a.set_title(title, fontsize=10, color=_INK, loc="left")
    a.set_xlabel(xlabel, fontsize=9, color=_MUTED)
    a.set_ylabel(ylabel, fontsize=9, color=_MUTED)


def _hist_vs_gauss(a: plt.Axes, x: np.ndarray, sigma: float, lim: float, label: str) -> None:
    a.hist(x, bins=40, range=(-lim, lim), density=True, color=_C[0], rwidth=0.9)
    g = np.linspace(-lim, lim, 200)
    pdf = np.exp(-0.5 * (g / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi))
    a.plot(g, pdf, color=_INK, linestyle="--", linewidth=1.5, label=label)
    a.legend(fontsize=8, frameon=False)


# ---------------------------------------------------------------- IMU


def white_sigma(x: np.ndarray) -> np.ndarray:
    """Per-column white-noise sigma from first differences: ``std(diff) / sqrt(2)``.

    Insensitive to slow signal (motion, bias drift), so it also works on a moving robot.
    """
    return np.std(np.diff(x, axis=0), axis=0, ddof=1) / np.sqrt(2.0)


def imu_section(
    t_ns: np.ndarray,
    gyro: np.ndarray,
    acc: np.ndarray,
    imu_cfg: dict,
    *,
    static: bool,
    static_s: float,
    allan_gate: bool,
    pipeline_imu: dict | None,
) -> Section:
    rate = float(imu_cfg["rate_hz"])
    window = (t_ns - t_ns[0]) <= int(static_s * _NS_PER_S)
    rows: list[Row] = []
    allan: list[tuple[str, list, float, float]] = []
    for name, data, n_key, k_key in (
        ("gyro", gyro, "gyro_noise_density", "gyro_random_walk"),
        ("acc", acc, "acc_noise_density", "acc_random_walk"),
    ):
        n_decl, k_decl = float(imu_cfg[n_key]), float(imu_cfg[k_key])
        sigma_decl = n_decl * np.sqrt(rate)  # discrete sigma = N * sqrt(rate)
        believed = None
        if pipeline_imu and n_key in pipeline_imu:
            believed = float(pipeline_imu[n_key]) * np.sqrt(rate)
        sig = white_sigma(data[window]) if window.sum() > 2 else np.full(3, np.nan)
        note = f"first {static_s:g} s" if static else "no static segment"
        for i, ax in enumerate(_AXES):
            m = float(sig[i])
            ok = _rel_ok(m, sigma_decl, _SIGMA_TOL) if static else None
            rows.append(Row(f"{name}_sigma_{ax}", sigma_decl, m, "±15 %", ok, note, believed))
        curves = [oadev(data[:, i], rate) for i in range(3)]
        devs, fits = [], []
        for taus, adev in curves:
            band = (taus >= _ALLAN_TAU_S[0]) & (taus <= _ALLAN_TAU_S[1])
            th = theory_curve(taus[band], n_decl, k_decl)
            devs.append(float(np.max(np.abs(adev[band] / th - 1.0))) if band.any() else np.nan)
            fits.append(fit_white_and_walk(taus, adev))
        dev = float(np.nanmax(devs)) if np.isfinite(devs).any() else float("nan")
        rows.append(
            Row(
                f"{name}_allan_max_dev",
                "theory(N, K)",
                dev,
                "< 10 % on τ ∈ [0.01, 100] s",
                bool(dev < _ALLAN_TOL) if allan_gate else None,
                "" if allan_gate else "shown only; gated with --allan-gate",
            )
        )
        k_fit = [f[1] for f in fits if np.isfinite(f[1])]
        n_fit = float(np.median([f[0] for f in fits]))
        rows.append(Row(f"{name}_N_fit", n_decl, n_fit, "informational", None, "median of axes"))
        k_meas = float(np.median(k_fit)) if k_fit else "too short"
        rows.append(Row(f"{name}_K_fit", k_decl, k_meas, "informational", None))
        allan.append((name, curves, n_decl, k_decl))
    figures = [_allan_figure(allan)]
    if static:
        figures.append(_static_hist(gyro[window], acc[window], imu_cfg, rate))
    note = (
        "Gazebo models the bias as first-order Gauss-Markov (τ = 1000 s), not a pure random "
        "walk, so the Allan comparison is meaningful only on τ ∈ [0.01, 100] s (ADR-011 §3). "
        "White σ uses first differences, std(Δx)/√2."
    )
    return Section("IMU", rows, figures, note)


def _allan_figure(per_sensor: list[tuple[str, list, float, float]]) -> bytes:
    fig, axes = _axes(2)
    units = {"gyro": "rad/s", "acc": "m/s²"}
    for a, (name, curves, n_decl, k_decl) in zip(axes, per_sensor, strict=True):
        a.axvspan(*_ALLAN_TAU_S, color=_SHADE, zorder=0)
        for i, (taus, adev) in enumerate(curves):
            a.loglog(taus, adev, color=_C[i], linewidth=2, marker="o", markersize=3, label=_AXES[i])
        all_t = np.concatenate([c[0] for c in curves])
        grid = np.geomspace(all_t.min(), all_t.max(), 50)
        th = theory_curve(grid, n_decl, k_decl)
        a.loglog(grid, th, color=_INK, linestyle="--", linewidth=1.5, label="declared N, K")
        a.set_xlim(all_t.min() / 1.5, all_t.max() * 1.5)  # the shaded gate band may be wider
        a.legend(fontsize=8, frameon=False)
        _label(a, f"{name}: Allan deviation per axis", "τ [s]", f"σ(τ) [{units[name]}]")
    return _png(fig)


def _static_hist(gyro: np.ndarray, acc: np.ndarray, imu_cfg: dict, rate: float) -> bytes:
    fig, axes = _axes(2)
    pairs = (("gyro", gyro, "gyro_noise_density"), ("acc", acc, "acc_noise_density"))
    for a, (name, data, key) in zip(axes, pairs, strict=True):
        z = (data - data.mean(axis=0)) / (float(imu_cfg[key]) * np.sqrt(rate))
        _hist_vs_gauss(a, z.ravel(), 1.0, 4.0, "declared Gaussian")
        _label(a, f"{name}: static samples", "sample / declared σ (all axes)")
    return _png(fig)


# ---------------------------------------------------------------- GNSS


def lla_to_enu(lla: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """WGS-84 geodetic (deg, deg, m), shape (N, 3), to ENU (m) about ``origin`` (3,)."""
    e2 = _WGS84_F * (2.0 - _WGS84_F)

    def ecef(x: np.ndarray) -> np.ndarray:
        lat, lon, h = np.deg2rad(x[:, 0]), np.deg2rad(x[:, 1]), x[:, 2]
        n = _WGS84_A_M / np.sqrt(1.0 - e2 * np.sin(lat) ** 2)
        return np.stack(
            [
                (n + h) * np.cos(lat) * np.cos(lon),
                (n + h) * np.cos(lat) * np.sin(lon),
                (n * (1.0 - e2) + h) * np.sin(lat),
            ],
            axis=1,
        )

    o = np.asarray(origin, dtype=float)
    lat, lon = np.deg2rad(o[0]), np.deg2rad(o[1])
    r_e_ecef = np.array(
        [
            [-np.sin(lon), np.cos(lon), 0.0],
            [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
            [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)],
        ]
    )
    return (ecef(np.asarray(lla, dtype=float)) - ecef(o[None, :])) @ r_e_ecef.T


def detrend(t_ns: np.ndarray, e: np.ndarray, window_s: float = _GNSS_DETREND_S) -> np.ndarray:
    """Subtract a centred ``window_s`` moving average (time-based), or the mean if shorter."""
    t = (t_ns - t_ns[0]) / _NS_PER_S
    if t[-1] < window_s:
        return e - e.mean(axis=0)
    csum = np.vstack([np.zeros((1, e.shape[1])), np.cumsum(e, axis=0)])
    lo = np.searchsorted(t, t - window_s / 2, side="left")
    hi = np.searchsorted(t, t + window_s / 2, side="right")
    return e - (csum[hi] - csum[lo]) / (hi - lo)[:, None]


def gnss_section(
    t_ns: np.ndarray,
    lla: np.ndarray,
    cov_diag: np.ndarray,
    traj: Trajectory,
    gnss_cfg: dict,
    p_b_g: np.ndarray,
    pipeline_gnss: dict | None,
) -> Section:
    sh, sv = float(gnss_cfg["sigma_h_m"]), float(gnss_cfg["sigma_v_m"])
    enu = lla_to_enu(lla, lla[0])  # origin = first fix
    p_w_g = traj.position(t_ns) + traj.rotation(t_ns) @ p_b_g  # p_w_g = p_w_b + R_w_b p_b_g
    err = enu - p_w_g  # includes the constant first-fix offset, removed by detrend
    nominal = np.isclose(cov_diag[:, 0], sh**2, rtol=_COV_MATCH_REL)
    n_nom = int(nominal.sum())
    res = detrend(t_ns[nominal], err[nominal]) if n_nom > 2 else np.full((2, 3), np.nan)
    m_h = float(np.sqrt((np.var(res[:, 0], ddof=1) + np.var(res[:, 1], ddof=1)) / 2))
    m_v = float(np.std(res[:, 2], ddof=1))
    believed = pipeline_gnss or {}
    note = "drift removed: 60 s moving average (the mean if shorter)"
    rows = [Row("fixes_nominal_cov", None, f"{n_nom} / {len(t_ns)}", "informational", None)]
    for name, decl, meas in (("sigma_h_m", sh, m_h), ("sigma_v_m", sv, m_v)):
        ok = _rel_ok(meas, decl, _SIGMA_TOL)
        rows.append(Row(name, decl, meas, "±15 %", ok, note, believed.get(name)))
    res_all = detrend(t_ns, err)
    nis = np.sum(res_all**2 / np.maximum(cov_diag, _MIN_VARIANCE_M2), axis=1)
    frac = float(np.mean(nis > chi2.ppf(_CHI2_P, 3)))
    nis_note = "drift-removed error normalised by the reported covariance, 3 dof"
    rows.append(
        Row("chi2_99_reject_fraction", 1.0 - _CHI2_P, frac, "informational", None, nis_note)
    )
    note = (
        "Multipath keeps the nominal covariance and cannot be separated without the scenario; "
        "degraded fixes are excluded from σ by their reported covariance. Shaded: message gaps."
    )
    return Section("GNSS", rows, [_gnss_figure(t_ns, res_all, res, sh)], note)


def _gnss_figure(t_ns: np.ndarray, res_all: np.ndarray, res: np.ndarray, sh: float) -> bytes:
    fig, (a, b) = _axes(2)
    t = (t_ns - t_ns[0]) / _NS_PER_S
    if len(t) > 1:
        period = float(np.median(np.diff(t)))
        for i in np.flatnonzero(np.diff(t) > 2.0 * period):
            a.axvspan(t[i], t[i + 1], color=_SHADE, zorder=0)
    for i, lab in enumerate("ENU"):
        a.plot(t, res_all[:, i], color=_C[i], linewidth=2, label=lab)
    a.legend(fontsize=8, frameon=False, ncols=3)
    _label(a, "GNSS error vs ground truth", "t [s]", "error, drift removed [m]")
    _hist_vs_gauss(b, res[:, :2].ravel(), sh, 4.0 * sh, f"declared σ_h = {sh:g} m")
    _label(b, "Horizontal error (E and N)", "error [m]")
    return _png(fig)


# ---------------------------------------------------------------- LiDAR


def estimate_range_sigma(distances: np.ndarray, sigma_declared: float) -> float:
    """Std of point-to-plane distances, excluding |d| > 5 sigma_declared (other surfaces)."""
    d = np.asarray(distances, dtype=float)
    d = d[np.abs(d) <= _OUTLIER_SIGMAS * sigma_declared]
    return float(np.std(d, ddof=1)) if len(d) > 1 else float("nan")


def plane_distances(p_w: np.ndarray, normals: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    """Signed distance of each point to its nearest plane ``n . p = d``."""
    d = p_w @ normals.T - offsets[None, :]
    return d[np.arange(len(d)), np.argmin(np.abs(d), axis=1)]


def scan_to_world(
    xyz: np.ndarray,
    t_ns: np.ndarray,
    traj: Trajectory,
    T_b_l: np.ndarray,  # noqa: N803
) -> np.ndarray:
    """``p_w = R_w_b(t_i) (R_b_l p_l + p_b_l) + p_w_b(t_i)`` per point."""
    p_b = xyz.astype(float) @ T_b_l[:3, :3].T + T_b_l[:3, 3]
    return np.einsum("nij,nj->ni", traj.rotation(t_ns), p_b) + traj.position(t_ns)


def lidar_section(
    distances: np.ndarray, n_nan: int, n_not_inf: int, lidar_cfg: dict, pipeline_lidar: dict | None
) -> Section:
    s_decl, p_decl = float(lidar_cfg["range_sigma_m"]), float(lidar_cfg["dropout_p"])
    sigma = estimate_range_sigma(distances, s_decl)
    frac = n_nan / n_not_inf if n_not_inf else float("nan")
    rows = [
        Row(
            "range_sigma_m",
            s_decl,
            sigma,
            "±15 %",
            _rel_ok(sigma, s_decl, _SIGMA_TOL),
            f"|d| ≤ {_OUTLIER_SIGMAS:g} σ_declared; {len(distances)} points",
            (pipeline_lidar or {}).get("range_sigma_m"),
        ),
        Row(
            "dropout_fraction",
            p_decl,
            frac,
            "±20 % rel.",
            _rel_ok(frac, p_decl, _DROPOUT_TOL),
            "NaN share among points that are not inf (non-returns)",
        ),
    ]
    fig, (a,) = _axes(1, width_in=6.5)
    lim = _OUTLIER_SIGMAS * s_decl
    _hist_vs_gauss(a, distances, s_decl, lim, f"declared σ = {s_decl:g} m")
    _label(a, "LiDAR point-to-plane distance", "distance [m]")
    return Section("LiDAR", rows, [_png(fig)])


# ---------------------------------------------------------------- wheel odom


def integrate_odom(
    t_ns: np.ndarray, v: np.ndarray, w: np.ndarray, start: tuple[float, float, float]
) -> np.ndarray:
    """Midpoint-rule planar integration of body twists from ``start = (x, y, yaw)``.

    Returns (N, 3) ``[x, y, yaw]``; the twist at sample ``i`` drives ``[t_{i-1}, t_i]``.
    """
    out = np.empty((len(t_ns), 3))
    out[0] = start
    dt = np.diff(t_ns) / _NS_PER_S
    for i, h in enumerate(dt):
        x, y, yaw = out[i]
        mid = yaw + w[i + 1] * h / 2.0
        step = v[i + 1] * h
        out[i + 1] = (x + step * np.cos(mid), y + step * np.sin(mid), yaw + w[i + 1] * h)
    return out


def odom_section(
    t_ns: np.ndarray, v: np.ndarray, w: np.ndarray, traj: Trajectory, *, odom_gate: bool
) -> Section:
    p_gt = traj.position(t_ns)
    yaw_gt = np.unwrap(traj.yaw(t_ns))
    odo = integrate_odom(t_ns, v, w, (p_gt[0, 0], p_gt[0, 1], yaw_gt[0]))
    dist = float(np.sum(np.linalg.norm(np.diff(p_gt[:, :2], axis=0), axis=1)))
    end_err = float(np.linalg.norm(odo[-1, :2] - p_gt[-1, :2]))
    drift = 100.0 * end_err / dist if dist > 0 else float("nan")
    turn = float(yaw_gt[-1] - yaw_gt[0])
    yaw_err_deg = float(np.degrees(odo[-1, 2] - yaw_gt[-1]))
    per_rev: float | str = "n/a"
    if abs(turn) > _MIN_TURN_RAD:
        per_rev = abs(yaw_err_deg) / (abs(turn) / (2 * np.pi))
    lo, hi = _ODOM_DRIFT_PCT
    rows = [
        Row("distance_m", None, dist, "informational", None),
        Row("end_error_m", None, end_err, "informational", None),
        Row(
            "drift_pct_of_distance",
            f"{lo:g}–{hi:g}",
            drift,
            f"[{lo:g}, {hi:g}] %",
            bool(lo <= drift <= hi) if odom_gate else None,
            "" if odom_gate else "gated with --odom-gate",
        ),
        Row(
            "yaw_drift_deg_per_rev",
            "{:g}–{:g}".format(*_ODOM_YAW_DEG_PER_REV),
            per_rev,
            "informational",
            None,
            f"total turn {np.degrees(turn):.1f}°",
        ),
    ]
    fig, (a,) = _axes(1, width_in=6.5)
    a.plot(p_gt[:, 0], p_gt[:, 1], color=_INK, linewidth=2, label="ground truth")
    a.plot(odo[:, 0], odo[:, 1], color=_C[1], linewidth=2, label="odom, integrated")
    a.set_aspect("equal", adjustable="datalim")
    a.legend(fontsize=8, frameon=False)
    _label(a, "Wheel odometry vs ground truth", "x [m]", "y [m]")
    note = "Target 1–3 % of distance on a warehouse floor, yaw 0.3–1.0° per turn (ADR-011 §3)."
    return Section("Wheel odom", rows, [_png(fig)], note)
