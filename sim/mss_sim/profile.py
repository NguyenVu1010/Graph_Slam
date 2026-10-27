"""Sensor noise profile: frozen dataclasses plus a strict YAML loader (ADR-011 section 3)."""

from __future__ import annotations

import dataclasses
import math
import typing
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ProfileError(ValueError):
    """Invalid noise profile. The message names the full dotted key path."""


@dataclass(frozen=True)
class ImuNoise:
    rate_hz: float
    gyro_noise_density: float  # rad/s/sqrt(Hz)
    acc_noise_density: float  # m/s^2/sqrt(Hz)
    gyro_random_walk: float  # rad/s^2/sqrt(Hz)
    acc_random_walk: float  # m/s^3/sqrt(Hz)
    gyro_bias_sigma_radps: float
    acc_bias_sigma_mps2: float
    bias_corr_time_s: float


@dataclass(frozen=True)
class GnssNoise:
    rate_hz: float
    sigma_h_m: float
    sigma_v_m: float
    drift_sigma_m: float
    drift_tau_s: float
    multipath_amp_min_m: float
    multipath_amp_max_m: float
    multipath_hold_min_s: float
    multipath_hold_max_s: float
    multipath_start_rate_per_s: float
    degraded_factor: float


@dataclass(frozen=True)
class LidarNoise:
    range_sigma_m: float  # range noise the noise layer adds (Gazebo LiDAR noise), m
    # Noise report only (eval/mss_eval/noise_measure.py): a point whose grid neighbour lies more
    # than edge_jump_m from the point's reference plane (or has no return) is a silhouette point;
    # points with |n.u| below min_incidence_cos are not used for the range sigma.
    edge_jump_m: float
    min_incidence_cos: float
    # What the renderer adds on top of range_sigma_m, measured (sim/README.md "LiDAR render
    # error"): {form: none | inv_cos_minus_1 | tan (k_m) | table (sigma_by_incidence_m:
    # {world: [[cos_lo, cos_hi, sigma_m], ...]})}.
    render_model: dict
    dropout_p: float
    dropout_p_grazing: float
    grazing_deg: float
    scan_period_s: float
    nan_fraction: float
    near_fraction: float
    min_range_m: float


@dataclass(frozen=True)
class Kinematics:
    wheel_radius_m: float
    track_m: float


@dataclass(frozen=True)
class OdomNoise:
    rate_hz: float  # encoder read rate = /wheel/odom rate, whatever the joint-state rate
    ticks_per_rev: int
    speed_sigma_mps: float
    kinematics_true: Kinematics
    kinematics_believed: Kinematics


@dataclass(frozen=True)
class NoiseProfile:
    imu: ImuNoise
    gnss: GnssNoise
    lidar: LidarNoise
    odom: OdomNoise
    clock_offset_ns: dict[str, int]
    inflation_max: float


# Float fields that must be strictly positive; every other float must be >= 0.
_STRICTLY_POSITIVE = {"rate_hz", "ticks_per_rev", "scan_period_s"}
_CLOCK_KEYS = ("imu", "lidar", "gnss", "odom")
# Probabilities / fractions that must lie in [0, 1].
_UNIT_INTERVAL = {
    "dropout_p",
    "dropout_p_grazing",
    "nan_fraction",
    "near_fraction",
    "min_incidence_cos",
}
_RENDER_FORMS = ("none", "inv_cos_minus_1", "tan", "table")
# (min, max) field pairs that must satisfy min <= max.
_ORDERED_PAIRS = (
    ("multipath_amp_min_m", "multipath_amp_max_m"),
    ("multipath_hold_min_s", "multipath_hold_max_s"),
)


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _build[T](cls: type[T], mapping: Any, prefix: str) -> T:
    """Build dataclass `cls` from `mapping`, reporting missing/extra/invalid keys by dotted path."""
    where = prefix.rstrip(".") or "<root>"
    if not isinstance(mapping, dict):
        raise ProfileError(f"{where}: expected a mapping")
    names = [f.name for f in dataclasses.fields(cls)]  # type: ignore[arg-type]
    for name in names:
        if name not in mapping:
            raise ProfileError(f"{prefix}{name}: missing")
    for key in mapping:
        if key not in names:
            raise ProfileError(f"{prefix}{key}: unknown key")
    hints = typing.get_type_hints(cls)
    kwargs: dict[str, Any] = {}
    for name in names:
        kwargs[name] = _convert(name, hints[name], mapping[name], prefix + name)
    for lo, hi in _ORDERED_PAIRS:
        if lo in kwargs and hi in kwargs and not kwargs[lo] <= kwargs[hi]:
            msg = f"{prefix}{lo}: must be <= {prefix}{hi}, got {kwargs[lo]!r} > {kwargs[hi]!r}"
            raise ProfileError(msg)
    return cls(**kwargs)


def _render_model(value: Any, path: str) -> dict:
    """Validate ``lidar.render_model`` (see ``LidarNoise``)."""
    if not isinstance(value, dict) or value.get("form") not in _RENDER_FORMS:
        raise ProfileError(f"{path}.form: must be one of {_RENDER_FORMS}")
    form = value["form"]
    keys = {"none": {"form"}, "table": {"form", "sigma_by_incidence_m"}}.get(form, {"form", "k_m"})
    if set(value) != keys:
        raise ProfileError(f"{path}: form {form!r} takes exactly the keys {sorted(keys)}")
    if "k_m" in keys and not (_is_number(value["k_m"]) and value["k_m"] >= 0):
        raise ProfileError(f"{path}.k_m: must be a number >= 0")
    if form == "table":
        tables = value["sigma_by_incidence_m"]
        if not isinstance(tables, dict) or not tables:
            raise ProfileError(f"{path}.sigma_by_incidence_m: expected {{world: rows}}")
        for world, rows in tables.items():
            where = f"{path}.sigma_by_incidence_m.{world}"
            if not isinstance(rows, list) or not rows:
                raise ProfileError(f"{where}: expected a list of [cos_lo, cos_hi, sigma_m]")
            for i, row in enumerate(rows):
                ok = isinstance(row, list) and len(row) == 3 and all(_is_number(v) for v in row)
                if not ok or not (0 <= row[0] < row[1] <= 1 and row[2] >= 0):
                    raise ProfileError(f"{where}[{i}]: need 0 <= cos_lo < cos_hi <= 1, sigma >= 0")
    return value


def _convert(name: str, hint: Any, value: Any, path: str) -> Any:
    if dataclasses.is_dataclass(hint):
        return _build(hint, value, path + ".")
    if hint is dict and name == "render_model":
        return _render_model(value, path)
    if hint is float:
        if not _is_number(value):
            raise ProfileError(f"{path}: expected a number, got {value!r}")
        number = float(value)
        if not math.isfinite(number):
            raise ProfileError(f"{path}: must be finite, got {value!r}")
    elif hint is int:
        if not isinstance(value, int) or isinstance(value, bool):
            raise ProfileError(f"{path}: expected an integer, got {value!r}")
        number = value
    else:
        raise ProfileError(f"{path}: unsupported type {hint}")  # pragma: no cover
    if name in _STRICTLY_POSITIVE:
        if not number > 0:
            raise ProfileError(f"{path}: must be > 0, got {value!r}")
    elif not number >= 0:
        raise ProfileError(f"{path}: must be >= 0, got {value!r}")
    if name in _UNIT_INTERVAL and not number <= 1:
        raise ProfileError(f"{path}: must be in [0, 1], got {value!r}")
    return number


def _clock_offsets(value: Any) -> dict[str, int]:
    if not isinstance(value, dict):
        raise ProfileError("clock_offset_ns: expected a mapping")
    for key in _CLOCK_KEYS:
        if key not in value:
            raise ProfileError(f"clock_offset_ns.{key}: missing")
    for key, offset in value.items():
        if key not in _CLOCK_KEYS:
            raise ProfileError(f"clock_offset_ns.{key}: unknown key")
        if not isinstance(offset, int) or isinstance(offset, bool):
            raise ProfileError(f"clock_offset_ns.{key}: expected an integer, got {offset!r}")
    return {key: value[key] for key in _CLOCK_KEYS}


def load_profile(path: Path) -> NoiseProfile:
    """Load and validate a noise profile YAML file."""
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ProfileError("<root>: expected a mapping")
    top = {"imu", "gnss", "lidar", "odom", "clock_offset_ns", "inflation_max"}
    for key in sorted(top - data.keys()):
        raise ProfileError(f"{key}: missing")
    for key in data:
        if key not in top:
            raise ProfileError(f"{key}: unknown key")
    inflation = data["inflation_max"]
    if not _is_number(inflation):
        raise ProfileError(f"inflation_max: expected a number, got {inflation!r}")
    if not inflation >= 1:
        raise ProfileError(f"inflation_max: must be >= 1, got {inflation!r}")
    return NoiseProfile(
        imu=_build(ImuNoise, data["imu"], "imu."),
        gnss=_build(GnssNoise, data["gnss"], "gnss."),
        lidar=_build(LidarNoise, data["lidar"], "lidar."),
        odom=_build(OdomNoise, data["odom"], "odom."),
        clock_offset_ns=_clock_offsets(data["clock_offset_ns"]),
        inflation_max=float(inflation),
    )
