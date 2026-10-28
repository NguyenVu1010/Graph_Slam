"""Scenario file loader (ADR-011 section 5). Validation errors name the dotted key path."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, cast

import yaml
from shapely.geometry import Polygon

from mss_sim.geodesy import Lla

GnssMode = Literal["nominal", "outage", "multipath", "degraded"]
_MODES = ("nominal", "outage", "multipath", "degraded")
_REPO = Path(__file__).resolve().parents[2]
_TOP = {
    "id",
    "world",
    "profile",
    "pipeline_config",
    "trajectory",
    "seed",
    "duration_s",
    "origin_lla",
    "gnss",
    "gnss_zones",
    "topics",
}
_OPTIONAL = frozenset({"prep", "pipeline_config_variants"})


class ScenarioError(ValueError):
    """Invalid scenario. The message names the full dotted key path."""


@dataclass(frozen=True)
class GnssSegment:
    mode: GnssMode
    t_start_s: float
    t_end_s: float


@dataclass(frozen=True)
class GnssZone:
    mode: GnssMode
    polygon_enu: list[tuple[float, float]] = field(default_factory=list)

    def polygon(self) -> Polygon:
        return Polygon(self.polygon_enu)


@dataclass(frozen=True)
class PrepMutate:
    """``mutate_bag.mutate`` arguments that turn the E-01 bag into the E-03 bag."""

    drop_fraction: float
    reorder_fraction: float
    window_ns: int
    seed: int


@dataclass(frozen=True)
class Prep:
    """Synthetic data for the integration harness (ADR-011 part A, section 5)."""

    static_s: float
    ramp_s: float
    speed_mps: float
    omega_radps: float
    box_half_m: tuple[float, float, float]
    mutate: PrepMutate | None  # None: no E-03 data (the harness then refuses E-03)


@dataclass(frozen=True)
class Route:
    """Driver route (``sim/trajectories/*.yaml``): world-frame waypoints, metres."""

    waypoints: tuple[tuple[float, float], ...]
    speed_mps: float  # cruise target before the +-10 % variation
    lookahead_m: float  # pure-pursuit lookahead distance


@dataclass(frozen=True)
class Scenario:
    id: str
    world: str
    profile: Path
    pipeline_config: Path
    trajectory: Path
    seed: int
    duration_s: float
    origin_lla: Lla
    gnss: list[GnssSegment]
    gnss_zones: list[GnssZone]
    topics: list[str]
    prep: Prep | None = None
    route: Route | None = None  # the loaded ``trajectory`` file
    # Extra pipeline overlays run on the same bag, e.g. degeneracy on/off (ADR-011 section 5).
    pipeline_config_variants: dict[str, Path] = field(default_factory=dict)


def _num(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{path}: expected a number, got {value!r}")
    return float(value)


def _str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ScenarioError(f"{path}: expected a non-empty string, got {value!r}")
    return value


def _mapping(value: Any, path: str, keys: set[str], optional: frozenset[str] = frozenset()) -> dict:
    if not isinstance(value, dict):
        raise ScenarioError(f"{path}: expected a mapping")
    for key in sorted(keys - value.keys()):
        raise ScenarioError(f"{path}.{key}: missing")
    for key in value:
        if key not in keys and key not in optional:
            raise ScenarioError(f"{path}.{key}: unknown key")
    return value


def _pos(value: Any, path: str) -> float:
    x = _num(value, path)
    if not (math.isfinite(x) and x > 0):
        raise ScenarioError(f"{path}: must be a finite number > 0, got {value!r}")
    return x


def _fraction(value: Any, path: str) -> float:
    x = _num(value, path)
    if not 0 <= x <= 1:
        raise ScenarioError(f"{path}: must be in [0, 1], got {value!r}")
    return x


def _int(value: Any, path: str, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ScenarioError(f"{path}: expected an integer >= {minimum}, got {value!r}")
    return value


def _mutate(raw: Any) -> PrepMutate:
    keys = {"drop_fraction", "reorder_fraction", "window_ns", "seed"}
    m = _mapping(raw, "prep.mutate", keys)
    out = PrepMutate(
        drop_fraction=_fraction(m["drop_fraction"], "prep.mutate.drop_fraction"),
        reorder_fraction=_fraction(m["reorder_fraction"], "prep.mutate.reorder_fraction"),
        window_ns=_int(m["window_ns"], "prep.mutate.window_ns", 1),
        seed=_int(m["seed"], "prep.mutate.seed", 0),
    )
    if out.drop_fraction == 0 and out.reorder_fraction == 0:
        raise ScenarioError(
            "prep.mutate: at least one of drop_fraction, reorder_fraction must be > 0"
        )
    return out


def _prep(raw: Any) -> Prep:
    keys = {"static_s", "ramp_s", "speed_mps", "omega_radps", "box_half_m"}
    d = _mapping(raw, "prep", keys, frozenset({"mutate"}))
    box = _list(d["box_half_m"], "prep.box_half_m")
    if len(box) != 3:
        raise ScenarioError(f"prep.box_half_m: expected [x_m, y_m, z_m], got {box!r}")
    return Prep(
        static_s=_pos(d["static_s"], "prep.static_s"),
        ramp_s=_pos(d["ramp_s"], "prep.ramp_s"),
        speed_mps=_pos(d["speed_mps"], "prep.speed_mps"),
        omega_radps=_pos(d["omega_radps"], "prep.omega_radps"),
        box_half_m=(
            _pos(box[0], "prep.box_half_m[0]"),
            _pos(box[1], "prep.box_half_m[1]"),
            _pos(box[2], "prep.box_half_m[2]"),
        ),
        mutate=_mutate(d["mutate"]) if "mutate" in d else None,
    )


def _mode(value: Any, path: str) -> GnssMode:
    if value not in _MODES:
        raise ScenarioError(f"{path}: must be one of {_MODES}, got {value!r}")
    return cast(GnssMode, value)


def _list(value: Any, path: str) -> list:
    if not isinstance(value, list):
        raise ScenarioError(f"{path}: expected a list")
    return value


def _segments(raw: Any) -> list[GnssSegment]:
    segments = []
    for i, item in enumerate(_list(raw, "gnss")):
        p = f"gnss[{i}]"
        item = _mapping(item, p, {"mode", "t_start_s", "t_end_s"})
        start = _num(item["t_start_s"], f"{p}.t_start_s")
        end = _num(item["t_end_s"], f"{p}.t_end_s")
        if not 0 <= start < end:
            raise ScenarioError(f"{p}.t_start_s: need 0 <= t_start_s < t_end_s, got {start}, {end}")
        segments.append(GnssSegment(_mode(item["mode"], f"{p}.mode"), start, end))
    ordered = sorted(range(len(segments)), key=lambda i: segments[i].t_start_s)
    for prev, cur in zip(ordered, ordered[1:], strict=False):
        if segments[cur].t_start_s < segments[prev].t_end_s:
            raise ScenarioError(
                f"gnss[{cur}].t_start_s: overlaps gnss[{prev}] "
                f"({segments[cur].t_start_s} < {segments[prev].t_end_s})"
            )
    return segments


def _zones(raw: Any) -> list[GnssZone]:
    zones = []
    for i, item in enumerate(_list(raw, "gnss_zones")):
        p = f"gnss_zones[{i}]"
        item = _mapping(item, p, {"mode", "polygon_enu"})
        mode = _mode(item["mode"], f"{p}.mode")
        verts = _list(item["polygon_enu"], f"{p}.polygon_enu")
        if len(verts) < 3:
            raise ScenarioError(f"{p}.polygon_enu: needs >= 3 vertices, got {len(verts)}")
        points = []
        for j, v in enumerate(verts):
            if not isinstance(v, list) or len(v) != 2:
                raise ScenarioError(f"{p}.polygon_enu[{j}]: expected [east_m, north_m]")

            points.append(
                (_num(v[0], f"{p}.polygon_enu[{j}]"), _num(v[1], f"{p}.polygon_enu[{j}]"))
            )
        if not Polygon(points).is_valid:
            raise ScenarioError(f"{p}.polygon_enu: not a valid (simple, non-degenerate) polygon")
        zones.append(GnssZone(mode, points))
    return zones


def load_route(path: Path) -> Route:
    """Load and validate a trajectory YAML (``waypoints``, ``speed_mps``, ``lookahead_m``)."""
    where = f"trajectory ({Path(path).name})"
    if not Path(path).is_file():
        raise ScenarioError(f"{where}: file not found")
    data = _mapping(
        yaml.safe_load(Path(path).read_text()), where, {"waypoints", "speed_mps", "lookahead_m"}
    )
    raw = _list(data["waypoints"], f"{where}.waypoints")
    if len(raw) < 2:
        raise ScenarioError(f"{where}.waypoints: needs >= 2 waypoints, got {len(raw)}")
    points: list[tuple[float, float]] = []
    for i, v in enumerate(raw):
        p = f"{where}.waypoints[{i}]"
        if not isinstance(v, list) or len(v) != 2:
            raise ScenarioError(f"{p}: expected [x_m, y_m]")
        xy = (_num(v[0], p), _num(v[1], p))
        if not all(math.isfinite(c) for c in xy):
            raise ScenarioError(f"{p}: must be finite, got {v!r}")
        if points and xy == points[-1]:
            raise ScenarioError(f"{p}: repeats the previous waypoint")
        points.append(xy)
    return Route(
        waypoints=tuple(points),
        speed_mps=_pos(data["speed_mps"], f"{where}.speed_mps"),
        lookahead_m=_pos(data["lookahead_m"], f"{where}.lookahead_m"),
    )


def _variants(raw: Any) -> dict[str, Path]:
    if not isinstance(raw, dict) or not raw:
        raise ScenarioError("pipeline_config_variants: expected a non-empty mapping name -> path")
    return {
        _str(k, "pipeline_config_variants"): _REPO / _str(v, f"pipeline_config_variants.{k}")
        for k, v in raw.items()
    }


def load_scenario(path: Path) -> Scenario:
    """Load and validate a scenario YAML.

    Referenced files are resolved against the repo root; the trajectory is loaded and validated
    (``route``), the profile and pipeline configs need not exist.
    """
    data = yaml.safe_load(Path(path).read_text())
    data = _mapping(data, "<root>", _TOP, _OPTIONAL)
    seed = data["seed"]
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ScenarioError(f"seed: expected an integer, got {seed!r}")
    duration = _num(data["duration_s"], "duration_s")
    if not duration > 0:
        raise ScenarioError(f"duration_s: must be > 0, got {duration}")
    lla_raw = _list(data["origin_lla"], "origin_lla")
    if len(lla_raw) != 3:
        raise ScenarioError("origin_lla: expected [lat_deg, lon_deg, alt_m]")
    lla = (
        _num(lla_raw[0], "origin_lla[0]"),
        _num(lla_raw[1], "origin_lla[1]"),
        _num(lla_raw[2], "origin_lla[2]"),
    )
    if not (-90 <= lla[0] <= 90 and -180 <= lla[1] <= 180):
        raise ScenarioError(f"origin_lla: latitude/longitude out of range, got {lla_raw}")
    topics = [_str(t, f"topics[{i}]") for i, t in enumerate(_list(data["topics"], "topics"))]
    trajectory = _REPO / _str(data["trajectory"], "trajectory")
    return Scenario(
        id=_str(data["id"], "id"),
        world=_str(data["world"], "world"),
        profile=_REPO / _str(data["profile"], "profile"),
        pipeline_config=_REPO / _str(data["pipeline_config"], "pipeline_config"),
        trajectory=trajectory,
        seed=seed,
        duration_s=duration,
        origin_lla=lla,
        gnss=_segments(data["gnss"]),
        gnss_zones=_zones(data["gnss_zones"]),
        topics=topics,
        prep=_prep(data["prep"]) if "prep" in data else None,
        route=load_route(trajectory),
        pipeline_config_variants=(
            _variants(data["pipeline_config_variants"])
            if "pipeline_config_variants" in data
            else {}
        ),
    )
