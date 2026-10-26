"""Scenario file loader (ADR-011 section 5). Validation errors name the dotted key path."""

from __future__ import annotations

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


def _num(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{path}: expected a number, got {value!r}")
    return float(value)


def _str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ScenarioError(f"{path}: expected a non-empty string, got {value!r}")
    return value


def _mapping(value: Any, path: str, keys: set[str]) -> dict:
    if not isinstance(value, dict):
        raise ScenarioError(f"{path}: expected a mapping")
    for key in sorted(keys - value.keys()):
        raise ScenarioError(f"{path}.{key}: missing")
    for key in value:
        if key not in keys:
            raise ScenarioError(f"{path}.{key}: unknown key")
    return value


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


def load_scenario(path: Path) -> Scenario:
    """Load and validate a scenario YAML. Referenced files are resolved but need not exist."""
    data = yaml.safe_load(Path(path).read_text())
    data = _mapping(data, "<root>", _TOP)
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
    return Scenario(
        id=_str(data["id"], "id"),
        world=_str(data["world"], "world"),
        profile=_REPO / _str(data["profile"], "profile"),
        pipeline_config=_REPO / _str(data["pipeline_config"], "pipeline_config"),
        trajectory=_REPO / _str(data["trajectory"], "trajectory"),
        seed=seed,
        duration_s=duration,
        origin_lla=lla,
        gnss=_segments(data["gnss"]),
        gnss_zones=_zones(data["gnss_zones"]),
        topics=topics,
    )
