"""Ground surfaces of a world (backlog L3 / S1): friction and WheelSlip compliance by position.

The world sidecar (``sim/worlds/<world>.planes.yaml``) may hold ``default_surface`` and
``surfaces``: axis-aligned ground patches ``rect_m: [x_min, y_min, x_max, y_max]`` (m, world)
with the ``mu`` of their SDF tile and the gz ``WheelSlip`` compliances (unitless: slip ratio per
tangential / normal force ratio). ``SurfaceCommander`` decides when to send the compliances of
the patch that holds the body origin and ``wheel_slip_request`` writes the request;
``surface_node`` is the rclpy / ``gz service`` glue. No ROS here.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class SurfaceError(ValueError):
    """Invalid ``surfaces`` block. The message names the key."""


@dataclass(frozen=True)
class Surface:
    name: str
    mu: float  # contact friction of the SDF tile
    slip_compliance_longitudinal: float  # gz WheelSlip, unitless, >= 0
    slip_compliance_lateral: float  # gz WheelSlip, unitless, >= 0
    model: str | None = None  # SDF model of the tile (None for the default surface)
    rect_m: tuple[float, float, float, float] | None = None  # x_min, y_min, x_max, y_max

    def contains(self, x_m: float, y_m: float) -> bool:
        if self.rect_m is None:
            return False
        x0, y0, x1, y1 = self.rect_m
        return x0 <= x_m < x1 and y0 <= y_m < y1


@dataclass(frozen=True)
class SurfaceMap:
    default: Surface
    patches: tuple[Surface, ...]

    def at(self, x_m: float, y_m: float) -> Surface:
        """The patch that holds ``(x_m, y_m)`` (world, m), else the default surface."""
        for s in self.patches:
            if s.contains(x_m, y_m):
                return s
        return self.default


def _num(d: dict, key: str, where: str, minimum: float) -> float:
    v = d.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise SurfaceError(f"{where}.{key}: expected a finite number, got {v!r}")
    if v < minimum:
        raise SurfaceError(f"{where}.{key}: must be >= {minimum}, got {v!r}")
    return float(v)


def _surface(d: Any, where: str, patch: bool) -> Surface:
    if not isinstance(d, dict):
        raise SurfaceError(f"{where}: expected a mapping")
    keys = {"name", "mu", "slip_compliance_longitudinal", "slip_compliance_lateral"}
    if patch:
        keys |= {"model", "rect_m"}
    for key in sorted(keys - d.keys()):
        raise SurfaceError(f"{where}.{key}: missing")
    for key in d:
        if key not in keys:
            raise SurfaceError(f"{where}.{key}: unknown key")
    rect = None
    if patch:
        r = d["rect_m"]
        if not isinstance(r, list) or len(r) != 4:
            raise SurfaceError(f"{where}.rect_m: expected [x_min, y_min, x_max, y_max]")
        rect = tuple(
            _num({"v": v}, "v", f"{where}.rect_m[{i}]", -math.inf) for i, v in enumerate(r)
        )
        if not (rect[0] < rect[2] and rect[1] < rect[3]):
            raise SurfaceError(f"{where}.rect_m: need x_min < x_max and y_min < y_max, got {r}")
    return Surface(
        name=str(d["name"]),
        mu=_num(d, "mu", where, 0.0),
        slip_compliance_longitudinal=_num(d, "slip_compliance_longitudinal", where, 0.0),
        slip_compliance_lateral=_num(d, "slip_compliance_lateral", where, 0.0),
        model=str(d["model"]) if patch else None,
        rect_m=rect,  # type: ignore[arg-type]
    )


def load_surfaces(sidecar: Path) -> SurfaceMap | None:
    """The surfaces of a world sidecar, or None when it declares none (indoor worlds)."""
    data = yaml.safe_load(Path(sidecar).read_text(encoding="utf-8"))
    if "surfaces" not in data and "default_surface" not in data:
        return None
    default = _surface(data.get("default_surface"), "default_surface", patch=False)
    raw = data.get("surfaces")
    if not isinstance(raw, list) or not raw:
        raise SurfaceError("surfaces: expected a non-empty list")
    patches = tuple(_surface(s, f"surfaces[{i}]", patch=True) for i, s in enumerate(raw))
    for i, a in enumerate(patches):
        for b in patches[i + 1 :]:
            ra, rb = a.rect_m, b.rect_m
            assert ra is not None and rb is not None
            if ra[0] < rb[2] and rb[0] < ra[2] and ra[1] < rb[3] and rb[1] < ra[3]:
                raise SurfaceError(f"surfaces: {a.name} and {b.name} overlap")
    return SurfaceMap(default, patches)


RESEND_NS = 1_000_000_000  # sim ns between retries of a failed command


def wheel_slip_request(model: str, s: Surface) -> str:
    """Text-format ``gz.msgs.WheelSlipParametersCmd`` for every wheel of ``model``."""
    return (
        f'entity {{name: "{model}" type: MODEL}} '
        f"slip_compliance_lateral: {s.slip_compliance_lateral!r} "
        f"slip_compliance_longitudinal: {s.slip_compliance_longitudinal!r}"
    )


class SurfaceCommander:
    """Sends the surface under the body once per change; retries a failed send.

    ``send(surface) -> bool`` performs the request (True when acknowledged). ``update`` is called
    with every ground-truth sample (``t_ns`` sim time, body origin in world, m); it returns the
    new surface when the surface changed (for logging), else None.
    """

    def __init__(
        self, surfaces: SurfaceMap, send: Callable[[Surface], bool], resend_ns: int = RESEND_NS
    ) -> None:
        self._map = surfaces
        self._send = send
        self._resend_ns = resend_ns
        self.current: Surface | None = None
        self.acknowledged = False  # the current surface's command was acknowledged
        self._last_try_ns: int | None = None
        self.n_sent = 0
        self.n_failed = 0

    def update(self, t_ns: int, x_m: float, y_m: float) -> Surface | None:
        s = self._map.at(x_m, y_m)
        changed = s is not self.current
        if changed:
            self.current, self.acknowledged, self._last_try_ns = s, False, None
        due = self._last_try_ns is None or t_ns - self._last_try_ns >= self._resend_ns
        if not self.acknowledged and due:
            self._last_try_ns = t_ns
            self.acknowledged = self._send(s)
            self.n_sent += 1
            self.n_failed += 0 if self.acknowledged else 1
        return s if changed else None
