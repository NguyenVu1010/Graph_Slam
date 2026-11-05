"""Timed sensor faults injected into a bridged bag (sensor impact explorer, offline re-runs).

A fault acts on the messages of one sensor whose header stamp lies in
``[t0 + t_start_s, t0 + t_end_s)``, ``t0`` being the first ground-truth stamp. Untouched messages
are copied byte for byte. Odometry faults change the twist only: the dataset converter
(``bag_to_mssdata.py``) reads nothing else from ``/wheel/odom``.

rng: ``SeedSequence(seed).spawn(5)[4]``, the ``mutate`` stream of the order
[lidar, gnss, odom, imu, mutate]; it is drawn in log order, so a seed fixes the output.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from mss_sim.bag_io import (
    TYPESTORE,
    BagReader,
    BagWriter,
    CloudArrays,
    arrays_to_cloud,
    cloud_to_arrays,
    stamp_ns,
)
from mss_sim.geodesy import enu_to_lla

_NS = 1_000_000_000
TOPICS = {"lidar": "/lidar/points", "imu": "/imu/data", "gnss": "/gnss/fix", "odom": "/wheel/odom"}
SPIKE_P = 0.02  # probability of a spike per IMU sample
JUMP_HOLD_S = 2.0  # a GNSS jump keeps its direction this long


@dataclass(frozen=True)
class KindSpec:
    unit: str  # "" when the kind takes no magnitude
    default: float | None
    lo: float
    hi: float
    lo_open: bool = False


_NONE = KindSpec("", None, 0.0, 0.0)
KINDS: dict[str, dict[str, KindSpec]] = {
    "lidar": {
        "blackout": _NONE,
        "sector_occlusion": KindSpec("deg", 120.0, 0.0, 360.0, lo_open=True),
        "range_limit": KindSpec("m", 5.0, 0.5, 200.0, lo_open=True),
    },
    "imu": {
        "gyro_bias_step": KindSpec("rad/s", 0.02, -1.0, 1.0),
        "acc_bias_step": KindSpec("m/s^2", 0.3, -5.0, 5.0),
        "spike": KindSpec("m/s^2", 20.0, 0.0, 200.0, lo_open=True),
        "saturation": KindSpec("m/s^2", 8.0, 0.5, 50.0, lo_open=True),
        "freeze": _NONE,
    },
    "gnss": {
        "outage": _NONE,
        "jump": KindSpec("m", 8.0, 0.0, 100.0, lo_open=True),
        "offset": KindSpec("m", 5.0, 0.0, 100.0, lo_open=True),
        "degraded": KindSpec("m", 3.0, 0.0, 50.0, lo_open=True),
    },
    "odom": {
        "slip": KindSpec("scale", 0.7, 0.0, 3.0),
        "stuck": _NONE,
        "dropout": _NONE,
    },
}
_DROP = {("lidar", "blackout"), ("gnss", "outage"), ("odom", "dropout")}


@dataclass(frozen=True)
class Fault:
    sensor: str
    kind: str
    t_start_s: float
    t_end_s: float
    magnitude: float | None = None


@dataclass(frozen=True)
class FaultReport:
    n_dropped: dict[str, int]  # per topic
    n_modified: dict[str, int]


def validate_fault(f: Fault, duration_s: float) -> Fault:
    """``f`` with the default magnitude filled in (``None`` for kinds without one)."""
    if f.sensor not in KINDS:
        raise ValueError(f"fault.sensor: '{f.sensor}' is not one of {sorted(KINDS)}")
    spec = KINDS[f.sensor].get(f.kind)
    if spec is None:
        raise ValueError(
            f"fault.kind: '{f.kind}' is not a {f.sensor} fault {sorted(KINDS[f.sensor])}"
        )
    if not (math.isfinite(f.t_start_s) and 0.0 <= f.t_start_s):
        raise ValueError(f"fault.t_start_s: {f.t_start_s} must be >= 0")
    if not (math.isfinite(f.t_end_s) and f.t_start_s < f.t_end_s <= duration_s):
        raise ValueError(f"fault.t_end_s: {f.t_end_s} must be in ({f.t_start_s}, {duration_s}]")
    if not spec.unit:
        return replace(f, magnitude=None)
    m = spec.default if f.magnitude is None else f.magnitude
    above_lo = m > spec.lo if spec.lo_open else m >= spec.lo
    if not (math.isfinite(m) and above_lo and m <= spec.hi):
        lo = "(" if spec.lo_open else "["
        raise ValueError(f"fault.magnitude: {m} {spec.unit} outside {lo}{spec.lo}, {spec.hi}]")
    return replace(f, magnitude=m)


def _vec(v: Any, d: np.ndarray) -> Any:
    return replace(v, x=float(d[0]), y=float(d[1]), z=float(d[2]))


def _arr(v: Any) -> np.ndarray:
    return np.array([v.x, v.y, v.z], dtype=np.float64)


def _keep_points(a: CloudArrays, keep: np.ndarray) -> CloudArrays:
    return replace(
        a,
        xyz=a.xyz[keep],
        intensity=a.intensity[keep],
        t_ns=None if a.t_ns is None else a.t_ns[keep],
        ring=None if a.ring is None else a.ring[keep],
        height=1,
        width=int(np.count_nonzero(keep)),
    )


class _Injector:
    def __init__(self, faults: Sequence[Fault], t0_ns: int, rng: np.random.Generator) -> None:
        self._windows = [
            (f, t0_ns + round(f.t_start_s * _NS), t0_ns + round(f.t_end_s * _NS)) for f in faults
        ]
        self._rng = rng
        self._last_imu: Any = None  # last IMU message before any freeze window
        self._jump_dir: dict[tuple[int, int], float] = {}

    def active(self, sensor: str, stamp: int) -> list[tuple[int, Fault, int]]:
        return [
            (i, f, start)
            for i, (f, start, end) in enumerate(self._windows)
            if f.sensor == sensor and start <= stamp < end
        ]

    def apply(self, sensor: str, msg: Any, stamp: int, faults: list[tuple[int, Fault, int]]) -> Any:
        """Faulted copy of ``msg``, or ``None`` to drop it."""
        for i, f, start in faults:
            if (f.sensor, f.kind) in _DROP:
                return None
            msg = getattr(self, f"_{sensor}")(msg, f, i, stamp - start)
        return msg

    def _lidar(self, msg: Any, f: Fault, _i: int, _dt: int) -> Any:
        a = cloud_to_arrays(msg)
        xyz = a.xyz.astype(np.float64)
        if f.kind == "range_limit":
            keep = ~(np.linalg.norm(xyz, axis=1) > f.magnitude)
        else:  # sector_occlusion, centred on the LiDAR's +x axis
            ang = np.degrees(np.abs(np.arctan2(xyz[:, 1], xyz[:, 0])))
            keep = ~(ang < f.magnitude / 2.0)
        return arrays_to_cloud(_keep_points(a, keep), stamp_ns(msg.header), msg.header.frame_id)

    def _imu(self, msg: Any, f: Fault, _i: int, _dt: int) -> Any:
        w, a = _arr(msg.angular_velocity), _arr(msg.linear_acceleration)
        if f.kind == "gyro_bias_step":
            w[2] += f.magnitude
        elif f.kind == "acc_bias_step":
            a[0] += f.magnitude
        elif f.kind == "spike":
            if self._rng.random() < SPIKE_P:
                a[self._rng.integers(3)] += f.magnitude * self._rng.choice((-1.0, 1.0))
        elif f.kind == "saturation":
            a = np.clip(a, -f.magnitude, f.magnitude)
        else:  # freeze
            src = self._last_imu if self._last_imu is not None else msg
            self._last_imu = src
            w, a = _arr(src.angular_velocity), _arr(src.linear_acceleration)
        return replace(
            msg,
            angular_velocity=_vec(msg.angular_velocity, w),
            linear_acceleration=_vec(msg.linear_acceleration, a),
        )

    def note_imu(self, msg: Any) -> None:
        self._last_imu = msg

    def _gnss(self, msg: Any, f: Fault, i: int, dt: int) -> Any:
        if f.kind == "offset":
            enu = np.array([f.magnitude, 0.0, 0.0])
        elif f.kind == "jump":
            key = (i, dt // round(JUMP_HOLD_S * _NS))
            if key not in self._jump_dir:
                self._jump_dir[key] = float(self._rng.uniform(0.0, 2.0 * math.pi))
            th = self._jump_dir[key]
            enu = np.array([f.magnitude * math.cos(th), f.magnitude * math.sin(th), 0.0])
        else:  # degraded
            enu = np.array([*(f.magnitude * self._rng.standard_normal(2)), 0.0])
        lat, lon, alt = enu_to_lla(enu, (msg.latitude, msg.longitude, msg.altitude))
        return replace(msg, latitude=float(lat), longitude=float(lon), altitude=float(alt))

    def _odom(self, msg: Any, f: Fault, _i: int, _dt: int) -> Any:
        k = f.magnitude if f.kind == "slip" else 0.0
        tw = msg.twist.twist
        tw = replace(
            tw,
            linear=_vec(tw.linear, k * _arr(tw.linear)),
            angular=_vec(tw.angular, k * _arr(tw.angular)),
        )
        return replace(msg, twist=replace(msg.twist, twist=tw))


def apply_faults(
    in_bag: Path, out_bag: Path, faults: Sequence[Fault], t0_ns: int, seed: int
) -> FaultReport:
    """Copy ``in_bag`` to ``out_bag`` with ``faults`` (already validated) applied."""
    rng = np.random.default_rng(np.random.SeedSequence(seed).spawn(5)[4])
    inj = _Injector(faults, t0_ns, rng)
    sensor_of = {topic: s for s, topic in TOPICS.items()}
    dropped = dict.fromkeys(TOPICS.values(), 0)
    modified = dict.fromkeys(TOPICS.values(), 0)
    with BagReader(in_bag) as r, BagWriter(out_bag) as w:
        types = r.topics()
        for name, typename in types.items():
            w.add_topic(name, typename)
        for topic, t, data in r.raw_messages(list(types)):
            sensor = sensor_of.get(topic)
            if sensor is None:
                w.write_raw(topic, t, data)
                continue
            msg = TYPESTORE.deserialize_cdr(data, types[topic])
            active = inj.active(sensor, stamp_ns(msg.header))
            if not active:
                if sensor == "imu":
                    inj.note_imu(msg)
                w.write_raw(topic, t, data)
                continue
            out = inj.apply(sensor, msg, stamp_ns(msg.header), active)
            if out is None:
                dropped[topic] += 1
            else:
                modified[topic] += 1
                w.write(topic, t, out)
    return FaultReport(dropped, modified)
