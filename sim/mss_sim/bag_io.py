"""MCAP bag I/O (via ``rosbags``, no ROS needed) and PointCloud2 <-> numpy helpers.

Timestamps are int64 nanoseconds. Bags are written as MCAP, zstd-compressed per chunk.
"""

from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import Any

import numpy as np
from rosbags.rosbag2 import CompressionFormat, CompressionMode, Reader, StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

_NS_PER_S = 1_000_000_000
_BAG_VERSION = 9

# sensor_msgs/PointField datatype codes.
F32, U32, U16 = 7, 6, 4
_NP_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
_DTYPE_SIZE = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}

CLOUD_FIELDS = [("x", F32), ("y", F32), ("z", F32), ("intensity", F32), ("t", U32), ("ring", U16)]
_CLOUD_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("t", "<u4"), ("ring", "<u2")]
)  # packed, 22 bytes per point


class CloudFormatError(ValueError):
    """A PointCloud2 message is malformed (missing fields or truncated data)."""


def stamp_ns(header: Any) -> int:
    """Header stamp as int nanoseconds."""
    return int(header.stamp.sec) * _NS_PER_S + int(header.stamp.nanosec)


def make_header(stamp_ns: int, frame_id: str) -> Any:
    """Build a std_msgs/Header from nanoseconds."""
    sec, nanosec = divmod(int(stamp_ns), _NS_PER_S)
    time = TYPESTORE.types["builtin_interfaces/msg/Time"](sec=sec, nanosec=nanosec)
    return TYPESTORE.types["std_msgs/msg/Header"](stamp=time, frame_id=frame_id)


class BagReader:
    """Read-only MCAP bag; use as a context manager."""

    def __init__(self, path: Path) -> None:
        self._reader = Reader(path)

    def __enter__(self) -> "BagReader":
        self._reader.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self._reader.close()

    def topics(self) -> dict[str, str]:
        """Topic name -> message type name."""
        return {c.topic: c.msgtype for c in self._reader.connections}

    def messages(self, topics: list[str]) -> Iterator[tuple[str, int, Any]]:
        """Yield (topic, log_time_ns, deserialised msg) for the topics, ordered by log time.

        Raises ``KeyError`` (eagerly, at call time) naming any requested topic not in the bag.
        An empty topic list yields nothing.
        """
        available = {c.topic for c in self._reader.connections}
        missing = sorted(set(topics) - available)
        if missing:
            raise KeyError(f"topics not in bag: {missing}; available: {sorted(available)}")
        conns = [c for c in self._reader.connections if c.topic in set(topics)]
        return self._iter_messages(conns)

    def _iter_messages(self, conns: list[Any]) -> Iterator[tuple[str, int, Any]]:
        if not conns:  # rosbags treats an empty filter as "all connections"
            return
        for conn, t_ns, raw in self._reader.messages(connections=conns):
            yield conn.topic, int(t_ns), TYPESTORE.deserialize_cdr(raw, conn.msgtype)


class BagWriter:
    """MCAP bag writer (zstd-compressed); use as a context manager."""

    def __init__(self, path: Path) -> None:
        self._writer = Writer(path, version=_BAG_VERSION, storage_plugin=StoragePlugin.MCAP)
        self._writer.set_compression(CompressionMode.STORAGE, CompressionFormat.ZSTD)
        self._conns: dict[str, Any] = {}
        self._types: dict[str, str] = {}

    def __enter__(self) -> "BagWriter":
        self._writer.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self._writer.close()

    def add_topic(self, name: str, typename: str) -> None:
        self._conns[name] = self._writer.add_connection(name, typename, typestore=TYPESTORE)
        self._types[name] = typename

    def write(self, topic: str, log_time_ns: int, msg: Any) -> None:
        raw = TYPESTORE.serialize_cdr(msg, self._types[topic])
        self._writer.write(self._conns[topic], int(log_time_ns), bytes(raw))


@dataclass
class CloudArrays:
    """Columnar point cloud. ``t_ns`` is the per-point time offset; ``ring`` the beam index."""

    xyz: np.ndarray  # (N, 3) float32, N = height * width
    intensity: np.ndarray  # (N,) float32
    t_ns: np.ndarray | None  # (N,) uint32
    ring: np.ndarray | None  # (N,) uint16
    height: int
    width: int


def cloud_to_arrays(msg: Any) -> CloudArrays:
    """Parse a PointCloud2 using its own ``fields`` and ``point_step`` (any order/padding)."""
    names, formats, offsets = [], [], []
    endian = ">" if msg.is_bigendian else "<"
    for f in msg.fields:
        if f.count != 1 or f.datatype not in _NP_DTYPES:
            continue
        names.append(f.name)
        formats.append(endian + _NP_DTYPES[f.datatype])
        offsets.append(int(f.offset))
    missing = [k for k in ("x", "y", "z") if k not in names]
    if missing:
        raise CloudFormatError(f"PointCloud2 lacks scalar field(s) {missing}; has {names}")
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": int(msg.point_step)}
    )
    n = int(msg.height) * int(msg.width)
    buf = np.asarray(msg.data, dtype=np.uint8).tobytes()
    if len(buf) < n * int(msg.point_step):
        raise CloudFormatError(
            f"PointCloud2 data too short: expected {n * int(msg.point_step)} bytes "
            f"({msg.height}x{msg.width} points x point_step {msg.point_step}), got {len(buf)}"
        )
    raw = np.frombuffer(buf, dtype=dtype, count=n)
    xyz = np.stack([raw[k].astype(np.float32) for k in ("x", "y", "z")], axis=1)
    intensity = (
        raw["intensity"].astype(np.float32) if "intensity" in names else np.zeros(n, np.float32)
    )
    t_ns = raw["t"].astype(np.uint32) if "t" in names else None
    ring = raw["ring"].astype(np.uint16) if "ring" in names else None
    return CloudArrays(xyz, intensity, t_ns, ring, int(msg.height), int(msg.width))


def arrays_to_cloud(arrays: CloudArrays, stamp_ns: int, frame_id: str) -> Any:
    """Build a PointCloud2 with exactly ``CLOUD_FIELDS`` (22 bytes per point)."""
    n = arrays.height * arrays.width
    rec = np.zeros(n, dtype=_CLOUD_DTYPE)
    rec["x"], rec["y"], rec["z"] = arrays.xyz[:, 0], arrays.xyz[:, 1], arrays.xyz[:, 2]
    rec["intensity"] = arrays.intensity
    if arrays.t_ns is not None:
        rec["t"] = arrays.t_ns
    if arrays.ring is not None:
        rec["ring"] = arrays.ring
    point_field = TYPESTORE.types["sensor_msgs/msg/PointField"]
    fields, offset = [], 0
    for name, datatype in CLOUD_FIELDS:
        fields.append(point_field(name=name, offset=offset, datatype=datatype, count=1))
        offset += _DTYPE_SIZE[datatype]
    step = _CLOUD_DTYPE.itemsize
    return TYPESTORE.types["sensor_msgs/msg/PointCloud2"](
        header=make_header(stamp_ns, frame_id),
        height=arrays.height,
        width=arrays.width,
        fields=fields,
        is_bigendian=False,
        point_step=step,
        row_step=step * arrays.width,
        data=np.frombuffer(rec.tobytes(), dtype=np.uint8),
        is_dense=False,
    )
