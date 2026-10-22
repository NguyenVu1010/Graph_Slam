"""Minimal PointCloud2 parser. Deliberately duplicates ``mss_sim.bag_io.cloud_to_arrays`` so that
``eval`` never imports ``sim`` (ADR-003/010)."""

from __future__ import annotations

from typing import Any

import numpy as np

# sensor_msgs/PointField datatype code -> numpy dtype character code.
_NP_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_to_arrays(
    msg: Any,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, int, int]:
    """``(xyz (N,3) float32, t_ns uint32 | None, ring uint16 | None, height, width)``.

    Uses the message's own ``fields`` and ``point_step``, so field order and padding may vary.
    """
    endian = ">" if msg.is_bigendian else "<"
    fields = [f for f in msg.fields if f.count == 1 and f.datatype in _NP_DTYPES]
    names = [f.name for f in fields]
    if not {"x", "y", "z"} <= set(names):
        raise ValueError(f"PointCloud2 lacks x/y/z fields; has {names}")
    dtype = np.dtype(
        {
            "names": names,
            "formats": [endian + _NP_DTYPES[f.datatype] for f in fields],
            "offsets": [int(f.offset) for f in fields],
            "itemsize": int(msg.point_step),
        }
    )
    n = int(msg.height) * int(msg.width)
    buf = np.asarray(msg.data, dtype=np.uint8).tobytes()
    if len(buf) < n * int(msg.point_step):
        raise ValueError(f"PointCloud2 data too short: {len(buf)} bytes for {n} points")
    raw = np.frombuffer(buf, dtype=dtype, count=n)
    xyz = np.stack([raw[k].astype(np.float32) for k in ("x", "y", "z")], axis=1)
    t_ns = raw["t"].astype(np.uint32) if "t" in names else None
    ring = raw["ring"].astype(np.uint16) if "ring" in names else None
    return xyz, t_ns, ring, int(msg.height), int(msg.width)
