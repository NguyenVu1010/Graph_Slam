"""Ground-truth trajectory: Odometry ingestion, pose interpolation, TUM export."""

from __future__ import annotations

import logging
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from mss_sim.bag_io import stamp_ns

_LOG = logging.getLogger("mss_sim")
_NS_PER_S = 1_000_000_000


@dataclass
class GroundTruth:
    """Pose samples `T_w_b`: positions `p_w_b` and Hamilton quaternions (w, x, y, z)."""

    t_ns: np.ndarray  # (N,) int64, strictly increasing, sensor clock
    p_w_b: np.ndarray  # (N, 3) metres
    q_wxyz: np.ndarray  # (N, 4) Hamilton order
    n_clamped: int = field(default=0, init=False)
    _warned: bool = field(default=False, init=False, repr=False)

    def __post_init__(self) -> None:
        self.t_ns = np.asarray(self.t_ns, dtype=np.int64)
        self.p_w_b = np.asarray(self.p_w_b, dtype=np.float64)
        self.q_wxyz = np.asarray(self.q_wxyz, dtype=np.float64)
        n = self.t_ns.shape[0]
        if n < 2:
            msg = f"GroundTruth needs at least 2 samples, got {n}"
            raise ValueError(msg)
        if self.p_w_b.shape != (n, 3) or self.q_wxyz.shape != (n, 4):
            msg = "GroundTruth shape mismatch: expected p_w_b (N,3) and q_wxyz (N,4)"
            raise ValueError(msg)
        if np.any(np.diff(self.t_ns) <= 0):
            msg = "GroundTruth t_ns must be strictly increasing"
            raise ValueError(msg)
        self._rot = Rotation.from_quat(self.q_wxyz[:, [1, 2, 3, 0]])  # scipy: x y z w
        # Relative keys: absolute epoch-scale ns are not exact in float64.
        self._slerp = Slerp((self.t_ns - self.t_ns[0]).astype(np.float64), self._rot)

    @classmethod
    def from_odometry_msgs(cls, msgs: Iterable[Any]) -> GroundTruth:
        t: list[int] = []
        p: list[tuple[float, float, float]] = []
        q: list[tuple[float, float, float, float]] = []
        for m in msgs:
            pose = m.pose.pose
            t.append(stamp_ns(m.header))
            p.append((pose.position.x, pose.position.y, pose.position.z))
            o = pose.orientation
            q.append((o.w, o.x, o.y, o.z))  # Hamilton order (w, x, y, z)
        return cls(
            np.array(t, dtype=np.int64), np.array(p).reshape(-1, 3), np.array(q).reshape(-1, 4)
        )

    @property
    def t_start_ns(self) -> int:
        return int(self.t_ns[0])

    @property
    def t_end_ns(self) -> int:
        return int(self.t_ns[-1])

    def pose_at(self, t_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return `(R_w_b (M,3,3), p_w_b (M,3))`; queries outside the range are clamped."""
        q = np.atleast_1d(np.asarray(t_ns, dtype=np.int64))
        outside = (q < self.t_start_ns) | (q > self.t_end_ns)
        n_out = int(outside.sum())
        if n_out:
            self.n_clamped += n_out
            if not self._warned:
                self._warned = True
                _LOG.warning(
                    "ground-truth query outside [%d, %d] ns clamped to the end pose",
                    self.t_start_ns,
                    self.t_end_ns,
                )
        qc = np.clip(q, self.t_start_ns, self.t_end_ns)
        idx = np.clip(np.searchsorted(self.t_ns, qc, side="right") - 1, 0, len(self.t_ns) - 2)
        t0 = self.t_ns[idx]
        # Weights from integer differences, converted to float only at the division.
        w = ((qc - t0) / (self.t_ns[idx + 1] - t0))[:, None]
        p = (1.0 - w) * self.p_w_b[idx] + w * self.p_w_b[idx + 1]
        rot = self._slerp((qc - self.t_ns[0]).astype(np.float64)).as_matrix()
        return rot.reshape(-1, 3, 3), p

    def to_tum(self, path: Path) -> None:
        """Write `t x y z qx qy qz qw` (seconds, 9 decimals)."""
        q_xyzw = self.q_wxyz[:, [1, 2, 3, 0]]  # TUM file order (x, y, z, w)
        rest = np.column_stack([self.p_w_b, q_xyzw])
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = ["# t x y z qx qy qz qw"]
        for t_ns, row in zip(self.t_ns.tolist(), rest, strict=True):
            # Seconds from integers: exact nanoseconds even at epoch scale (float64 is not).
            t_s = f"{t_ns // _NS_PER_S}.{t_ns % _NS_PER_S:09d}"
            lines.append(" ".join([t_s, *(f"{v:.9f}" for v in row)]))
        # LF on every OS, like np.savetxt, so the file is byte-identical across platforms.
        path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
