"""TUM trajectory format: `t x y z qx qy qz qw` per line, `#` comments, time in seconds."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Trajectory:
    t_s: np.ndarray  # (N,) seconds
    p: np.ndarray  # (N, 3) position, metres
    q_xyzw: np.ndarray  # (N, 4) orientation, file order (x, y, z, w)

    def __len__(self) -> int:
        return int(self.t_s.shape[0])


def read_tum(path: Path) -> Trajectory:
    rows = [
        line.split()
        for line in path.read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if not rows:
        return Trajectory(np.empty(0), np.empty((0, 3)), np.empty((0, 4)))
    bad = [i for i, r in enumerate(rows) if len(r) != 8]
    if bad:
        msg = f"{path}: line {bad[0]} has {len(rows[bad[0]])} fields, expected 8"
        raise ValueError(msg)
    data = np.asarray(rows, dtype=float)
    return Trajectory(t_s=data[:, 0], p=data[:, 1:4], q_xyzw=data[:, 4:8])


def write_tum(path: Path, traj: Trajectory) -> None:
    data = np.column_stack([traj.t_s, traj.p, traj.q_xyzw])
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, data, fmt="%.9f", header="t x y z qx qy qz qw")
