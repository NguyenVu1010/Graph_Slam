from pathlib import Path

import numpy as np
import pytest

from mss_eval.tum import Trajectory, read_tum, write_tum


def test_round_trip(tmp_path: Path) -> None:
    traj = Trajectory(
        t_s=np.array([0.0, 0.1]),
        p=np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]),
        q_xyzw=np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]),
    )
    out = tmp_path / "traj.tum"
    write_tum(out, traj)
    back = read_tum(out)
    assert len(back) == 2
    np.testing.assert_allclose(back.p, traj.p)
    np.testing.assert_allclose(back.q_xyzw, traj.q_xyzw)


def test_rejects_wrong_field_count(tmp_path: Path) -> None:
    f = tmp_path / "bad.tum"
    f.write_text("# header\n0 1 2 3\n")
    with pytest.raises(ValueError, match="expected 8"):
        read_tum(f)
