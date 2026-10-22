"""Shared fixtures."""

from pathlib import Path

import pytest

from mss_sim.synth import SynthInfo, write_synthetic_raw_bag

_ORIGIN_LLA = (21.0278, 105.8342, 12.0)


@pytest.fixture(scope="session")
def raw_bag(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, SynthInfo]:
    """6 s synthetic raw bag (clean sensors + ground truth) and its description."""
    out_dir = tmp_path_factory.mktemp("raw_bag")
    info = write_synthetic_raw_bag(out_dir, duration_s=6.0, seed=0, origin_lla=_ORIGIN_LLA)
    return out_dir / "raw", info
