"""Shared fixtures."""

from dataclasses import replace
from pathlib import Path

import pytest

from mss_sim.bridge import BridgeReport, run_bridge
from mss_sim.profile import load_profile
from mss_sim.scenario import load_scenario
from mss_sim.synth import SynthInfo, write_synthetic_raw_bag

_ORIGIN_LLA = (21.0278, 105.8342, 12.0)
_REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def raw_bag(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, SynthInfo]:
    """6 s synthetic raw bag (clean sensors + ground truth) and its description."""
    out_dir = tmp_path_factory.mktemp("raw_bag")
    info = write_synthetic_raw_bag(out_dir, duration_s=6.0, seed=0, origin_lla=_ORIGIN_LLA)
    return out_dir / "raw", info


def e01_scenario(info: SynthInfo):  # noqa: ANN201
    """Scenario E-01 with the raw-bag fixture's origin and a 6 s duration."""
    sc = load_scenario(_REPO / "sim/scenarios/E-01.yaml")
    return replace(sc, origin_lla=info.origin_lla, duration_s=6.0)


@pytest.fixture(scope="session")
def bridged_bag(
    raw_bag: tuple[Path, SynthInfo], tmp_path_factory: pytest.TempPathFactory
) -> tuple[Path, BridgeReport]:
    """Noisy bridge output dir (``bag/``, ``gt.tum``, ``bridge_report.json``), seed 42."""
    raw, info = raw_bag
    scenario = e01_scenario(info)
    out_dir = tmp_path_factory.mktemp("bridged") / "out"
    report = run_bridge(raw, scenario, load_profile(scenario.profile), out_dir, seed=42)
    return out_dir, report
