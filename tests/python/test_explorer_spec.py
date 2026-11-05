"""Explorer experiment spec: YAML round trip, validation and the cache key."""

from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from conftest import _REPO

from mss_explorer.spec import ExperimentSpec, SpecError, dump_spec, load_spec, spec_key
from mss_sim.faults import Fault
from mss_sim.noise_scale import NoiseScale

_ROOM = _REPO / "sim/scenarios/ROOM-01.yaml"


def _spec(tmp_path: Path) -> ExperimentSpec:
    return ExperimentSpec(
        scenario=_ROOM,
        recording=tmp_path / "rec",
        factors={"gnss": True, "wheel_odom": None, "loop_closure": False},
        noise=NoiseScale(imu=2.0, gnss=0.5),
        faults=(Fault("gnss", "outage", 60.0, 120.0), Fault("lidar", "range_limit", 1, 2, 5.0)),
        seed=None,
        label="gnss outage",
    )


def _write(tmp_path: Path, doc: dict) -> Path:
    p = tmp_path / "spec.yaml"
    p.write_text(yaml.safe_dump(doc))
    return p


def test_spec_roundtrip(tmp_path: Path) -> None:
    s = _spec(tmp_path)
    dump_spec(s, tmp_path / "s.yaml")
    assert load_spec(tmp_path / "s.yaml") == s


def test_spec_defaults(tmp_path: Path) -> None:
    s = load_spec(_write(tmp_path, {"scenario": str(_ROOM), "recording": str(tmp_path)}))
    assert s.factors == {"gnss": None, "wheel_odom": None, "loop_closure": None}
    assert s.noise == NoiseScale() and s.faults == () and s.seed is None and s.label == ""


def test_spec_relative_paths_resolve_from_repo(tmp_path: Path) -> None:
    s = load_spec(_write(tmp_path, {"scenario": "sim/scenarios/ROOM-01.yaml", "recording": "r"}))
    assert s.scenario == _ROOM and s.recording == _REPO / "r"


def test_spec_key_ignores_label_and_changes_with_noise(tmp_path: Path) -> None:
    s = _spec(tmp_path)
    k = spec_key(s, "abc")
    assert len(k) == 16
    assert spec_key(replace(s, label="other"), "abc") == k
    assert spec_key(replace(s, noise=NoiseScale(imu=3.0)), "abc") != k
    assert spec_key(s, "def") != k


@pytest.mark.parametrize(
    ("extra", "field"),
    [
        ({"colour": "red"}, "colour"),
        ({"factors": {"lidar": False}}, "lidar"),
        ({"noise": {"imu": 0.5}}, "imu"),
        ({"noise": {"sonar": 1.0}}, "sonar"),
        (
            {"faults": [{"sensor": "gnss", "kind": "outage", "t_start_s": 5, "t_end_s": 4}]},
            "t_end_s",
        ),
        (
            {"faults": [{"sensor": "gnss", "kind": "outage", "t_start_s": 5, "t_end_s": 300}]},
            "t_end_s",
        ),
        ({"faults": [{"sensor": "imu", "kind": "outage", "t_start_s": 1, "t_end_s": 2}]}, "kind"),
        ({"seed": "x"}, "seed"),
    ],
)
def test_spec_rejects(tmp_path: Path, extra: dict, field: str) -> None:
    doc = {"scenario": str(_ROOM), "recording": str(tmp_path), **extra}
    with pytest.raises(SpecError, match=field):
        load_spec(_write(tmp_path, doc))


def test_spec_rejects_missing_scenario(tmp_path: Path) -> None:
    with pytest.raises(SpecError, match="scenario"):
        load_spec(_write(tmp_path, {"recording": str(tmp_path)}))


def test_spec_fault_default_magnitude(tmp_path: Path) -> None:
    f = {"sensor": "gnss", "kind": "offset", "t_start_s": 1, "t_end_s": 2}
    s = load_spec(_write(tmp_path, {"scenario": str(_ROOM), "recording": "r", "faults": [f]}))
    assert s.faults[0].magnitude == 5.0


def test_spec_variant(tmp_path: Path) -> None:
    doc = {"scenario": str(_ROOM), "recording": "r", "variant": "loops_on"}
    s = load_spec(_write(tmp_path, doc))
    assert s.variant == "loops_on"
    assert spec_key(s, "c") != spec_key(replace(s, variant=None), "c")


def test_spec_rejects_unknown_variant(tmp_path: Path) -> None:
    doc = {"scenario": str(_ROOM), "recording": "r", "variant": "nope"}
    with pytest.raises(SpecError, match="variant"):
        load_spec(_write(tmp_path, doc))
