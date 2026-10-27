from collections.abc import Callable
from pathlib import Path

import pytest
import yaml

from mss_sim.config_overlay import merge_overlay
from mss_sim.scenario import ScenarioError, load_scenario

REPO = Path(__file__).resolve().parents[2]
E01 = REPO / "sim/scenarios/E-01.yaml"


def _write(tmp_path: Path, mutate: Callable[[dict], None]) -> Path:
    data = yaml.safe_load(E01.read_text())
    mutate(data)
    f = tmp_path / "s.yaml"
    f.write_text(yaml.safe_dump(data))
    return f


def test_e01_loads() -> None:
    sc = load_scenario(E01)
    assert sc.id == "E-01"
    assert sc.world == "warehouse"
    assert sc.profile == REPO / "sim/noise/default.yaml"
    assert sc.pipeline_config == REPO / "config/sim/warehouse.yaml"
    assert sc.trajectory == REPO / "sim/trajectories/warehouse_lap_60s.yaml"
    assert sc.seed == 42
    assert sc.duration_s == 60.0
    assert sc.origin_lla == (21.0278, 105.8342, 12.0)
    assert [(g.mode, g.t_start_s, g.t_end_s) for g in sc.gnss] == [("nominal", 0.0, 60.0)]
    assert sc.gnss_zones == []
    assert len(sc.topics) == 7
    assert "/ground_truth" in sc.topics
    assert sc.prep is not None
    assert (sc.prep.static_s, sc.prep.ramp_s) == (5.0, 2.0)
    assert (sc.prep.speed_mps, sc.prep.omega_radps) == (0.5, 0.3)
    assert sc.prep.box_half_m == (8.0, 5.0, 2.0)
    m = sc.prep.mutate
    assert (m.drop_fraction, m.reorder_fraction, m.window_ns, m.seed) == (
        0.05,
        0.02,
        20_000_000,
        42,
    )


def test_prep_absent_is_none(tmp_path: Path) -> None:
    assert load_scenario(_write(tmp_path, lambda d: d.pop("prep"))).prep is None


def test_negative_ramp_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["ramp_s"] = -1.0

    with pytest.raises(ScenarioError, match=r"prep\.ramp_s"):
        load_scenario(_write(tmp_path, mutate))


def test_two_element_box_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["box_half_m"] = [8.0, 5.0]

    with pytest.raises(ScenarioError, match=r"prep\.box_half_m"):
        load_scenario(_write(tmp_path, mutate))


def test_zero_drop_fraction_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["mutate"]["drop_fraction"] = 0

    with pytest.raises(ScenarioError, match=r"prep\.mutate\.drop_fraction"):
        load_scenario(_write(tmp_path, mutate))


def test_overlapping_segments_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["gnss"] = [
            {"mode": "nominal", "t_start_s": 0, "t_end_s": 30},
            {"mode": "outage", "t_start_s": 20, "t_end_s": 60},
        ]

    with pytest.raises(ScenarioError, match=r"gnss\[1\]\.t_start_s"):
        load_scenario(_write(tmp_path, mutate))


def test_two_vertex_polygon_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["gnss_zones"] = [{"mode": "multipath", "polygon_enu": [[0, 0], [1, 1]]}]

    with pytest.raises(ScenarioError, match=r"gnss_zones\[0\]\.polygon_enu"):
        load_scenario(_write(tmp_path, mutate))


def test_valid_zone_builds_polygon(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["gnss_zones"] = [{"mode": "outage", "polygon_enu": [[0, 0], [4, 0], [4, 3]]}]

    zone = load_scenario(_write(tmp_path, mutate)).gnss_zones[0]
    assert zone.polygon().area == pytest.approx(6.0)


def test_unknown_mode_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["gnss"][0]["mode"] = "bogus"

    with pytest.raises(ScenarioError, match=r"gnss\[0\]\.mode"):
        load_scenario(_write(tmp_path, mutate))


def test_merge_overlay_unknown_key_names_path() -> None:
    with pytest.raises(KeyError, match=r"imu\.foo"):
        merge_overlay({"imu": {"a": 1}}, {"imu": {"foo": 2}})


def test_merge_overlay_deep_and_null_base() -> None:
    base = {"a": {"x": 1, "y": 2}, "b": None}
    out = merge_overlay(base, {"a": {"y": 3}, "b": {"new": 1}})
    assert out == {"a": {"x": 1, "y": 3}, "b": {"new": 1}}
    assert base == {"a": {"x": 1, "y": 2}, "b": None}  # inputs untouched
