from collections.abc import Callable
from pathlib import Path

import pytest
import yaml

from mss_sim.config_overlay import merge_overlay
from mss_sim.scenario import ScenarioError, load_route, load_scenario

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


def test_infinite_speed_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["speed_mps"] = float("inf")  # written as YAML .inf

    with pytest.raises(ScenarioError, match=r"prep\.speed_mps: must be a finite number > 0"):
        load_scenario(_write(tmp_path, mutate))


def test_drop_fraction_above_one_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["mutate"]["drop_fraction"] = 1.5

    with pytest.raises(ScenarioError, match=r"prep\.mutate\.drop_fraction"):
        load_scenario(_write(tmp_path, mutate))


def test_zero_reorder_accepted(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["mutate"]["reorder_fraction"] = 0

    prep = load_scenario(_write(tmp_path, mutate)).prep
    assert prep is not None and prep.mutate is not None
    assert (prep.mutate.drop_fraction, prep.mutate.reorder_fraction) == (0.05, 0.0)


def test_both_fractions_zero_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["prep"]["mutate"]["drop_fraction"] = 0
        d["prep"]["mutate"]["reorder_fraction"] = 0

    with pytest.raises(ScenarioError, match=r"prep\.mutate: at least one of drop_fraction"):
        load_scenario(_write(tmp_path, mutate))


def test_mutate_absent_is_none(tmp_path: Path) -> None:
    sc = load_scenario(_write(tmp_path, lambda d: d["prep"].pop("mutate")))
    assert sc.prep is not None and sc.prep.mutate is None


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


def test_e01_route_loaded() -> None:
    route = load_scenario(E01).route
    assert route is not None
    assert route.waypoints[0] == (-8.0, 0.8)
    assert route.waypoints[-1] == route.waypoints[0]  # a closed lap
    assert (route.speed_mps, route.lookahead_m) == (0.5, 1.0)


def test_e04_loads_with_variant() -> None:
    sc = load_scenario(REPO / "sim/scenarios/E-04.yaml")
    assert sc.world == "corridor"
    assert sc.pipeline_config == REPO / "config/sim/corridor.yaml"
    assert sc.pipeline_config_variants == {
        "degeneracy_on": REPO / "config/sim/corridor_degeneracy_on.yaml"
    }
    assert sc.route is not None and sc.route.waypoints[-1][0] > 60.0  # ends in the room
    assert load_scenario(E01).pipeline_config_variants == {}


def _route_file(tmp_path: Path, data: dict) -> Path:
    f = tmp_path / "traj.yaml"
    f.write_text(yaml.safe_dump(data))
    return f


def _route(**over: object) -> dict:
    data: dict = {"waypoints": [[0.0, 0.0], [1.0, 0.0]], "speed_mps": 0.5, "lookahead_m": 1.0}
    data.update(over)
    return data


@pytest.mark.parametrize(
    ("data", "match"),
    [
        (_route(waypoints=[[0.0, 0.0]]), r"waypoints: needs >= 2"),
        (_route(waypoints=[[0.0, 0.0], [0.0, 0.0]]), r"waypoints\[1\]: repeats"),
        (_route(waypoints=[[0.0, 0.0], [1.0]]), r"waypoints\[1\]: expected \[x_m, y_m\]"),
        (_route(speed_mps=0), r"speed_mps: must be a finite number > 0"),
        (_route(lookahead_m=-1.0), r"lookahead_m"),
        ({"waypoints": [[0, 0], [1, 0]], "speed_mps": 0.5}, r"lookahead_m: missing"),
    ],
)
def test_bad_route_rejected(tmp_path: Path, data: dict, match: str) -> None:
    with pytest.raises(ScenarioError, match=match):
        load_route(_route_file(tmp_path, data))


def test_missing_trajectory_file_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["trajectory"] = "sim/trajectories/does_not_exist.yaml"

    with pytest.raises(ScenarioError, match=r"trajectory .*file not found"):
        load_scenario(_write(tmp_path, mutate))


def test_empty_variants_rejected(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        d["pipeline_config_variants"] = {}

    with pytest.raises(ScenarioError, match=r"pipeline_config_variants"):
        load_scenario(_write(tmp_path, mutate))
