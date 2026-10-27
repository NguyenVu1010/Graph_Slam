"""sim/worlds: SDF parses, required models exist, planes sidecar matches the box faces."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

import gen_sim_assets as gsa
from mss_eval.noise_report import load_planes
from mss_sim.scenario import load_scenario

REPO = Path(__file__).resolve().parents[2]
WORLDS = REPO / "sim/worlds"
REQUIRED = {
    "warehouse": {"floor", "ceiling", "wall_east", "wall_west", "wall_north", "wall_south"},
    "corridor": {
        "floor",
        "ceiling",
        "wall_start",
        "corridor_wall_north",
        "corridor_wall_south",
        "room_wall_east",
    },
}


def _models(world: str) -> dict[str, ET.Element]:
    root = ET.parse(WORLDS / f"{world}.sdf").getroot()
    w = root.find("world")
    assert w is not None and w.get("name") == world
    return {m.get("name", ""): m for m in w.findall("model")}


@pytest.mark.parametrize("world", sorted(REQUIRED))
def test_world_parses_with_required_models(world: str) -> None:
    models = _models(world)
    assert REQUIRED[world] <= models.keys()
    w = ET.parse(WORLDS / f"{world}.sdf").getroot().find("world")
    assert w is not None
    physics = w.find("physics")
    assert physics is not None and physics.get("type") == "dart"
    assert physics.findtext("max_step_size") == "0.001"
    assert float(physics.findtext("real_time_factor") or "nan") == 0.0  # no real-time lock
    plugins = {p.get("name") for p in w.findall("plugin")}
    for system in ("Physics", "Sensors", "Imu", "NavSat", "UserCommands"):
        assert f"gz::sim::systems::{system}" in plugins


@pytest.mark.parametrize("world", sorted(REQUIRED))
def test_planes_lie_on_box_faces(world: str) -> None:
    assert gsa.check_world(WORLDS / f"{world}.sdf", WORLDS / f"{world}.planes.yaml") == []


def test_check_worlds_cli_passes() -> None:
    assert gsa.main(["--check-worlds"]) == 0


def test_check_detects_a_plane_off_the_faces(tmp_path: Path) -> None:
    data = yaml.safe_load((WORLDS / "warehouse.planes.yaml").read_text())
    data["planes"][2]["d"] = 20.002  # 2 mm off the east wall face
    side = tmp_path / "warehouse.planes.yaml"
    side.write_text(yaml.safe_dump(data))
    errors = gsa.check_world(WORLDS / "warehouse.sdf", side)
    assert len(errors) == 1 and "wall_east" in errors[0]


def test_warehouse_extents_and_shelf_run() -> None:
    faces = gsa.world_box_faces(WORLDS / "warehouse.sdf")
    run = [f for f in faces if f.model == "shelf_run_15m"]
    xs = sorted(f.coord_m for f in run if f.axis == 0)
    assert xs[1] - xs[0] == pytest.approx(15.0)
    planes = yaml.safe_load((WORLDS / "warehouse.planes.yaml").read_text())["planes"]
    by_name = {p["name"]: p["d"] for p in planes}
    assert by_name["wall_east"] - by_name["wall_west"] == pytest.approx(40.0)
    assert by_name["wall_north"] - by_name["wall_south"] == pytest.approx(25.0)
    assert by_name["ceiling"] - by_name["floor"] == pytest.approx(6.0)


def test_corridor_extents() -> None:
    planes = yaml.safe_load((WORLDS / "corridor.planes.yaml").read_text())["planes"]
    by_name = {p["name"]: p["d"] for p in planes}
    assert by_name["room_wall_west"] - by_name["wall_start"] == pytest.approx(60.0)
    assert by_name["corridor_wall_north"] - by_name["corridor_wall_south"] == pytest.approx(2.5)


@pytest.mark.parametrize("world", sorted(REQUIRED))
def test_floor_friction_is_concrete(world: str) -> None:
    floor = _models(world)["floor"]
    assert float(floor.findtext("link/collision/surface/friction/ode/mu") or "nan") == 0.8


@pytest.mark.parametrize("world", sorted(REQUIRED))
def test_noise_report_reads_the_sidecar(world: str) -> None:
    n, d = load_planes(WORLDS / f"{world}.planes.yaml")
    assert n.shape == (len(d), 3)
    np.testing.assert_allclose(np.linalg.norm(n, axis=1), 1.0)


def test_scenario_origin_matches_world_spherical_coordinates() -> None:
    for path in sorted((REPO / "sim/scenarios").glob("*.yaml")):
        sc = load_scenario(path)
        lla = gsa.world_spherical_lla(WORLDS / f"{sc.world}.sdf")
        assert lla == sc.origin_lla, path.name


def test_spawn_inside_the_world() -> None:
    for world in REQUIRED:
        side = gsa.load_sidecar(WORLDS / f"{world}.planes.yaml")
        x, y = side["spawn"]["x_m"], side["spawn"]["y_m"]
        planes = side["planes"]
        xs = sorted(p["d"] for p in planes if p["n"] == [1.0, 0.0, 0.0])
        ys = sorted(p["d"] for p in planes if p["n"] == [0.0, 1.0, 0.0])
        assert xs[0] < x < xs[-1] and ys[0] < y < ys[-1]


def test_check_requires_the_named_model(tmp_path: Path) -> None:
    """A plane on another model's face (shelf_run_15m's y = 1.6) fails under the wrong name."""
    data = yaml.safe_load((WORLDS / "warehouse.planes.yaml").read_text())
    data["planes"].append({"name": "wall_north", "n": [0.0, 1.0, 0.0], "d": 1.6})
    side = tmp_path / "warehouse.planes.yaml"
    side.write_text(yaml.safe_dump(data))
    errors = gsa.check_world(WORLDS / "warehouse.sdf", side)
    assert len(errors) == 1 and "wall_north" in errors[0]
    data["planes"][-1]["name"] = "shelf_run_15m"
    side.write_text(yaml.safe_dump(data))
    assert gsa.check_world(WORLDS / "warehouse.sdf", side) == []
