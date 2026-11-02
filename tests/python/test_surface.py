"""Ground surfaces of the outdoor world (backlog L3): sidecar vs SDF tiles, lookup, validation,
the send decisions of SurfaceCommander and the gz service request."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

import gen_sim_assets as gsa
from mss_sim.surface import (
    RESEND_NS,
    Surface,
    SurfaceCommander,
    SurfaceError,
    load_surfaces,
    wheel_slip_request,
)

REPO = Path(__file__).resolve().parents[2]
WORLDS = REPO / "sim/worlds"
OUTDOOR = WORLDS / "outdoor.planes.yaml"


def _tile_mu(world: Path) -> dict[str, float]:
    """Friction ``mu`` of every ground tile (models named ``ground_*``)."""
    root = ET.parse(world).getroot()
    out = {}
    for m in root.iter("model"):
        name = m.get("name", "")
        if name.startswith("ground_"):
            mu = m.findtext("link/collision/surface/friction/ode/mu")
            mu2 = m.findtext("link/collision/surface/friction/ode/mu2")
            assert mu is not None and mu == mu2, name
            out[name] = float(mu)
    return out


def test_indoor_worlds_declare_no_surfaces() -> None:
    assert load_surfaces(WORLDS / "warehouse.planes.yaml") is None
    assert load_surfaces(WORLDS / "corridor.planes.yaml") is None


def test_outdoor_patches_match_their_sdf_tiles() -> None:
    surfaces = load_surfaces(OUTDOOR)
    assert surfaces is not None
    mu = _tile_mu(WORLDS / "outdoor.sdf")
    boxes = {b.model: b for b in gsa.world_boxes(WORLDS / "outdoor.sdf")}
    for s in surfaces.patches:
        assert s.model is not None and s.rect_m is not None
        assert mu[s.model] == s.mu, s.name
        b = boxes[s.model]
        assert (b.lo[0], b.lo[1], b.hi[0], b.hi[1]) == pytest.approx(s.rect_m), s.name
        assert b.hi[2] == pytest.approx(0.0)  # flush with the rest of the ground
    patch_models = {s.model for s in surfaces.patches}
    for model, value in mu.items():
        if model not in patch_models:
            assert value == surfaces.default.mu, model


def test_outdoor_ground_tiles_cover_the_world_without_overlap() -> None:
    tiles = [b for b in gsa.world_boxes(WORLDS / "outdoor.sdf") if b.model.startswith("ground_")]
    area = sum((b.hi[0] - b.lo[0]) * (b.hi[1] - b.lo[1]) for b in tiles)
    assert area == pytest.approx(120.0 * 120.0)
    for i, a in enumerate(tiles):
        for b in tiles[i + 1 :]:
            overlap_x = min(a.hi[0], b.hi[0]) - max(a.lo[0], b.lo[0])
            overlap_y = min(a.hi[1], b.hi[1]) - max(a.lo[1], b.lo[1])
            assert overlap_x <= 0 or overlap_y <= 0, (a.model, b.model)


def test_lookup_and_compliance_rule() -> None:
    surfaces = load_surfaces(OUTDOOR)
    assert surfaces is not None
    assert surfaces.at(-14.0, -15.0).name == "wet"
    assert surfaces.at(20.0, -25.0).name == "grass"
    assert surfaces.at(0.0, -15.0).name == "concrete"
    assert surfaces.at(40.0, -15.0).name == "concrete"  # x_max is exclusive
    assert surfaces.default.slip_compliance_longitudinal == 0.0
    for s in surfaces.patches:  # secant to the friction peak: slip 0.2 at mu (sim/README.md)
        assert s.slip_compliance_longitudinal == pytest.approx(0.2 / s.mu, abs=0.005)
        assert s.slip_compliance_lateral == s.slip_compliance_longitudinal


def _write(tmp_path: Path, data: dict) -> Path:
    p = tmp_path / "x.planes.yaml"
    p.write_text(yaml.safe_dump(data))
    return p


@pytest.mark.parametrize(
    ("change", "message"),
    [
        (lambda d: d["surfaces"][0].pop("mu"), r"surfaces\[0\]\.mu: missing"),
        (lambda d: d["surfaces"][0].update(mu=-0.1), r"surfaces\[0\]\.mu: must be >= 0"),
        (lambda d: d["surfaces"][0].update(rect_m=[0, 0, 0, 1]), "x_min < x_max"),
        (lambda d: d["surfaces"][1].update(rect_m=[-15, -20, 12, -12]), "overlap"),
        (lambda d: d["surfaces"][0].update(colour="red"), "unknown key"),
        (lambda d: d.update(surfaces=[]), "non-empty list"),
        (lambda d: d["default_surface"].update(slip_compliance_lateral=float("nan")), "finite"),
    ],
)
def test_invalid_surfaces_are_rejected(tmp_path: Path, change: object, message: str) -> None:
    data = yaml.safe_load(OUTDOOR.read_text())
    change(data)  # type: ignore[operator]
    with pytest.raises(SurfaceError, match=message):
        load_surfaces(_write(tmp_path, data))


class _FakeSender:
    def __init__(self, replies: list[bool]) -> None:
        self.replies = replies
        self.sent: list[tuple[str, float]] = []

    def __call__(self, s: Surface) -> bool:
        self.sent.append((s.name, s.slip_compliance_longitudinal))
        return self.replies.pop(0) if self.replies else True


def _commander(replies: list[bool]) -> tuple[SurfaceCommander, _FakeSender]:
    surfaces = load_surfaces(OUTDOOR)
    assert surfaces is not None
    fake = _FakeSender(replies)
    return SurfaceCommander(surfaces, fake, resend_ns=RESEND_NS), fake


def test_commander_sends_once_per_surface_change() -> None:
    cmd, fake = _commander([])
    t = 0
    changes = []
    for x in np.arange(-30.0, 20.0, 0.01):  # concrete -> wet -> concrete -> grass
        t += 5_000_000  # 200 Hz
        s = cmd.update(t, float(x), -15.0)
        if s is not None:
            changes.append(s.name)
    assert changes == ["concrete", "wet", "concrete", "grass"]
    assert [n for n, _ in fake.sent] == changes
    assert cmd.n_sent == 4 and cmd.n_failed == 0 and cmd.acknowledged


def test_commander_retries_a_failed_send_after_the_resend_period() -> None:
    cmd, fake = _commander([False, False, True])
    t0 = 1_000_000_000
    assert cmd.update(t0, -14.0, -15.0) is not None  # wet: first try fails
    assert len(fake.sent) == 1
    cmd.update(t0 + RESEND_NS - 1, -14.0, -15.0)  # within the period: no retry
    assert len(fake.sent) == 1
    cmd.update(t0 + RESEND_NS, -14.0, -15.0)  # retry, fails again
    cmd.update(t0 + 2 * RESEND_NS, -14.0, -15.0)  # retry, succeeds
    cmd.update(t0 + 5 * RESEND_NS, -14.0, -15.0)  # acknowledged: nothing more
    assert [n for n, _ in fake.sent] == ["wet"] * 3
    assert cmd.n_failed == 2 and cmd.acknowledged


def test_commander_change_resets_a_pending_retry() -> None:
    cmd, fake = _commander([False, True])
    cmd.update(0, -14.0, -15.0)  # wet fails
    cmd.update(1, 0.0, -15.0)  # concrete at once, not after the period
    assert [n for n, _ in fake.sent] == ["wet", "concrete"]


def test_wheel_slip_request_text() -> None:
    surfaces = load_surfaces(OUTDOOR)
    assert surfaces is not None
    req = wheel_slip_request("robot", surfaces.at(20.0, -25.0))
    assert req == (
        'entity {name: "robot" type: MODEL} slip_compliance_lateral: 0.4 '
        "slip_compliance_longitudinal: 0.4"
    )
