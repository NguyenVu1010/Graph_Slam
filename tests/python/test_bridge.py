"""Offline bridge: topic set, per-point time/ring, SIM-05 determinism, CLI."""

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np
from conftest import _REPO, e01_scenario

from mss_sim.bag_io import BagReader, cloud_to_arrays
from mss_sim.bridge import BridgeReport, run_bridge
from mss_sim.profile import load_profile
from mss_sim.synth import SynthInfo

Fixture = tuple[Path, SynthInfo]
_OUT_TOPICS = [
    "/lidar/points",
    "/imu/data",
    "/gnss/fix",
    "/wheel/odom",
    "/ground_truth",
    "/tf_static",
]


def _raw(bag: Path) -> dict[str, list[tuple[int, bytes]]]:
    with BagReader(bag) as r:
        out: dict[str, list[tuple[int, bytes]]] = {}
        for topic, t, data in r.raw_messages(list(r.topics())):
            out.setdefault(topic, []).append((t, data))
    return out


def _run(raw_bag: Fixture, out_dir: Path, **kw: Any) -> BridgeReport:
    raw, info = raw_bag
    sc = e01_scenario(info)
    return run_bridge(raw, sc, load_profile(sc.profile), out_dir, **kw)


def test_bridge_outputs_all_topics(bridged_bag: tuple[Path, BridgeReport]) -> None:
    out_dir, report = bridged_bag
    with BagReader(out_dir / "bag") as r:
        assert sorted(r.topics()) == sorted(_OUT_TOPICS)
    assert (out_dir / "gt.tum").exists()
    saved = json.loads((out_dir / "bridge_report.json").read_text())
    assert saved["n_msgs"] == report.n_msgs
    assert set(saved["n_msgs"]) == set(_OUT_TOPICS)
    assert report.n_msgs["/lidar/points"] == 60
    assert report.n_msgs["/wheel/odom"] > 0


def test_lidar_has_t_and_ring(bridged_bag: tuple[Path, BridgeReport]) -> None:
    out_dir, _ = bridged_bag
    with BagReader(out_dir / "bag") as r:
        msgs = list(r.messages(["/lidar/points"]))
    assert msgs
    for _, _, msg in msgs:
        a = cloud_to_arrays(msg)
        assert a.t_ns is not None
        assert a.ring is not None
        assert int(a.t_ns.max()) < 1e8
        assert msg.header.frame_id == "lidar_link"


def test_wheel_odom_fields(bridged_bag: tuple[Path, BridgeReport]) -> None:
    out_dir, _ = bridged_bag
    with BagReader(out_dir / "bag") as r:
        msgs = [m for _, _, m in r.messages(["/wheel/odom"])]
    assert all(m.child_frame_id == "base_link" and m.header.frame_id == "odom" for m in msgs)
    assert msgs[-1].pose.pose.position.x > 0.5  # robot drives forward


def test_gnss_fix_fields(bridged_bag: tuple[Path, BridgeReport]) -> None:
    out_dir, report = bridged_bag
    with BagReader(out_dir / "bag") as r:
        msgs = [m for _, _, m in r.messages(["/gnss/fix"])]
    assert len(msgs) + report.n_dropped_gnss > 0
    assert all(m.position_covariance_type == 2 for m in msgs)
    assert all(m.position_covariance[0] > 0 for m in msgs)


def test_bridge_is_deterministic(
    raw_bag: Fixture, bridged_bag: tuple[Path, BridgeReport], tmp_path: Path
) -> None:
    """SIM-05: same seed -> byte-identical messages and log times on every topic."""
    out_dir, _ = bridged_bag
    _run(raw_bag, tmp_path / "again", seed=42)
    a, b = _raw(out_dir / "bag"), _raw(tmp_path / "again" / "bag")
    assert set(a) == set(b)
    for topic in a:
        assert a[topic] == b[topic], topic


def test_different_seed_differs(
    raw_bag: Fixture, bridged_bag: tuple[Path, BridgeReport], tmp_path: Path
) -> None:
    out_dir, _ = bridged_bag
    _run(raw_bag, tmp_path / "other", seed=7)
    a, b = _raw(out_dir / "bag"), _raw(tmp_path / "other" / "bag")
    assert a["/gnss/fix"] != b["/gnss/fix"]
    assert a["/ground_truth"] == b["/ground_truth"]


def test_output_log_time_follows_raw_order(
    raw_bag: Fixture, bridged_bag: tuple[Path, BridgeReport]
) -> None:
    out_dir, _ = bridged_bag
    raw = _raw(raw_bag[0])
    out = _raw(out_dir / "bag")
    assert [t for t, _ in out["/ground_truth"]] == [t for t, _ in raw["/ground_truth"]]
    assert out["/tf_static"] == raw["/tf_static"]


def test_no_skew_flag_keeps_xyz(raw_bag: Fixture, tmp_path: Path) -> None:
    _run(raw_bag, tmp_path / "ns", seed=42, skew=False)
    with BagReader(raw_bag[0]) as r:
        raw = [cloud_to_arrays(m).xyz for _, _, m in r.messages(["/lidar/points"])]
    with BagReader(tmp_path / "ns" / "bag") as r:
        out = [cloud_to_arrays(m).xyz for _, _, m in r.messages(["/lidar/points"])]
    n_checked = 0
    for xr, xo in zip(raw, out, strict=True):
        mask = np.isfinite(xo).all(axis=1) & np.isfinite(xr).all(axis=1)
        # Bad points replace xyz; the vast majority of kept points must be untouched.
        same = (xr[mask] == xo[mask]).all(axis=1)
        assert same.mean() > 0.95
        n_checked += int(mask.sum())
    assert n_checked > 0


def test_cli_smoke(raw_bag: Fixture, tmp_path: Path) -> None:
    raw, _ = raw_bag
    out_dir = tmp_path / "cli"
    proc = subprocess.run(
        [sys.executable, "-m", "mss_sim", "bridge", str(raw),
         str(_REPO / "sim/scenarios/E-01.yaml"), str(out_dir), "--seed", "3"],
        capture_output=True, text=True, check=False,
        env={**__import__("os").environ, "PYTHONPATH": str(_REPO / "sim")},
    )  # fmt: skip
    assert proc.returncode == 0, proc.stderr
    assert "n_msgs" in json.loads(proc.stdout.strip().splitlines()[-1])
    tum = tmp_path / "gt2.tum"
    proc = subprocess.run(
        [sys.executable, "-m", "mss_sim", "gt-to-tum", str(out_dir / "bag"), str(tum)],
        capture_output=True, text=True, check=False,
        env={**__import__("os").environ, "PYTHONPATH": str(_REPO / "sim")},
    )  # fmt: skip
    assert proc.returncode == 0, proc.stderr
    assert tum.exists()
