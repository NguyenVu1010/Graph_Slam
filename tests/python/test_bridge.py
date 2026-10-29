"""Offline bridge: topic set, per-point time/ring, SIM-05 determinism, CLI."""

import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from conftest import _REPO, cli_env, copy_bag_without, e01_scenario

from mss_sim.bag_io import BagReader, cloud_to_arrays, stamp_ns
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


def test_wheel_odom_fields(raw_bag: Fixture, bridged_bag: tuple[Path, BridgeReport]) -> None:
    out_dir, _ = bridged_bag
    with BagReader(raw_bag[0]) as r:
        tfs = [ts for _, _, m in r.messages(["/tf_static"]) for ts in m.transforms]
    body = next(ts.header.frame_id for ts in tfs if ts.child_frame_id == "lidar_link")
    with BagReader(out_dir / "bag") as r:
        msgs = [m for _, _, m in r.messages(["/wheel/odom"])]
    assert msgs
    assert all(m.child_frame_id == body and m.header.frame_id == "odom" for m in msgs)
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
        env=cli_env(),
    )  # fmt: skip
    assert proc.returncode == 0, proc.stderr
    assert "n_msgs" in json.loads(proc.stdout.strip().splitlines()[-1])
    tum = tmp_path / "gt2.tum"
    proc = subprocess.run(
        [sys.executable, "-m", "mss_sim", "gt-to-tum", str(out_dir / "bag"), str(tum)],
        capture_output=True, text=True, check=False,
        env=cli_env(),
    )  # fmt: skip
    assert proc.returncode == 0, proc.stderr
    assert tum.exists()


def test_clock_offsets_shift_stamp_and_log_time(raw_bag: Fixture, tmp_path: Path) -> None:
    offsets = {"imu": 1_000, "lidar": 2_000, "gnss": 3_000, "odom": 4_000}
    raw, info = raw_bag
    sc = e01_scenario(info)  # nominal GNSS only: no fix is dropped
    profile = replace(load_profile(sc.profile), clock_offset_ns=offsets)
    report = run_bridge(raw, sc, profile, tmp_path / "off", seed=42)
    assert report.n_dropped_gnss == 0

    def read(bag: Path, topics: list[str]) -> dict[str, list[tuple[int, int]]]:
        out: dict[str, list[tuple[int, int]]] = {t: [] for t in topics}
        with BagReader(bag) as r:
            for topic, t, msg in r.messages(topics):
                out[topic].append((t, stamp_ns(msg.header)))
        return out

    topics = ["/imu/data", "/lidar/points", "/gnss/fix", "/joint_states", "/wheel/odom"]
    raw_t = read(raw, topics[:4])
    out_t = read(tmp_path / "off" / "bag", [topics[i] for i in (0, 1, 2, 4)])
    # (output topic, raw topic, sensor key, raw index of the first message with output)
    cases = [
        ("/imu/data", "/imu/data", "imu", 0),
        ("/lidar/points", "/lidar/points", "lidar", 0),
        ("/gnss/fix", "/gnss/fix", "gnss", 0),
        ("/wheel/odom", "/joint_states", "odom", 1),  # first joint state only sets the time base
    ]
    for out_topic, raw_topic, key, first in cases:
        src = raw_t[raw_topic][first:]
        dst = out_t[out_topic]
        assert len(dst) == len(src) > 0, out_topic
        for (t_out, s_out), (t_raw, s_raw) in zip(dst, src, strict=True):
            assert t_out - t_raw == offsets[key], out_topic
            assert s_out - s_raw == offsets[key], out_topic


def test_missing_tf_static_raises(raw_bag: Fixture, tmp_path: Path) -> None:
    raw, info = raw_bag
    no_tf = copy_bag_without(raw, tmp_path / "no_tf", "/tf_static")
    with pytest.raises(ValueError, match="lidar_link"):
        run_bridge(no_tf, e01_scenario(info), load_profile(e01_scenario(info).profile), tmp_path)


def test_report_has_gnss_rejected(bridged_bag: tuple[Path, BridgeReport]) -> None:
    out_dir, report = bridged_bag
    assert report.n_gnss_rejected == 0
    saved = json.loads((out_dir / "bridge_report.json").read_text())
    assert saved["n_gnss_rejected"] == 0
