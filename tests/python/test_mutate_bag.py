"""mutate_bag: packet loss + reordering for the E-03 robustness bag."""

import json
import subprocess
import sys
from pathlib import Path

from conftest import _REPO

from mss_sim.bag_io import BagReader, BagWriter, stamp_ns
from mss_sim.bridge import BridgeReport
from mutate_bag import MutateReport, mutate

_TARGETS = ["/imu/data", "/lidar/points"]


def _raw(bag: Path) -> list[tuple[str, int, bytes]]:
    with BagReader(bag) as r:
        return list(r.raw_messages(list(r.topics())))


def _mutated(
    bridged: tuple[Path, BridgeReport], tmp_path: Path, name: str, seed: int
) -> tuple[Path, MutateReport]:
    out = tmp_path / name
    report = mutate(bridged[0] / "bag", out, seed=seed)
    return out, report


def test_drop_fraction_and_passthrough(
    bridged_bag: tuple[Path, BridgeReport], tmp_path: Path
) -> None:
    src = _raw(bridged_bag[0] / "bag")
    out, report = _mutated(bridged_bag, tmp_path, "m", 0)
    dst = _raw(out)
    n_target = sum(1 for t, _, _ in src if t in _TARGETS)
    n_target_out = sum(1 for t, _, _ in dst if t in _TARGETS)
    assert isinstance(report, MutateReport)
    assert report.n_in == len(src)
    assert report.n_dropped == n_target - n_target_out
    assert abs(report.n_dropped / n_target - 0.05) < 0.01
    gt = lambda msgs: [m for m in msgs if m[0] == "/ground_truth"]  # noqa: E731
    assert gt(dst) == gt(src) and gt(src)
    other = lambda msgs: [m for m in msgs if m[0] not in _TARGETS]  # noqa: E731
    assert other(dst) == other(src)
    assert [t for _, t, _ in dst] == sorted(t for _, t, _ in dst)


def test_reordering_present(bridged_bag: tuple[Path, BridgeReport], tmp_path: Path) -> None:
    out, report = _mutated(bridged_bag, tmp_path, "m", 0)
    assert report.n_reordered > 0
    inversions = 0
    with BagReader(out) as r:
        last: dict[str, tuple[int, int]] = {}
        for topic, t, msg in r.messages(_TARGETS):
            s = stamp_ns(msg.header)
            if topic in last and t > last[topic][0] and s < last[topic][1]:
                inversions += 1
            last[topic] = (t, s)
    assert inversions >= 1


def test_seed_determinism(bridged_bag: tuple[Path, BridgeReport], tmp_path: Path) -> None:
    a, _ = _mutated(bridged_bag, tmp_path, "a", 5)
    b, _ = _mutated(bridged_bag, tmp_path, "b", 5)
    c, _ = _mutated(bridged_bag, tmp_path, "c", 6)
    assert _raw(a) == _raw(b)
    assert _raw(a) != _raw(c)


def test_write_raw_roundtrip(bridged_bag: tuple[Path, BridgeReport], tmp_path: Path) -> None:
    with BagReader(bridged_bag[0] / "bag") as r:
        types = r.topics()
        first = next(r.raw_messages(["/imu/data"]))
    with BagWriter(tmp_path / "w") as w:
        w.add_topic("/imu/data", types["/imu/data"])
        w.write_raw("/imu/data", first[1] + 1, first[2])
    assert _raw(tmp_path / "w") == [("/imu/data", first[1] + 1, first[2])]


def test_cli_smoke(bridged_bag: tuple[Path, BridgeReport], tmp_path: Path) -> None:
    proc = subprocess.run(
        [sys.executable, str(_REPO / "tools/mutate_bag.py"), str(bridged_bag[0] / "bag"),
         str(tmp_path / "cli"), "--seed", "1"],
        capture_output=True, text=True, check=False,
        env={**__import__("os").environ, "PYTHONPATH": str(_REPO / "sim")},
    )  # fmt: skip
    assert proc.returncode == 0, proc.stderr
    rep = json.loads(proc.stdout.strip().splitlines()[-1])
    assert set(rep) == {"n_in", "n_dropped", "n_reordered"}
