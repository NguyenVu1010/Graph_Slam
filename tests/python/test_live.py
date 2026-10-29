"""Online noise layer (``mss_sim.live``): same output as the offline bridge for the same input
and seed, scans held until the ground truth covers them, start-up queues."""

from pathlib import Path

from conftest import e01_scenario

from mss_sim.bag_io import TYPESTORE, BagReader, cloud_to_arrays, stamp_ns
from mss_sim.bridge import BridgeReport
from mss_sim.live import GNSS, GT, IMU, IN_TYPES, JOINTS, LIDAR, MAX_PENDING, ODOM, TF, LiveNoise
from mss_sim.profile import load_profile
from mss_sim.synth import SynthInfo

Fixture = tuple[Path, SynthInfo]


def _raw_in_log_order(bag: Path) -> list[tuple[str, bytes]]:
    with BagReader(bag) as r:
        topics = [t for t in IN_TYPES if t in r.topics()]
        return [(t, data) for t, _, data in r.raw_messages(topics)]


def _offline(bag: Path) -> dict[str, list[bytes]]:
    out: dict[str, list[bytes]] = {}
    with BagReader(bag) as r:
        for topic, _, data in r.raw_messages([IMU, LIDAR, GNSS, ODOM]):
            out.setdefault(topic, []).append(data)
    return out


def _live(raw_bag: Fixture, msgs: list[tuple[str, bytes]], seed: int = 42) -> tuple:
    sc = e01_scenario(raw_bag[1])
    live = LiveNoise(sc, load_profile(sc.profile), seed=seed)
    out: dict[str, list[bytes]] = {}
    for topic, data in msgs:
        for t, cdr in live.push(topic, data):
            out.setdefault(t, []).append(cdr)
    return live, out


def test_same_bytes_as_offline_bridge(
    raw_bag: Fixture, bridged_bag: tuple[Path, BridgeReport]
) -> None:
    """Raw messages in log order, seed 42: every noisy message equals the offline bag's."""
    out_dir, report = bridged_bag
    live, out = _live(raw_bag, _raw_in_log_order(raw_bag[0]))
    offline = _offline(out_dir / "bag")
    assert set(out) == set(offline) == {IMU, LIDAR, GNSS, ODOM}
    for topic in offline:
        # The last scan(s) wait for ground truth past the end of the bag: a prefix, no clamp.
        n = len(out[topic])
        assert n == len(offline[topic]) or topic == LIDAR, topic
        assert out[topic] == offline[topic][:n], topic
    assert len(offline[LIDAR]) - len(out[LIDAR]) <= 1
    assert live.counters.n_gt_clamped == 0
    layer = live.layer
    assert layer is not None
    assert layer.n_dropped_gnss == report.n_dropped_gnss
    assert layer.quantizer.errors == report.odom_errors


def test_sensor_interleaving_does_not_matter(
    raw_bag: Fixture, bridged_bag: tuple[Path, BridgeReport]
) -> None:
    """All GNSS first, /tf_static 300 messages (~0.5 s) late (start-up queues): same bytes."""
    msgs = _raw_in_log_order(raw_bag[0])
    gnss = [m for m in msgs if m[0] == GNSS]
    tf = [m for m in msgs if m[0] == TF]
    rest = [m for m in msgs if m[0] not in (GNSS, TF)]
    regrouped = gnss + rest[:300] + tf + rest[300:]
    live, out = _live(raw_bag, regrouped)
    offline = _offline(bridged_bag[0] / "bag")
    for topic in offline:
        n = len(out[topic])
        assert n > 0
        assert out[topic] == offline[topic][:n], topic
    assert len(out[GNSS]) == len(offline[GNSS])  # all held, none lost
    assert sum(live.counters.n_overflow.values()) == 0


def test_scan_waits_for_ground_truth(raw_bag: Fixture) -> None:
    msgs = _raw_in_log_order(raw_bag[0])
    sc = e01_scenario(raw_bag[1])
    live = LiveNoise(sc, load_profile(sc.profile), seed=1)
    tf = next(d for t, d in msgs if t == TF)
    gts = [d for t, d in msgs if t == GT]
    scan = next(d for t, d in msgs if t == LIDAR)
    scan_ns = stamp_ns(TYPESTORE.deserialize_cdr(scan, IN_TYPES[LIDAR]).header)
    assert live.push(TF, tf) == []
    assert live.push(LIDAR, scan) == []  # no ground truth yet
    released: list[tuple[str, bytes]] = []
    for d in gts:
        t = stamp_ns(TYPESTORE.deserialize_cdr(d, IN_TYPES[GT]).header)
        got = live.push(GT, d)
        if got:
            assert t >= scan_ns + 100_000_000  # the scan period of the default profile
            released = got
            break
    assert [t for t, _ in released] == [LIDAR]
    cloud = cloud_to_arrays(TYPESTORE.deserialize_cdr(released[0][1], IN_TYPES[LIDAR]))
    assert cloud.t_ns is not None and cloud.ring is not None


def test_imu_passes_before_start(raw_bag: Fixture) -> None:
    msgs = _raw_in_log_order(raw_bag[0])
    sc = e01_scenario(raw_bag[1])
    live = LiveNoise(sc, load_profile(sc.profile))
    imu = next(d for t, d in msgs if t == IMU)
    assert live.push(IMU, imu) == [(IMU, imu)]  # zero clock offset: bytes unchanged
    joint = next(d for t, d in msgs if t == JOINTS)
    assert live.push(JOINTS, joint) == []
    assert not live.started


def test_different_seed_differs(raw_bag: Fixture) -> None:
    msgs = _raw_in_log_order(raw_bag[0])
    _, a = _live(raw_bag, msgs, seed=42)
    _, b = _live(raw_bag, msgs, seed=7)
    assert a[GNSS] != b[GNSS]
    assert a[IMU] == b[IMU]


def test_start_up_queue_is_bounded(raw_bag: Fixture) -> None:
    """Scans without ground truth: the oldest are dropped and counted, nothing is emitted."""
    msgs = _raw_in_log_order(raw_bag[0])
    sc = e01_scenario(raw_bag[1])
    live = LiveNoise(sc, load_profile(sc.profile))
    scans = [d for t, d in msgs if t == LIDAR]
    assert len(scans) > MAX_PENDING[LIDAR]
    for d in scans:
        assert live.push(LIDAR, d) == []
    assert live.counters.n_overflow[LIDAR] == len(scans) - MAX_PENDING[LIDAR]
