"""Derive the E-03 robustness bag (packet loss + reordering) from a bridged bag.

Usage: python tools/mutate_bag.py <in> <out> [--drop 0.05] [--reorder 0.02]
       [--window-ns 20000000] [--seed 0]
Prints the report as one JSON line.
"""

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from mss_sim.bag_io import BagReader, BagWriter


@dataclass(frozen=True)
class MutateReport:
    n_in: int
    n_dropped: int
    n_reordered: int


def mutate(
    in_bag: Path,
    out_bag: Path,
    *,
    drop_fraction: float = 0.05,
    reorder_fraction: float = 0.02,
    window_ns: int = 20_000_000,
    topics: tuple[str, ...] = ("/imu/data", "/lidar/points"),
    seed: int = 0,
) -> MutateReport:
    """Drop and locally reorder messages on ``topics``; copy all others byte-for-byte.

    Reordering swaps the log times of a selected message and the next surviving message on
    the same topic when they are less than ``window_ns`` apart; stamps and payloads are kept.

    rng: ``SeedSequence(seed).spawn(5)[4]``, the ``mutate`` stream of the plan's spawn order
    [lidar, gnss, odom, imu, mutate] (the bridge uses the first three).
    """
    rng = np.random.default_rng(np.random.SeedSequence(seed).spawn(5)[4])
    with BagReader(in_bag) as r:
        types = r.topics()
        msgs = list(r.raw_messages(list(types)))
    n_in = len(msgs)
    target = set(topics)

    idx_target = [i for i, m in enumerate(msgs) if m[0] in target]
    keep_draw = rng.random(len(idx_target)) >= drop_fraction
    survivors = [i for i, k in zip(idx_target, keep_draw, strict=True) if k]
    dropped = set(idx_target) - set(survivors)

    log_time = {i: msgs[i][1] for i in range(n_in)}
    by_topic: dict[str, list[int]] = {}
    for i in survivors:
        by_topic.setdefault(msgs[i][0], []).append(i)
    select = dict(zip(survivors, rng.random(len(survivors)) < reorder_fraction, strict=True))
    swapped: set[int] = set()
    n_reordered = 0
    for chain in by_topic.values():
        for a, b in zip(chain, chain[1:], strict=False):
            if not select[a] or a in swapped or b in swapped:
                continue
            if msgs[b][1] - msgs[a][1] < window_ns:
                log_time[a], log_time[b] = log_time[b], log_time[a]
                swapped.update((a, b))
                n_reordered += 1

    order = sorted((log_time[i], i) for i in range(n_in) if i not in dropped)
    with BagWriter(out_bag) as w:
        for name, typename in types.items():
            w.add_topic(name, typename)
        for t, i in order:
            w.write_raw(msgs[i][0], t, msgs[i][2])
    return MutateReport(n_in, len(dropped), n_reordered)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("in_bag", type=Path)
    ap.add_argument("out_bag", type=Path)
    ap.add_argument("--drop", type=float, default=0.05)
    ap.add_argument("--reorder", type=float, default=0.02)
    ap.add_argument("--window-ns", type=int, default=20_000_000)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    report = mutate(
        a.in_bag,
        a.out_bag,
        drop_fraction=a.drop,
        reorder_fraction=a.reorder,
        window_ns=a.window_ns,
        seed=a.seed,
    )
    print(json.dumps(asdict(report)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
