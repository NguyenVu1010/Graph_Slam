"""CLI: ``python -m mss_sim bridge|gt-to-tum``."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

from mss_sim.bag_io import BagReader
from mss_sim.bridge import run_bridge
from mss_sim.gt import GroundTruth
from mss_sim.profile import load_profile
from mss_sim.scenario import load_scenario

_GT_TOPIC = "/ground_truth"


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="mss_sim", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    b = sub.add_parser("bridge", help="raw bag -> noisy bag")
    b.add_argument("raw_bag", type=Path)
    b.add_argument("scenario", type=Path, help="scenario YAML")
    b.add_argument("out_dir", type=Path, help="output directory; its 'bag' subdir must not exist")
    b.add_argument("--seed", type=int, default=None, help="override the scenario seed")
    b.add_argument("--no-skew", action="store_true", help="do not inject motion skew")
    g = sub.add_parser("gt-to-tum", help="export /ground_truth of a bag as a TUM file")
    g.add_argument("bag", type=Path)
    g.add_argument("out", type=Path)
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == "bridge":
        scenario = load_scenario(args.scenario)
        report = run_bridge(
            args.raw_bag,
            scenario,
            load_profile(scenario.profile),
            args.out_dir,
            seed=args.seed,
            skew=not args.no_skew,
        )
        print(json.dumps(asdict(report)))
    else:
        with BagReader(args.bag) as reader:
            gt = GroundTruth.from_odometry_msgs(m for _, _, m in reader.messages([_GT_TOPIC]))
        gt.to_tum(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
