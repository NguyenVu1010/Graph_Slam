"""Loop-closure evaluation of one ``mss_bench`` run on a recording (H6; reviews M-8, R-1).

Lives in ``tools/``, not ``eval/``: besides TUM trajectories it reads ``bench.json`` and the
pipeline's stderr log, and CONTRIBUTING.md keeps ``eval/`` to TUM files. It uses ``mss_eval``
for the trajectory parts (``PYTHONPATH=eval:sim:tools``).

Run the bench with its stderr kept, then this tool (the exact commands behind the E-07 and E-07W
numbers are in ``docs/derivations/loop_closure.md`` section 15)::

  mss_bench config/default.yaml <mssdata> --overlay <overlay.yaml> --out <run> 2> <run>/bench.err
  python tools/loop_eval.py --gt <recording>/gt.tum --run <run> --out results/<run-id> \\
      [--revisit-after-s 100] [--max-gt-gap-s 0.05] [--floor-m M --floor-deg D ...]

Inputs: the recording's ``gt.tum`` (``/ground_truth``, the OdometryPublisher of the body link
``imu_link``, the estimator's body frame ``b``, so relative truth is ``T_w_b(t_i)^-1 T_w_b(t_j)``
with no extrinsic), and the bench output directory: ``est.tum`` (online estimate, every scan),
``kf_opt.tum`` (stored keyframes at their optimised poses after the replay), ``bench.json`` and
``bench.err`` (the bench's stderr, which ``mss_bench`` does not write itself: redirect it, as
above; it holds the ``loop search:``, ``loop applied:``, ``loop closure accepted:`` and
``loop closure removed:`` lines). A missing ``gt.tum``, ``bench.json`` or ``bench.err``, or a
``bench.err`` without ``loop search`` lines while ``bench.json`` counts verified loops, is an
error (exit 2), not an empty result.

The truth is interpolated at each estimate stamp (positions linear, rotations SLERP between the
two bracketing samples); a stamp outside the truth or between samples more than
``--max-gt-gap-s`` apart is an error. The largest gap used is reported.

Writes ``<out>/loop_eval.json`` (and prints a summary):
  - ATE after SE(3) alignment of ``est.tum`` and ``kf_opt.tum`` with their pair counts (the two
    are different sample sets: every scan against keyframes only);
  - start-to-pose displacement error at the return (closest ground-truth pass to the start after
    ``--revisit-after-s``) and at the last pose, horizontal and vertical, for both files;
  - per loop search: outcome, candidate, true query-candidate distance; for a verified loop its
    error against the truth and its NEES in the loop factor's chart (``decoupled``, ADR-021:
    ``e = [R_Z^T (p_true - p_Z), Log(R_Z^T R_true)]``) with the logged floored sigmas (diagonal;
    the floor dominates every logged loop) and with each ``--floor-m/--floor-deg`` pair;
  - the bench counters and latency percentiles.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mss_eval.ate import align
from mss_eval.tum import read_tum


class EvalError(Exception):
    """A missing or inconsistent input: the tool exits 2."""


def quat_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    """Rotation matrix of a Hamilton quaternion given in TUM order (x, y, z, w)."""
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """SLERP of unit quaternions (any component order, the same for both), shortest arc."""
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


def so3_log(rot: np.ndarray) -> np.ndarray:
    c = float(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0))
    th = math.acos(c)
    v = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    if th < 1e-9:
        return v / 2.0
    return th / (2.0 * math.sin(th)) * v


def so3_exp(w: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(w))
    k = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if th < 1e-12:
        return np.eye(3) + k
    return np.eye(3) + math.sin(th) / th * k + (1.0 - math.cos(th)) / th**2 * k @ k


def loop_residual(
    rot_z: np.ndarray, p_z: np.ndarray, rot_true: np.ndarray, p_true: np.ndarray
) -> np.ndarray:
    """Residual of the decoupled loop factor at the true relative pose: [r_p, r_phi] (m, rad)."""
    return np.concatenate([rot_z.T @ (p_true - p_z), so3_log(rot_z.T @ rot_true)])


@dataclass
class Traj:
    t: np.ndarray
    p: np.ndarray
    q: np.ndarray  # (N, 4) x y z w
    rot: np.ndarray

    @classmethod
    def read(cls, path: Path) -> Traj:
        if not path.is_file():
            raise EvalError(f"{path}: missing")
        tr = read_tum(path)
        if len(tr) < 2:
            raise EvalError(f"{path}: fewer than 2 poses")
        return cls(tr.t_s, tr.p, tr.q_xyzw, np.array([quat_to_rot(q) for q in tr.q_xyzw]))

    def index(self, t: float) -> int:
        return int(np.argmin(np.abs(self.t - t)))


class Truth:
    """Ground truth interpolated in time, with a bound on the gap between samples."""

    def __init__(self, traj: Traj, max_gap_s: float) -> None:
        self.tr = traj
        self.max_gap_s = max_gap_s
        self.largest_gap_s = 0.0

    def at(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        tr = self.tr
        k = int(np.searchsorted(tr.t, t))
        if k < len(tr.t) and tr.t[k] == t:
            return tr.rot[k], tr.p[k]
        if k == 0 or k == len(tr.t):
            raise EvalError(f"ground truth does not cover t = {t:.6f} s")
        t0, t1 = tr.t[k - 1], tr.t[k]
        gap = float(t1 - t0)
        if gap > self.max_gap_s:
            raise EvalError(
                f"ground truth gap {gap:.4f} s around t = {t:.6f} s exceeds --max-gt-gap-s"
            )
        self.largest_gap_s = max(self.largest_gap_s, gap)
        u = float((t - t0) / gap)
        p = (1 - u) * tr.p[k - 1] + u * tr.p[k]
        return quat_to_rot(slerp(tr.q[k - 1], tr.q[k], u)), p


def displacement_errors(est: Traj, gt: Truth, times: list[float]) -> list[tuple[float, float]]:
    """Error of the start-to-pose displacement in the start frame, estimate vs truth: (h, v)."""
    r0, p0 = gt.at(float(est.t[0]))
    out = []
    for t in times:
        k = est.index(t)
        rk, pk = gt.at(float(est.t[k]))
        de = est.rot[0].T @ (est.p[k] - est.p[0])
        dg = r0.T @ (pk - p0)
        d = de - dg
        out.append((float(math.hypot(d[0], d[1])), float(abs(d[2]))))
    return out


SEARCH = re.compile(r"loop search: (.*)")
APPLIED = re.compile(
    r"loop applied: j (\d+) i (\d+) residual_m ([\d.]+) -> ([\d.]+) "
    r"residual_deg ([\d.]+) -> ([\d.]+)"
)
ACCEPTED = re.compile(r"loop closure accepted: j (\d+) t_ns (-?\d+) i (\d+) t_ns (-?\d+)")
REMOVED = re.compile(r"loop closure removed: j (\d+) i (\d+)")


def parse_search(text: str) -> dict:
    """Fields of one ``loop search:`` line; ``Z`` and ``sd`` are lists of 6 floats."""
    tok = text.split()
    out: dict = {}
    k = 0
    while k < len(tok):
        key = tok[k]
        if key in ("Z", "sd"):
            out[key] = [float(x) for x in tok[k + 1 : k + 7]]
            k += 7
        else:
            out[key] = tok[k + 1]
            k += 2
    return out


def nees(e: np.ndarray, sd: np.ndarray) -> float:
    return float(np.sum((e / sd) ** 2))


def evaluate(
    gt_path: Path,
    run: Path,
    revisit_after_s: float,
    floors: list[tuple[float, float]],
    max_gt_gap_s: float = 0.05,
) -> dict:
    gt = Truth(Traj.read(gt_path), max_gt_gap_s)
    for name in ("bench.json", "bench.err"):
        if not (run / name).is_file():
            raise EvalError(f"{run / name}: missing (bench.err is the redirected stderr)")
    res: dict = {}
    for name in ("est", "kf_opt"):
        path = run / f"{name}.tum"
        if not path.exists():
            continue
        al = align(path, gt_path)
        err = np.linalg.norm(al.gt - al.est_aligned, axis=1)
        tr = Traj.read(path)
        _, p_start = gt.at(float(tr.t[0]))
        after = gt.tr.t > tr.t[0] + revisit_after_s
        dist = np.linalg.norm(gt.tr.p[after][:, :2] - p_start[:2], axis=1)
        t_ret = float(gt.tr.t[after][np.argmin(dist)]) if after.any() else float(tr.t[-1])
        (rh, rv), (fh, fv) = displacement_errors(tr, gt, [t_ret, float(tr.t[-1])])
        res[name] = {
            "ate_m": float(np.sqrt(np.mean(err * err))),
            "pairs": int(len(err)),
            "t_return_s": t_ret,
            "return_h_m": rh,
            "return_v_m": rv,
            "final_h_m": fh,
            "final_v_m": fv,
        }
    bench = json.loads((run / "bench.json").read_text())
    res["bench"] = {k: v for k, v in bench.items() if k.startswith(("n_loop", "scan_process"))}
    text = (run / "bench.err").read_text(errors="replace")
    searches = []
    for m in SEARCH.finditer(text):
        s = parse_search(m.group(1))
        row: dict = {"j": int(s["j"]), "t_s": int(s["t_ns"]) * 1e-9, "outcome": s["outcome"]}
        row["n_cand"] = int(s["n_cand"])
        if row["n_cand"] > 0:
            tq, tc = int(s["t_ns"]) * 1e-9, int(s["cand_t_ns"]) * 1e-9
            rq, pq = gt.at(tq)
            rc, pc = gt.at(tc)
            row["cand"] = int(s["cand"])
            row["true_dist_m"] = float(np.linalg.norm(pq - pc))
            for key in ("sc", "ratio", "rms", "eig", "amb", "corr_m", "corr_yaw_deg"):
                row[key] = float(s[key])
            if "Z" in s:
                z = np.array(s["Z"])
                e = loop_residual(so3_exp(z[3:]), z[:3], rc.T @ rq, rc.T @ (pq - pc))
                sd = np.array(s["sd"])
                row["err_m"] = float(np.linalg.norm(e[:3]))
                row["err_deg"] = float(np.degrees(np.linalg.norm(e[3:])))
                row["nees"] = nees(e, sd)
                row["nees_floors"] = [
                    nees(e, np.array([fm] * 3 + [math.radians(fd)] * 3)) for fm, fd in floors
                ]
        searches.append(row)
    verified_counted = int(bench.get("n_loops_accepted", 0)) + int(bench.get("n_loops_rejected", 0))
    if verified_counted > 0 and not searches:
        raise EvalError(
            f"{run / 'bench.err'}: bench.json counts {verified_counted} verified loops but the log "
            "has no 'loop search' line (is it the bench's stderr?)"
        )
    res["searches"] = searches
    outcomes: dict[str, int] = {}
    for row in searches:
        outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
    res["outcomes"] = outcomes
    verified = [r for r in searches if "nees" in r]
    res["verified_loops"] = len(verified)
    if verified:
        res["max_err_m"] = max(r["err_m"] for r in verified)
        res["max_err_deg"] = max(r["err_deg"] for r in verified)
        res["mean_nees"] = float(np.mean([r["nees"] for r in verified]))
        res["mean_nees_floors"] = [
            float(np.mean([r["nees_floors"][k] for r in verified])) for k in range(len(floors))
        ]
    res["floors"] = floors
    res["largest_gt_gap_s"] = gt.largest_gap_s
    res["applied"] = [
        {"j": int(m[1]), "i": int(m[2]), "before_m": float(m[3]), "after_m": float(m[4])}
        for m in APPLIED.finditer(text)
    ]
    res["accepted"] = [[int(m[1]), int(m[3])] for m in ACCEPTED.finditer(text)]
    res["removed"] = [[int(m[1]), int(m[2])] for m in REMOVED.finditer(text)]
    return res


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--run", type=Path, required=True, help="mss_bench --out directory")
    ap.add_argument("--out", type=Path, required=True, help="results/<run-id>")
    ap.add_argument("--revisit-after-s", type=float, default=100.0)
    ap.add_argument(
        "--max-gt-gap-s", type=float, default=0.05, help="largest truth gap to interpolate, s"
    )
    ap.add_argument("--floor-m", type=float, action="append", default=[])
    ap.add_argument("--floor-deg", type=float, action="append", default=[])
    a = ap.parse_args(argv)
    if len(a.floor_m) != len(a.floor_deg):
        ap.error("--floor-m and --floor-deg come in pairs")
    try:
        res = evaluate(
            a.gt,
            a.run,
            a.revisit_after_s,
            list(zip(a.floor_m, a.floor_deg, strict=True)),
            a.max_gt_gap_s,
        )
    except EvalError as e:
        print(f"loop_eval: {e}", file=sys.stderr)
        return 2
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "loop_eval.json").write_text(json.dumps(res, indent=1))
    summary = {k: v for k, v in res.items() if k not in ("searches", "applied")}
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
