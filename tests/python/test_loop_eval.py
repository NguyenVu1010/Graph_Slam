"""tools/loop_eval.py: parser, factor-chart residual, NEES, truth interpolation, errors (H6)."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

import loop_eval


def test_parse_search_reads_fields_and_vectors() -> None:
    line = (
        "j 12 t_ns 5000000000 outcome accepted n_cand 3 cand 2 cand_t_ns 1000000000 sc 0.100 "
        "ratio 0.900 rms 0.0100 eig 50.0 amb 0.000 corr_m 0.200 corr_yaw_deg 1.00 "
        "Z 1.0 0.0 0.0 0.0 0.0 0.1 sd 0.02 0.02 0.02 0.004 0.004 0.004"
    )
    s = loop_eval.parse_search(line)
    assert s["outcome"] == "accepted" and s["cand"] == "2"
    assert s["Z"] == [1.0, 0.0, 0.0, 0.0, 0.0, 0.1]
    assert s["sd"][3] == 0.004


def test_residual_is_zero_at_the_truth_and_in_the_measurement_frame() -> None:
    rot = loop_eval.so3_exp(np.array([0.0, 0.0, math.pi / 2]))
    p = np.array([1.0, 2.0, 0.0])
    assert np.allclose(loop_eval.loop_residual(rot, p, rot, p), 0.0)
    # Truth 0.1 m further along world x: the residual is that offset in the measurement frame.
    e = loop_eval.loop_residual(rot, p, rot, p + np.array([0.1, 0.0, 0.0]))
    assert np.allclose(e[:3], [0.0, -0.1, 0.0])
    assert math.isclose(loop_eval.nees(e, np.array([0.05] * 3 + [0.01] * 3)), 4.0)


def _write_tum(path: Path, rows: list[tuple[float, float, float]]) -> None:
    path.write_text("".join(f"{t:.9f} {x} {y} 0 0 0 0 1\n" for t, x, y in rows))


LOOP_LINES = (
    "[DEBUG] loop search: j 8 t_ns 40000000000 outcome accepted n_cand 1 cand 0 cand_t_ns "
    "10000000000 sc 0.1 ratio 0.9 rms 0.01 eig 50 amb 0 corr_m 0.3 corr_yaw_deg 0 "
    "Z 30.02 0 0 0 0 0 sd 0.02 0.02 0.02 0.00436 0.00436 0.00436\n"
    "[INFO] loop closure accepted: j 8 t_ns 40000000000 i 0 t_ns 10000000000 dp_m 30 0 0 "
    "dyaw_rad 0\n"
)


def _run_dir(tmp_path: Path, err_text: str | None) -> Path:
    # A straight 40 m at 1 m/s: truth at 0.02 s, an estimate stretched 1 % and exact keyframes.
    gt = [(k * 0.02, k * 0.02, 0.0) for k in range(0, 2001)]
    _write_tum(tmp_path / "gt.tum", gt)
    run = tmp_path / "run"
    run.mkdir()
    _write_tum(run / "est.tum", [(float(t), 1.01 * t, 0.0) for t in range(0, 41)])
    _write_tum(run / "kf_opt.tum", [(float(t), float(t), 0.0) for t in range(0, 41, 4)])
    (run / "bench.json").write_text(json.dumps({"n_loops_accepted": 1, "n_loops_rejected": 0}))
    if err_text is not None:
        (run / "bench.err").write_text(err_text)
    return run


def test_evaluate_end_to_end(tmp_path: Path) -> None:
    run = _run_dir(tmp_path, LOOP_LINES)
    res = loop_eval.evaluate(tmp_path / "gt.tum", run, 10.0, [(0.05, 1.0)])
    assert res["kf_opt"]["ate_m"] < 1e-9 and res["kf_opt"]["pairs"] == 11
    assert res["est"]["pairs"] == 41
    assert res["verified_loops"] == 1
    loop = res["searches"][0]
    assert math.isclose(loop["err_m"], 0.02, abs_tol=1e-9)
    assert math.isclose(loop["nees"], 1.0, rel_tol=1e-6)
    assert math.isclose(loop["nees_floors"][0], 0.16, rel_tol=1e-6)
    assert res["accepted"] == [[8, 0]]
    out = tmp_path / "results" / "r1"
    args = ["--gt", str(tmp_path / "gt.tum"), "--run", str(run), "--out", str(out)]
    assert loop_eval.main(args) == 0
    assert (out / "loop_eval.json").exists()


def test_truth_is_interpolated_between_samples() -> None:
    tr = loop_eval.Traj(
        np.array([0.0, 0.1]),
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, math.sin(0.25), math.cos(0.25)]]),
        np.array([np.eye(3), loop_eval.so3_exp(np.array([0.0, 0.0, 0.5]))]),
    )
    truth = loop_eval.Truth(tr, 0.2)
    rot, p = truth.at(0.05)
    assert np.allclose(p, [0.5, 0.0, 0.0])
    assert math.isclose(loop_eval.so3_log(rot)[2], 0.25, rel_tol=1e-9)
    with pytest.raises(loop_eval.EvalError):
        loop_eval.Truth(tr, 0.05).at(0.05)  # gap 0.1 s > 0.05 s
    with pytest.raises(loop_eval.EvalError):
        truth.at(0.2)  # outside the truth


def test_missing_or_empty_log_fails_loudly(tmp_path: Path) -> None:
    run = _run_dir(tmp_path, None)
    args = ["--gt", str(tmp_path / "gt.tum"), "--run", str(run), "--out", str(tmp_path / "o")]
    assert loop_eval.main(args) == 2  # bench.err missing
    (run / "bench.err").write_text("[INFO] nothing about loops\n")
    assert loop_eval.main(args) == 2  # verified loops counted, no loop search line
    missing_gt = ["--gt", str(tmp_path / "none.tum"), "--run", str(run), "--out", str(tmp_path)]
    assert loop_eval.main(missing_gt) == 2
