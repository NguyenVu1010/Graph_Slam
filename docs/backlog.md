# Backlog

The project builds one end-to-end flow first:
- slice A: an offline map from a bag;
- slice B: the ROS 2 node and RViz, from a bag and then live in Gazebo;
- slice C: loop closure.

While those slices are built, a PR is merged once its blockers are fixed. A blocker is wrong output, a
crash, broken CI, or a change in behaviour with the feature flags off. Every other finding is listed
here and reviewed before each merge. After slice C, the hardening phase works through this list, and
every review finding is fixed before merge again.

Each item names where its numbers live.

## GNSS

| # | Item | Source | Notes |
|---|------|--------|-------|
| G1 | The default `GnssAligner` (flags of designs 1/2 off) aligns E-01 about 21° off at 13 s and reports no yaw sigma | design-2 review (#34) | First PR after slice B, or earlier if the slices need GNSS on. |
| G2 | Design 3: inflate each fix's covariance by the shift uncertainty; then decide the defaults of the design 1/2/2b/3 flags | `gnss_fusion_study.md` §13.6 | Measure it against `prealign_trackyaw`. |
| G3 | Design 2b: the front-end travel gate (number of scan-match constraints along the travel axis, at least 100) partly reverses ADR-023 decision 13 | ADR-023 amendment, study §16 | It needs the owner's confirmation and a check on a recorded Gazebo bag; compare it with the degeneracy module's flagged directions. |
| G4 | Design 2b: the residual limits outside LiDAR-observed scenes (the tolerance envelope) | ADR-023, lessons §1.18 | GNSS drift ≤ 0.15 m per fix with stuck wheels and ≤ 0.10 m per fix parked without wheels, skid ≤ 5°. |
| G5 | A first-yaw bias of 1.4° on noise-free S-jogs at 0.5 m/s, from the design-2 heading branch | design-2b root-cause study | Cause still to confirm. |
| G6 | GNSS Doppler velocity for the frame yaw: an ADR candidate | study §16 | Check whether the receiver outputs RMC/VTG or NAV-PVT velocity. |
| G7 | A shared stationarity detector (IMU + wheel + front end), used by GNSS and a possible zero-velocity factor | lessons §1.18 | Partly covered by 2b in scenes the LiDAR observes. |
| G8 | An "aligned" flag on the ENU output | study §12 | Planned for S2. |

## Degeneracy

| # | Item | Source |
|---|------|--------|
| D1 | PR 4b-2 (`feat/degeneracy-sigma`) is on hold until the map edge-leak fix | lessons §1.6 |
| D2 | Retune `unobserved_var_m2` toward 0.01 | 4b-1 (#29) |

## Simulation and evaluation (S2)

| # | Item |
|---|------|
| S1 | Outdoor and loop worlds, `testdata-v1`, WheelSlip |
| S2 | E-04 vertical drift; the E-01 vertical offset |
| S3 | Bench dump of the degeneracy state |
| S4 | `run_integration` variants |
