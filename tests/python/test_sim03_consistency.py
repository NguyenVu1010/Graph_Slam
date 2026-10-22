"""SIM-03 (ADR-011 section 8): pipeline parameters are consistent with the simulated sensors."""

from pathlib import Path

import pytest

from mss_sim.config_overlay import get, load_pipeline_config
from mss_sim.profile import NoiseProfile, load_profile
from mss_sim.scenario import load_scenario

REPO = Path(__file__).resolve().parents[2]
ADR_T_B_L_TRANSLATION_M = [0.10, 0.0, 0.50]
ADR_T_B_L_QUATERNION_WXYZ = [0.99984770, 0.0, 0.01745241, 0.0]
ADR_P_B_G_M = [-0.20, 0.10, 0.60]


def pairs(prof: NoiseProfile) -> list[tuple[str, float]]:
    return [
        ("imu.gyro_noise_density", prof.imu.gyro_noise_density),
        ("imu.acc_noise_density", prof.imu.acc_noise_density),
        ("imu.gyro_random_walk", prof.imu.gyro_random_walk),
        ("imu.acc_random_walk", prof.imu.acc_random_walk),
        ("gnss.sigma_h_m", prof.gnss.sigma_h_m),
        ("gnss.sigma_v_m", prof.gnss.sigma_v_m),
        ("lidar.range_sigma_m", prof.lidar.range_sigma_m),
        ("wheel_odom.speed_sigma_mps", prof.odom.speed_sigma_mps),
    ]


SCENARIOS = sorted((REPO / "sim/scenarios").glob("*.yaml"))


def _load(scenario: Path) -> tuple[NoiseProfile, dict]:
    sc = load_scenario(scenario)
    prof = load_profile(REPO / sc.profile)
    cfg = load_pipeline_config(REPO / "config/default.yaml", REPO / sc.pipeline_config)
    return prof, cfg


def test_scenarios_exist() -> None:
    assert SCENARIOS


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda p: p.name)
def test_pipeline_noise_within_inflation(scenario: Path) -> None:
    prof, cfg = _load(scenario)
    for key_cfg, val_sim in pairs(prof):
        ratio = get(cfg, key_cfg) / val_sim
        assert 1.0 <= ratio <= prof.inflation_max, f"{key_cfg}: ratio {ratio}"


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda p: p.name)
def test_pipeline_uses_believed_kinematics(scenario: Path) -> None:
    prof, cfg = _load(scenario)
    believed = prof.odom.kinematics_believed
    assert get(cfg, "wheel_odom.wheel_radius_m") == believed.wheel_radius_m
    assert get(cfg, "wheel_odom.track_m") == believed.track_m
    assert get(cfg, "frames.p_b_g_m") == ADR_P_B_G_M
    assert get(cfg, "frames.T_b_l.translation_m") == ADR_T_B_L_TRANSLATION_M
    assert get(cfg, "frames.T_b_l.quaternion_wxyz") == ADR_T_B_L_QUATERNION_WXYZ
