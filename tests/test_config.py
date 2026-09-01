import numpy as np
import pytest

from reflex_quad.config import apply_fidelity, list_experiments, load_experiment


def test_every_experiment_loads():
    ids = list_experiments()
    assert "01_stand" in ids and "06_first_step" in ids
    for exp in ids:
        cfg = load_experiment(exp)
        assert cfg["physics_dt"] > 0
        assert cfg["control_dt"] >= cfg["physics_dt"]
        assert cfg["controller"]["mode"] in ("hold", "posture", "dither", "statemachine")


def test_unknown_experiment_names_the_known_ones():
    with pytest.raises(KeyError, match="01_stand"):
        load_experiment("does_not_exist")


def test_experiment_overrides_defaults():
    cfg = load_experiment("02_uneven_ground")
    assert cfg["controller"]["mode"] == "posture"
    assert cfg["terrain"]["blocks"][0]["height"] == 0.020
    assert cfg["controller"]["gains"]["k_roll"] > 0      # inherited from defaults


def test_fidelity_stage_1_removes_all_sensor_noise():
    cfg = apply_fidelity(load_experiment("01_stand"), 1)
    assert cfg["servo"]["phase"] == "A"
    assert cfg["sensors"]["body_imu"]["accel_noise_std"] == 0.0
    assert cfg["sensors"]["foot_force"]["zero_drift_std"] == 0.0
    assert cfg["sensors"]["foot_force"]["gain_spread"] == 0.0


def test_fidelity_stage_4_randomises_the_servo():
    base = load_experiment("01_stand")["servo"]["pd"]["kp"]
    cfg = apply_fidelity(load_experiment("01_stand"), 4, np.random.default_rng(0))
    assert cfg["servo"]["phase"] == "E"
    assert cfg["servo"]["pd"]["kp"] != base


def test_experiment_can_demand_a_servo_phase():
    assert load_experiment("05_fault")["servo"]["phase"] == "D"
    assert load_experiment("01_stand")["servo"]["phase"] == "B"
