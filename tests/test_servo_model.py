import numpy as np
import pytest

from reflex_quad import N_JOINTS
from reflex_quad.servo_model import ServoBank


def bank(cfg, rng, phase="A", **over):
    servo = {**cfg["servo"], "phase": phase}
    for k, v in over.items():
        servo[k] = {**servo[k], **v} if isinstance(v, dict) else v
    return ServoBank(servo, cfg["physics_dt"], rng)


def test_pd_law_sign_and_magnitude(cfg, rng):
    s = bank(cfg, rng, "A")
    q = np.zeros(N_JOINTS)
    tau = s.step(np.full(N_JOINTS, 0.1), q, np.zeros(N_JOINTS))
    assert np.allclose(tau, 0.1 * s.kp)
    tau = s.step(q, q, np.full(N_JOINTS, 1.0))
    assert np.allclose(tau, -s.kd)


def test_phase_b_clips_torque(cfg, rng):
    s = bank(cfg, rng, "B")
    tau = s.step(np.full(N_JOINTS, 10.0), np.zeros(N_JOINTS), np.zeros(N_JOINTS))
    assert np.all(np.abs(tau) <= s.tau_max + 1e-9)


def test_phase_a_ignores_deadband_and_delay(cfg, rng):
    s = bank(cfg, rng, "A")
    tiny = np.full(N_JOINTS, 1e-4)          # well inside the deadband
    tau = s.step(tiny, np.zeros(N_JOINTS), np.zeros(N_JOINTS))
    assert np.all(np.abs(tau) > 0)


def test_phase_c_applies_deadband(cfg, rng):
    s = bank(cfg, rng, "C")
    tiny = np.full(N_JOINTS, 1e-4)
    for _ in range(50):                      # let the delay buffer fill
        tau = s.step(tiny, np.zeros(N_JOINTS), np.zeros(N_JOINTS))
    assert np.allclose(tau, 0.0)


def test_current_follows_torque_not_friction(cfg, rng):
    """Friction is a load the motor pushes through: current must still rise."""
    clean = bank(cfg, rng, "D")
    dirty = bank(cfg, np.random.default_rng(1234), "D")
    dirty.friction_scale[:] = 10.0
    q = np.zeros(N_JOINTS)
    qd = np.full(N_JOINTS, 0.5)
    tau_clean = clean.step(np.full(N_JOINTS, 0.2), q, qd)
    tau_dirty = dirty.step(np.full(N_JOINTS, 0.2), q, qd)
    assert np.all(np.abs(tau_dirty) < np.abs(tau_clean))          # less at the output
    assert np.allclose(clean.current_true, dirty.current_true)    # same motor torque


def test_current_model_and_measurement_noise(cfg, rng):
    s = bank(cfg, rng, "A")
    s.step(np.full(N_JOINTS, 0.05), np.zeros(N_JOINTS), np.zeros(N_JOINTS))
    expected = s.i_idle + s.k_tau * np.abs(s.tau_cmd)
    assert np.allclose(s.current_true, expected)
    assert np.all(s.measured_current() >= 0)


def test_power_uses_voltage(cfg, rng):
    s = bank(cfg, rng, "A")
    s.step(np.full(N_JOINTS, 0.05), np.zeros(N_JOINTS), np.zeros(N_JOINTS))
    per, total, current = s.power()
    assert total == pytest.approx(float(np.sum(per)))
    assert current == pytest.approx(float(np.sum(s.current_true)))
