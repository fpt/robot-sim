import numpy as np
import pytest

from reflex_quad.observer import AttitudeObserver
from reflex_quad.sensors import SensorSuite

GRAVITY = 9.81


@pytest.fixture
def suite(cfg, rng):
    return SensorSuite(cfg["sensors"], cfg["physics_dt"], rng)


def test_fsr_saturates_and_never_goes_negative(suite):
    for _ in range(50):
        f = suite.fsr.read(np.array([-5.0, 0.0, 5.0, 500.0]))
    assert f[0] >= 0.0
    assert f[3] == pytest.approx(suite.fsr.f_max, abs=1e-6)


def test_fsr_stuck_freezes_one_channel(suite):
    for _ in range(200):
        suite.fsr.read(np.full(4, 4.0))
    suite.fsr.stuck[1] = True
    frozen = suite.fsr.read(np.full(4, 4.0))[1]
    for value in (0.0, 9.0, 2.0):
        assert suite.fsr.read(np.full(4, value))[1] == frozen


def test_imu_bias_is_fixed_per_run_not_per_sample(suite):
    a1, _ = suite.body_imu.read(np.array([0, 0, GRAVITY]), np.zeros(3))
    a2, _ = suite.body_imu.read(np.array([0, 0, GRAVITY]), np.zeros(3))
    assert not np.allclose(a1, a2)                       # noise moves
    assert np.allclose(suite.body_imu.accel_bias, suite.body_imu.accel_bias)


def test_observer_recovers_a_static_tilt(cfg):
    obs = AttitudeObserver(cfg["sensors"]["observer"], 0.002)
    roll, pitch = 0.10, -0.05
    # specific force of a body tilted by (roll, pitch), small angle
    accel = np.array([-pitch * GRAVITY, roll * GRAVITY, GRAVITY])
    for _ in range(3000):
        obs.update(accel, np.zeros(3))
    assert obs.roll == pytest.approx(roll, abs=2e-3)
    assert obs.pitch == pytest.approx(pitch, abs=2e-3)


def test_observer_integrates_the_gyro_between_accel_updates(cfg):
    obs = AttitudeObserver(cfg["sensors"]["observer"], 0.01)
    obs.update(np.array([0.0, 0.0, GRAVITY]), np.zeros(3))
    start = obs.roll
    for _ in range(50):                       # 0.5 s at 1 rad/s, accel unusable
        obs.update(np.array([0.0, 0.0, 40.0]), np.array([1.0, 0.0, 0.0]))
    assert obs.roll - start == pytest.approx(0.5, abs=1e-6)
