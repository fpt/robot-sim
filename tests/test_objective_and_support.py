import numpy as np
import pytest

from reflex_quad.objective import Objective
from reflex_quad.support import target_forces


@pytest.fixture
def obj():
    return Objective({"w_pose": 10.0, "w_force": 0.05, "w_power": 0.005})


def test_terms_are_zero_for_a_perfect_stand(obj):
    t = obj(0.0, 0.0, np.full(4, 3.4), np.zeros(8))
    assert t.J_pose == 0 and t.J_force == 0 and t.J_power == 0


def test_force_term_uses_the_mean_when_no_target_is_given(obj):
    t = obj(0.0, 0.0, np.array([4.0, 2.0, 4.0, 2.0]), np.zeros(8))
    assert t.J_force == pytest.approx(4.0)


def test_force_term_follows_the_target_distribution(obj):
    f = np.array([0.0, 5.0, 6.8, 2.0])
    assert obj(0, 0, f, np.zeros(8), force_target=f).J_force == pytest.approx(0.0)


def test_weights_are_applied(obj):
    t = obj(0.1, 0.0, np.full(4, 3.0), np.zeros(8))
    assert t.J_total == pytest.approx(10.0 * 0.01)


FX = np.array([0.15, 0.15, -0.15, -0.15])
FY = np.array([0.09, -0.09, 0.09, -0.09])


def test_four_leg_target_is_even():
    f = target_forces(FX, FY, 13.7)
    assert np.allclose(f, 13.7 / 4)


def test_lifted_leg_target_balances_the_body():
    f = target_forces(FX, FY, 13.7, lifted=0)
    assert f[0] == 0
    assert f.sum() == pytest.approx(13.7)
    assert float(f @ FY) == pytest.approx(0.0, abs=1e-9)
    assert float(f @ FX) == pytest.approx(0.0, abs=1e-9)


def test_diagonal_partner_carries_half_not_a_third():
    """The reason a uniform W/3 target cannot be met.  See support.py."""
    f = target_forces(FX, FY, 13.7, lifted=0)
    assert f[2] == pytest.approx(13.7 / 2, rel=1e-6)     # RL, the diagonal
    assert f[2] > 13.7 / 3


def test_weight_shift_gives_the_third_leg_a_positive_margin():
    shifted = FX.copy()
    shifted[1] += 0.05                    # FR foot moved forward
    f = target_forces(shifted, FY, 13.7, lifted=0)
    assert f[3] > 0.3, "with no shift RR carries exactly zero: neutrally stable"
    assert np.all(f >= 0)
