import numpy as np
import pytest

from reflex_quad import N_JOINTS


def test_ik_fk_round_trip(geom):
    for h in (0.13, 0.18, 0.22):
        for fwd in (-0.04, 0.0, 0.05):
            q0, q1 = geom.ik(h, fwd)
            assert geom.hip_limit[0] < q0 < geom.hip_limit[1], (
                f"({h}, {fwd}) needs hip {q0:.3f} rad, outside the joint limit: "
                "the round trip cannot hold and the stride is being clipped"
            )
            x, z = geom.foot_offset(q0, q1)
            assert z == pytest.approx(-h, abs=1e-6)
            assert x == pytest.approx(fwd, abs=1e-6)


def test_extension_matches_negative_foot_z(geom):
    q0, q1 = geom.ik(0.17, 0.02)
    assert geom.leg_extension(q0, q1) == pytest.approx(0.17, abs=1e-9)


def test_extension_jacobian_matches_finite_difference(geom):
    q0, q1, eps = -0.6, 1.3, 1e-6
    j0, j1 = geom.extension_jacobian(q0, q1)
    n0 = (geom.leg_extension(q0 + eps, q1) - geom.leg_extension(q0 - eps, q1)) / (2 * eps)
    n1 = (geom.leg_extension(q0, q1 + eps) - geom.leg_extension(q0, q1 - eps)) / (2 * eps)
    assert j0 == pytest.approx(n0, abs=1e-6)
    assert j1 == pytest.approx(n1, abs=1e-6)


def test_foot_offset_jacobian_matches_finite_difference(geom):
    q0, q1, eps = -0.6, 1.3, 1e-6
    j0, j1 = geom.foot_offset_jacobian(q0, q1)
    fx = lambda a, b: geom.foot_offset(a, b)[0]  # noqa: E731
    n0 = (fx(q0 + eps, q1) - fx(q0 - eps, q1)) / (2 * eps)
    n1 = (fx(q0, q1 + eps) - fx(q0, q1 - eps)) / (2 * eps)
    assert j0 == pytest.approx(n0, abs=1e-6)
    assert j1 == pytest.approx(n1, abs=1e-6)


def test_unreachable_height_is_clipped_not_nan(geom):
    q0, q1 = geom.ik(0.9)          # beyond a 0.24 m reach
    assert np.isfinite([q0, q1]).all()


def test_stance_command_shape_and_limits(geom):
    u = geom.stance_command(np.full(4, 0.18))
    assert u.shape == (N_JOINTS,)
    assert np.all(u[1::2] >= geom.knee_limit[0])
    # symmetric leg: hip is minus half the knee
    assert u[0] == pytest.approx(-u[1] / 2, abs=1e-6)
