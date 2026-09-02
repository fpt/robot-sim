"""_assert_sane_initial_state, in isolation.

docs/FINDINGS.md #15: IsaacLabBackend.reset() silently spawned the robot at
the URDF's raw rest pose (straight legs, feet in the ground) instead of
geom.nominal_command(), and it took a full run of tumbling truth.csv to
notice.  This is the guard that is supposed to catch that in under a
millisecond, on any backend -- exercised here directly against a synthetic
SimState so it does not need a run, mock or Isaac, to check.
"""
import numpy as np
import pytest

from reflex_quad.backends.base import SimState
from reflex_quad.config import load_experiment
from reflex_quad.robot import LegGeometry
from reflex_quad.runner import _assert_sane_initial_state


@pytest.fixture
def geom():
    return LegGeometry(load_experiment("01_stand")["robot"])


class _FlatTerrain:
    """A stand-in backend: only terrain_height() is used by the check."""

    def terrain_height(self, x, y):
        return 0.0


def _state_at(geom: LegGeometry, q: np.ndarray, foot_dz: float = 0.0) -> SimState:
    """A SimState with the given joint_pos and feet resting on `_FlatTerrain`
    (z=0) plus foot_dz -- the two checks are independent, so this does not
    need to be leg-kinematics-consistent to exercise either one."""
    n_legs = len(geom.hip_xy)
    foot_pos = np.zeros((n_legs, 3))
    for i, (hx, hy) in enumerate(geom.hip_xy):
        foot_pos[i] = [hx, hy, foot_dz]
    return SimState(
        t=0.0, q=q, qd=np.zeros_like(q),
        body_pos=np.zeros(3), body_rpy=np.zeros(3), body_vel=np.zeros(3),
        body_omega=np.zeros(3), body_accel_body=np.zeros(3),
        foot_pos=foot_pos, foot_accel_body=np.zeros((n_legs, 3)),
        foot_omega=np.zeros((n_legs, 3)), contact_force=np.zeros((n_legs, 3)),
    )


def test_nominal_pose_passes(geom):
    sim = _state_at(geom, geom.nominal_command())
    _assert_sane_initial_state(sim, geom, _FlatTerrain())    # must not raise


def test_wrong_joint_pos_raises(geom):
    """The exact failure mode from #15: reset() left q at the URDF rest pose."""
    sim = _state_at(geom, np.zeros_like(geom.nominal_command()))
    with pytest.raises(AssertionError, match="does not match nominal_command"):
        _assert_sane_initial_state(sim, geom, _FlatTerrain())


def test_foot_penetration_raises(geom):
    sim = _state_at(geom, geom.nominal_command(), foot_dz=-0.015)
    with pytest.raises(AssertionError, match="inside the terrain"):
        _assert_sane_initial_state(sim, geom, _FlatTerrain())


def test_foot_floating_raises(geom):
    sim = _state_at(geom, geom.nominal_command(), foot_dz=0.1)
    with pytest.raises(AssertionError, match="above the terrain"):
        _assert_sane_initial_state(sim, geom, _FlatTerrain())


def test_non_finite_joint_pos_raises(geom):
    q = geom.nominal_command().copy()
    q[0] = np.nan
    sim = _state_at(geom, q)
    with pytest.raises(AssertionError, match="non-finite"):
        _assert_sane_initial_state(sim, geom, _FlatTerrain())
