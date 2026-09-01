"""The one rule: memo.txt section 13.

If these fail, the experiment is measuring something other than what it claims.
"""
import inspect
from dataclasses import fields

import pytest

from eval.isolation import check_isolation
from reflex_quad import controller, dither, state_machine
from reflex_quad.types import GroundTruth, Observation

ALLOWED = {
    "t", "body_accel", "body_gyro", "foot_accel", "foot_gyro", "foot_force",
    "servo_current", "battery_voltage", "command_history",
}


def test_observation_carries_only_the_allowed_sensor_set():
    assert {f.name for f in fields(Observation)} == ALLOWED


def test_ground_truth_is_a_separate_type():
    assert not set(f.name for f in fields(GroundTruth)) & (ALLOWED - {"t"})


@pytest.mark.parametrize("cls", [
    controller.HoldController, controller.PostureController,
    dither.DitherController, state_machine.LegCycleController,
])
def test_controllers_only_accept_an_observation(cls):
    sig = inspect.signature(cls.step)
    params = [p for p in sig.parameters if p != "self"]
    assert params == ["obs"], f"{cls.__name__}.step takes {params}"
    hints = inspect.get_annotations(cls.step, eval_str=False)
    assert hints.get("obs") in ("Observation", Observation)


def test_no_controller_side_module_names_joint_truth():
    violations = check_isolation()
    assert violations == [], "\n".join(violations)


def test_runner_refuses_a_controller_holding_the_backend():
    from reflex_quad.runner import _assert_isolated

    class Sneaky:
        pass

    backend = object()
    sneaky = Sneaky()
    sneaky.backend = backend
    with pytest.raises(AssertionError):
        _assert_isolated(sneaky, backend, object())
