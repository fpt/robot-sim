"""The data contract between the simulator, the controller and the logger.

memo.txt sections 13, 14, 41.  `Observation` is everything the controller is
allowed to see.  `GroundTruth` is everything it is not.  They are separate types
on purpose: a function that takes an `Observation` cannot reach the truth even
by accident.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import N_JOINTS, N_LEGS


@dataclass(frozen=True)
class Observation:
    """The complete sensor set of memo.txt section 14.  No q, no qdot, no tau."""

    t: float
    body_accel: np.ndarray        # (3,) m/s^2, body frame, includes gravity
    body_gyro: np.ndarray         # (3,) rad/s, body frame
    foot_accel: np.ndarray        # (4, 3) m/s^2, foot frame
    foot_gyro: np.ndarray         # (4, 3) rad/s, foot frame
    foot_force: np.ndarray        # (4,) N, FSR reading, clamped to [0, Fmax]
    servo_current: np.ndarray     # (8,) A, measured
    battery_voltage: float        # V
    command_history: np.ndarray   # (H, 8) rad, newest last

    @property
    def last_command(self) -> np.ndarray:
        return self.command_history[-1]

    def validate(self) -> None:
        assert self.body_accel.shape == (3,)
        assert self.body_gyro.shape == (3,)
        assert self.foot_accel.shape == (N_LEGS, 3)
        assert self.foot_gyro.shape == (N_LEGS, 3)
        assert self.foot_force.shape == (N_LEGS,)
        assert self.servo_current.shape == (N_JOINTS,)
        assert self.command_history.ndim == 2 and self.command_history.shape[1] == N_JOINTS


@dataclass(frozen=True)
class GroundTruth:
    """Simulator internals.  Logged for analysis only (memo.txt sections 41, 43)."""

    t: float
    q: np.ndarray                 # (8,) rad
    qd: np.ndarray                # (8,) rad/s
    tau: np.ndarray               # (8,) Nm
    body_pos: np.ndarray          # (3,) m, world
    body_rpy: np.ndarray          # (3,) rad, world
    body_vel: np.ndarray          # (3,) m/s
    body_omega: np.ndarray        # (3,) rad/s
    foot_pos: np.ndarray          # (4, 3) m, world
    contact_force: np.ndarray     # (4, 3) N, world


@dataclass
class ControlOutput:
    """What a controller returns each control tick."""

    u: np.ndarray                             # (8,) rad, servo position command
    state: str = "STAND"                      # state-machine label for the log
    fault_flag: int = 0                       # bitmask, see faults.FaultMonitor
    extras: dict = field(default_factory=dict)  # scalars merged into the CSV
