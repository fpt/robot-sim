"""The interface every backend implements.

The servo model, the sensor models and the controller live *outside* the
backend, so swapping mock <-> Isaac Lab changes the physics and nothing else.
The backend only has to: hold the articulation, accept joint torques, advance
one physics step, and report truth.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class SimState:
    """Raw simulator truth for one physics tick, in SI units, world frame."""

    t: float
    q: np.ndarray             # (8,)
    qd: np.ndarray            # (8,)
    body_pos: np.ndarray      # (3,)
    body_rpy: np.ndarray      # (3,)
    body_vel: np.ndarray      # (3,)
    body_omega: np.ndarray    # (3,)
    body_accel_body: np.ndarray   # (3,) specific force in the body frame (IMU truth)
    foot_pos: np.ndarray      # (4, 3)
    foot_accel_body: np.ndarray   # (4, 3) specific force at each foot
    foot_omega: np.ndarray    # (4, 3)
    contact_force: np.ndarray  # (4, 3) world


class SimBackend(Protocol):
    dt: float

    def reset(self) -> SimState: ...
    def step(self, tau: np.ndarray) -> SimState: ...
    def state(self) -> SimState: ...
    def terrain_height(self, x: float, y: float) -> float: ...
    def close(self) -> None: ...
