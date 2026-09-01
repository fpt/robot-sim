"""The evaluation function J.  memo.txt section 23.

    J_pose  = roll^2 + pitch^2
    J_force = sum_i (F_i - F_i*)^2      F* defaults to the mean (section 23)
    J_power = sum_j I_j^2
    J       = w_pose J_pose + w_force J_force + w_power J_power

Built only from things the controller may see: estimated attitude, FSR readings
and measured servo current.  This is deliberate -- J is what the robot itself
minimises online (section 28), so it may not contain ground truth.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ObjectiveTerms:
    J_pose: float
    J_force: float
    J_power: float
    J_total: float

    def as_dict(self) -> dict[str, float]:
        return {
            "J_pose": self.J_pose,
            "J_force": self.J_force,
            "J_power": self.J_power,
            "J_total": self.J_total,
        }


class Objective:
    def __init__(self, cfg: dict):
        self.w_pose = float(cfg["w_pose"])
        self.w_force = float(cfg["w_force"])
        self.w_power = float(cfg["w_power"])

    def __call__(
        self,
        roll: float,
        pitch: float,
        foot_force: np.ndarray,
        servo_current: np.ndarray,
        force_target: np.ndarray | None = None,
    ) -> ObjectiveTerms:
        """`force_target` is F* -- the desired load distribution.  memo_full.txt
        keeps it uniform while standing and ramps one entry to zero before a leg
        is lifted; None means "share the load equally" (section 23)."""
        f = np.asarray(foot_force, dtype=float)
        i = np.asarray(servo_current, dtype=float)
        target = f.mean() if force_target is None else np.asarray(force_target, float)
        j_pose = float(roll**2 + pitch**2)
        j_force = float(np.sum((f - target) ** 2))
        j_power = float(np.sum(i**2))
        total = self.w_pose * j_pose + self.w_force * j_force + self.w_power * j_power
        return ObjectiveTerms(j_pose, j_force, j_power, total)
