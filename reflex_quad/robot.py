"""Geometry of the 2-link planar leg and the command-space parametrisation.

memo.txt sections 10, 11.  Angle convention:

    hip  q0 : rotation about the body y axis, 0 = upper link straight down,
              positive swings the foot toward +x (forward).
    knee q1 : relative to the upper link, 0 = straight leg, positive bends the
              foot backward (single-sided joint).

Foot position relative to the hip, in the leg's sagittal plane:

    x = L1 sin(q0) + L2 sin(q0 + q1)
    z = -(L1 cos(q0) + L2 cos(q0 + q1))

Everything here is command-space arithmetic.  It is used by the controller to
turn "I want this leg 5 mm longer" into u[8], which is legal: the controller is
allowed to know what it commanded, only not what the joint actually did.
"""
from __future__ import annotations

import numpy as np

from . import N_JOINTS, N_LEGS


class LegGeometry:
    def __init__(self, robot_cfg: dict):
        self.l1 = float(robot_cfg["leg"]["upper_length"])
        self.l2 = float(robot_cfg["leg"]["lower_length"])
        self.hip_limit = tuple(robot_cfg["leg"]["hip_limit"])
        self.knee_limit = tuple(robot_cfg["leg"]["knee_limit"])
        self.hip_xy = np.array(
            [[leg["hip_x"], leg["hip_y"]] for leg in robot_cfg["legs"]], dtype=float
        )
        self.nominal_height = float(robot_cfg["stance"]["height"])
        self.min_height = float(robot_cfg["stance"]["min_height"])
        self.max_height = float(robot_cfg["stance"]["max_height"])
        self.reach = self.l1 + self.l2

    # -- forward kinematics ------------------------------------------------
    def foot_offset(self, q0: float, q1: float) -> tuple[float, float]:
        x = self.l1 * np.sin(q0) + self.l2 * np.sin(q0 + q1)
        z = -(self.l1 * np.cos(q0) + self.l2 * np.cos(q0 + q1))
        return float(x), float(z)

    def leg_extension(self, q0: float, q1: float) -> float:
        """Vertical distance from hip down to foot (positive = leg reaching down)."""
        return float(self.l1 * np.cos(q0) + self.l2 * np.cos(q0 + q1))

    def extension_jacobian(self, q0: float, q1: float) -> tuple[float, float]:
        """d(extension)/dq0, d(extension)/dq1."""
        s0 = np.sin(q0)
        s01 = np.sin(q0 + q1)
        return float(-(self.l1 * s0 + self.l2 * s01)), float(-self.l2 * s01)

    def foot_offset_jacobian(self, q0: float, q1: float) -> tuple[float, float]:
        """d(foot_offset x)/dq0, d(foot_offset x)/dq1.

        Needed by the mock backend's Layer 2 physics (docs/reflex_quad_12dof_trot_plan.md)
        to get an analytic foot horizontal velocity -- v_foot = v_hip + omega x r +
        R @ (fx_dot, 0, -ext_dot) -- rather than a finite difference, for the same
        numerical-stability reason extension_jacobian already exists: a spring-damper
        contact model wants a velocity that is not a step behind the force it damps.
        """
        c0 = np.cos(q0)
        c01 = np.cos(q0 + q1)
        return float(self.l1 * c0 + self.l2 * c01), float(self.l2 * c01)

    # -- inverse kinematics ------------------------------------------------
    def ik(self, height: float, forward: float = 0.0) -> tuple[float, float]:
        """Angles that put the foot `height` below and `forward` ahead of the hip."""
        height = float(np.clip(height, 1e-3, self.reach - 1e-3))
        r = float(np.hypot(forward, height))
        r = min(r, self.reach - 1e-4)
        # knee interior angle from the law of cosines
        cos_knee = (self.l1**2 + self.l2**2 - r**2) / (2 * self.l1 * self.l2)
        knee = np.pi - np.arccos(np.clip(cos_knee, -1.0, 1.0))
        cos_a = (self.l1**2 + r**2 - self.l2**2) / (2 * self.l1 * r)
        alpha = np.arccos(np.clip(cos_a, -1.0, 1.0))
        hip = np.arctan2(forward, height) - alpha
        return (
            float(np.clip(hip, *self.hip_limit)),
            float(np.clip(knee, *self.knee_limit)),
        )

    def stance_command(
        self, heights: np.ndarray, forwards: np.ndarray | None = None
    ) -> np.ndarray:
        """Per-leg (height, forward) -> u[8] in the fixed joint order."""
        heights = np.asarray(heights, dtype=float).reshape(N_LEGS)
        forwards = (
            np.zeros(N_LEGS) if forwards is None else np.asarray(forwards, float).reshape(N_LEGS)
        )
        u = np.zeros(N_JOINTS)
        for i in range(N_LEGS):
            hip, knee = self.ik(heights[i], forwards[i])
            u[2 * i], u[2 * i + 1] = hip, knee
        return u

    def clip_command(self, u: np.ndarray) -> np.ndarray:
        u = np.asarray(u, dtype=float).copy()
        u[0::2] = np.clip(u[0::2], *self.hip_limit)
        u[1::2] = np.clip(u[1::2], *self.knee_limit)
        return u

    def nominal_command(self) -> np.ndarray:
        return self.stance_command(np.full(N_LEGS, self.nominal_height))
