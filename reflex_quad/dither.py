"""Active sensing by dither.  memo.txt sections 27, 28, 29, 30 -- the core idea.

For one joint at a time:

    u_plus  = u + delta            measure J_plus,  I_plus
    u_minus = u - delta            measure J_minus, I_minus

    dJ/du  ~= (J_plus - J_minus) / (2 delta)
    D_I     =  I_plus - I_minus              (section 27, logged, not yet used)

    u <- u - alpha dJ/du

The robot never learns *why* J improved.  It only knows the command it sent and
the response it got back (section 50).
"""
from __future__ import annotations

import numpy as np

from . import N_JOINTS
from .controller import BaseController, PostureController
from .robot import LegGeometry
from .types import ControlOutput, Observation

PHASE_CODE = {"SETTLE": 0.0, "MEASURE": 1.0, "UPDATE": 2.0}


def build_schedule(repeats: int) -> list[tuple[str, int]]:
    """One probe cycle: repeats x (settle, measure) per sign, in ABBA order.

    ABBA (+ - - +) cancels any drift that is linear in time over the cycle, so a
    body that is still slowly settling does not masquerade as a gradient.  This
    is the discrete version of the synchronous detection memo_full.txt asks for
    ("電流波形を同期検波するように見る").
    """
    out: list[tuple[str, int]] = []
    for r in range(max(1, repeats)):
        order = (+1, -1) if r % 2 == 0 else (-1, +1)
        for sign in order:
            out.append(("SETTLE", sign))
            out.append(("MEASURE", sign))
    out.append(("UPDATE", 0))
    return out


class DitherController(BaseController):
    mode = "dither"

    def __init__(self, cfg: dict, geom: LegGeometry):
        super().__init__(cfg, geom)
        d = cfg["dither"]
        self.delta = float(d["delta"])
        self.settle_time = float(d["settle_time"])
        self.measure_time = float(d["measure_time"])
        self.alpha = float(d["alpha"])
        self.update_rule = str(d.get("update_rule", "gradient"))
        self.rprop_step = np.full(N_JOINTS, float(d.get("rprop_step0", 0.004)))
        self.rprop_min = float(d.get("rprop_step_min", 4e-4))
        self.rprop_max = float(d.get("rprop_step_max", 0.02))
        self.rprop_up = float(d.get("rprop_up", 1.2))
        self.rprop_down = float(d.get("rprop_down", 0.5))
        self._prev_sign = np.zeros(N_JOINTS)
        self.step_clip = float(d["step_clip"])
        self.joint_list = [int(j) for j in d["joints"]]
        self.repeats = int(d.get("repeats", 2))
        self.schedule = build_schedule(self.repeats)
        self.use_posture_base = bool(d.get("use_posture_base", False))

        self.offset = np.zeros(N_JOINTS)      # what gradient descent has learned
        self.gradient = np.zeros(N_JOINTS)    # latest g[j] (section 30)
        self._posture = PostureController(cfg, geom) if self.use_posture_base else None

        self._joint_i = 0
        self._phase = 0
        self._phase_t = 0.0
        self._acc_j: list[float] = []
        self._acc_i: list[float] = []
        self._plus_j: list[float] = []
        self._minus_j: list[float] = []
        self._plus_i: list[float] = []
        self._minus_i: list[float] = []
        self.update_count = 0
        self.last_dJ = 0.0
        self.last_dI = 0.0

    # ------------------------------------------------------------------
    @property
    def joint(self) -> int:
        return self.joint_list[self._joint_i]

    def _base_command(self, obs: Observation) -> np.ndarray:
        if self._posture is not None:
            heights, _, _ = self._posture.leg_heights(obs)
            base = self.geom.stance_command(heights)
        else:
            base = self.geom.nominal_command()
        return base + self.offset

    def _advance_phase(self) -> None:
        self._phase = (self._phase + 1) % len(self.schedule)
        self._phase_t = 0.0
        self._acc_j.clear()
        self._acc_i.clear()

    def step(self, obs: Observation) -> ControlOutput:
        self._observe(obs)
        j_now = self.terms.J_total
        joint = self.joint
        base = self._base_command(obs)
        kind, sign = self.schedule[self._phase]

        probe = np.zeros(N_JOINTS)
        probe[joint] = sign * self.delta

        if kind == "MEASURE":
            self._acc_j.append(j_now)
            self._acc_i.append(float(obs.servo_current[joint]))

        self._phase_t += self.control_dt
        limit = {"SETTLE": self.settle_time, "MEASURE": self.measure_time, "UPDATE": 0.0}[kind]
        if self._phase_t >= limit:
            if kind == "MEASURE":
                mj = float(np.mean(self._acc_j)) if self._acc_j else j_now
                mi = float(np.mean(self._acc_i)) if self._acc_i else 0.0
                (self._plus_j if sign > 0 else self._minus_j).append(mj)
                (self._plus_i if sign > 0 else self._minus_i).append(mi)
            elif kind == "UPDATE":
                self._apply_update(joint)
            self._advance_phase()

        u = base + probe
        extras = {
            "dither_joint": float(joint),
            "dither_phase": PHASE_CODE[kind],
            "dither_sign": float(sign),
            "dither_grad": float(self.gradient[joint]),
            "dither_dJ": self.last_dJ,
            "dither_dI": self.last_dI,
            "dither_updates": float(self.update_count),
            "u_offset_norm": float(np.linalg.norm(self.offset)),
            "dither_step": float(self.rprop_step[joint]),
        }
        return self._emit(u, "DITHER", extras)

    def _rprop_step(self, joint: int, g: float) -> float:
        """Sign-only descent with a per-joint adaptive step.

        Deviation from memo.txt section 28, and a deliberate one: `alpha * dJ/du`
        needs the scale of J, which changes with the weights, the terrain and the
        sensor calibration.  A finite-difference probe on a real robot gives a
        trustworthy *sign* and a noisy *magnitude*, so RPROP -- which uses only
        the sign, and adapts its own step -- is the safer estimator.  Set
        `dither.update_rule: gradient` to get the literal section 28 rule back.
        """
        s = float(np.sign(g))
        prev = self._prev_sign[joint]
        if s * prev > 0:
            self.rprop_step[joint] = min(self.rprop_step[joint] * self.rprop_up, self.rprop_max)
        elif s * prev < 0:
            self.rprop_step[joint] = max(self.rprop_step[joint] * self.rprop_down, self.rprop_min)
        self._prev_sign[joint] = s
        return float(-s * self.rprop_step[joint])

    def _apply_update(self, joint: int) -> None:
        """Section 28: finite-difference gradient, then one descent step."""
        # mean over the repeats: zero-mean measurement noise averages out, and
        # the ABBA order has already cancelled linear drift
        self.last_dJ = float(np.mean(self._plus_j) - np.mean(self._minus_j))
        self.last_dI = float(np.mean(self._plus_i) - np.mean(self._minus_i))
        for acc in (self._plus_j, self._minus_j, self._plus_i, self._minus_i):
            acc.clear()
        g = self.last_dJ / (2.0 * self.delta)
        self.gradient[joint] = g
        if self.update_rule == "rprop":
            step = self._rprop_step(joint, g)
        else:
            step = float(np.clip(-self.alpha * g, -self.step_clip, self.step_clip))
        self.offset[joint] += step
        self.offset[joint] = float(np.clip(self.offset[joint], -0.35, 0.35))
        self.update_count += 1
        self._joint_i = (self._joint_i + 1) % len(self.joint_list)
