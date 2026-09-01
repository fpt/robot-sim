"""One-leg unload / lift / verify / lower / reload cycle.

memo.txt sections 32, 33, 34, 39.  The whole point of section 33 is that the
transition out of LIFT is *not* "I commanded a lift, so the leg is up".  It is

    foot IMU shows motion  AND  foot force below the contact threshold

Both come from sensors on the foot, never from the joint.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from . import N_LEGS
from .controller import PostureController
from .robot import LegGeometry
from .support import target_forces
from .types import ControlOutput, Observation

STATES = (
    "STAND",
    "WEIGHT_SHIFT",
    "UNLOAD_LEG",
    "VERIFY_UNLOAD",
    "LIFT",
    "VERIFY_MOTION",
    "LOWER",
    "CONTACT_SEARCH",
    "LOAD",
    "DONE",
    "ABORT",
)
STATE_INDEX = {s: i for i, s in enumerate(STATES)}

# safety: if a state does not finish in this long, abort instead of hanging
STATE_TIMEOUT = {
    "WEIGHT_SHIFT": 4.0,
    "UNLOAD_LEG": 6.0,
    "VERIFY_UNLOAD": 1.5,
    "LIFT": 3.0,
    "VERIFY_MOTION": 2.0,
    "LOWER": 3.0,
    "CONTACT_SEARCH": 4.0,
    "LOAD": 4.0,
}


class LegCycleController(PostureController):
    """Posture control on three legs + a scripted cycle on the fourth."""

    mode = "statemachine"

    def __init__(self, cfg: dict, geom: LegGeometry):
        super().__init__(cfg, geom)
        sm = cfg.get("state_machine", {})
        # `legs` runs the cycle on each leg in turn -- the section 38 self-check
        # and the section 40 crawl order (FL, RR, FR, RL) both use it.
        self.legs = [int(x) for x in sm.get("legs", [sm.get("leg", 0)])]
        self._leg_i = 0
        self.leg = self.legs[0]
        self.unload_rate = float(sm.get("unload_rate", 2.0)) * 0.01   # m/s of retraction
        self.unload_threshold = float(sm.get("unload_threshold", 0.30))
        self.lift_height = float(sm.get("lift_height", 0.030))
        self.motion_threshold = float(sm.get("motion_threshold", 1.2))
        self.motion_window = float(sm.get("motion_window", 0.20))
        self.contact_threshold = float(sm.get("contact_threshold", 0.50))
        self.forward_target = float(sm.get("forward", 0.0))
        self.start_delay = float(sm.get("start_delay", 2.0))
        self.rest_time = float(sm.get("rest_time", 1.5))
        # memo.txt section 32 says "move the load to the other three legs" but not
        # how.  With the CoM at the geometric centre it sits exactly ON the line
        # between the two adjacent feet, so the three-leg support is neutrally
        # stable and the leg cannot be unloaded at all -- the body simply rotates
        # about that line until the foot touches down again.  These legs have no
        # lateral freedom, so the only available weight shift is to move the
        # OTHER foot at the same end (front/rear) further out along x, which
        # rotates the support line clear of the CoM.  See docs/FINDINGS.md.
        self.shift = float(sm.get("weight_shift", 0.050))

        n = max(2, int(self.motion_window / self.control_dt))
        self._gyro_hist: deque[np.ndarray] = deque(maxlen=n)
        self.state = "STAND"
        self._t_state = 0.0
        self._t = 0.0
        self.motion_metric = 0.0
        self.motion_verified = False
        self._gyro_ref: np.ndarray | None = None
        self._foot_angle_est = np.zeros(3)
        self.cycle_count = 0
        self.aborted_reason = ""

    # ------------------------------------------------------------------
    def _motion_metric(self, obs: Observation) -> float:
        """How far the target foot has *rotated*, in radians, from its own IMU.

        Three candidates were tried on the mock backend (docs/FINDINGS.md):

          RMS of foot acceleration   -- wrong sign.  A foot standing in contact
              rattles harder (~4 m/s^2) than a foot hanging in the air (~1.3),
              so a "motion detected" threshold on it fires while planted.
          double-integrated accel    -- drifts.  A 29 mm lift read as 405 mm,
              because the resting reference is not valid once the foot is free.
          integrated gyro            -- works.  A 30 mm lift rotates hip+knee by
              ~0.37 rad, the bias over a 2 s window is ~0.01 rad, so the margin
              is more than an order of magnitude.

        memo_full.txt says the same thing in words: use an IMU for attitude,
        angular rate, and start/stop of motion -- not for position.
        """
        w = np.asarray(obs.foot_gyro[self.leg], dtype=float)
        self._gyro_hist.append(w)
        if self._gyro_ref is None:
            return 0.0
        self._foot_angle_est = self._foot_angle_est + (w - self._gyro_ref) * self.control_dt
        return float(np.linalg.norm(self._foot_angle_est))

    def _arm_motion_estimate(self) -> None:
        """Freeze the resting gyro bias and zero the integrator."""
        if len(self._gyro_hist) >= 2:
            self._gyro_ref = np.mean(np.stack(self._gyro_hist), axis=0)
        else:
            self._gyro_ref = np.zeros(3)
        self._foot_angle_est = np.zeros(3)

    def _update_force_target(self) -> None:
        """F*: hand the target leg's share to the other three before moving it.

        memo_full.txt is explicit that F_FL* must be *ramped* to zero rather than
        stepped -- the load has to be somewhere, and the posture loop needs time
        to put it on the other legs.  What it must be ramped *to* comes from
        support.target_forces(), not from an even split: see support.py.
        """
        fx = self.geom.hip_xy[:, 0] + self.forward
        fy = self.geom.hip_xy[:, 1]
        full = target_forces(fx, fy, self.weight_N, None)
        if self.state in ("WEIGHT_SHIFT", "UNLOAD_LEG", "VERIFY_UNLOAD", "LIFT",
                          "VERIFY_MOTION", "LOWER", "CONTACT_SEARCH"):
            lifted = target_forces(fx, fy, self.weight_N, self.leg)
            # ramp over the weight shift so the posture loop can follow it
            k = 1.0 if self.state != "WEIGHT_SHIFT" else min(1.0, self._t_state / 1.0)
            self.force_target = (1 - k) * full + k * lifted
        elif self.state == "LOAD":
            lifted = target_forces(fx, fy, self.weight_N, self.leg)
            k = min(1.0, self._t_state / 1.5)
            self.force_target = (1 - k) * lifted + k * full
        else:
            self.force_target = full

    def _support_shift(self) -> tuple[int, float]:
        """(leg to move, direction) that frees `self.leg`.

        Legs are ordered FL, FR, RL, RR.  The partner is the other leg at the
        same end; front feet move forward (+x), rear feet backward (-x).
        """
        partner = {0: 1, 1: 0, 2: 3, 3: 2}[self.leg]
        direction = +1.0 if self.leg in (0, 1) else -1.0
        return partner, direction

    def _goto(self, state: str) -> None:
        self.state = state
        self._t_state = 0.0

    def step(self, obs: Observation) -> ControlOutput:
        self._update_force_target()
        self.posture_mask = np.ones(N_LEGS)
        if self.state in ("WEIGHT_SHIFT", "UNLOAD_LEG", "VERIFY_UNLOAD", "LIFT",
                          "VERIFY_MOTION", "LOWER", "CONTACT_SEARCH", "LOAD"):
            self.posture_mask[self.leg] = 0.0
        heights, e_roll, e_pitch = self.leg_heights(obs)
        self._t += self.control_dt
        self._t_state += self.control_dt
        f_leg = float(obs.foot_force[self.leg])
        self.motion_metric = self._motion_metric(obs)
        bias = self.height_bias[self.leg]
        bias_before = self.height_bias.copy()
        d = self.unload_rate * self.control_dt

        if self.state == "STAND":
            if self._t > self.start_delay and self.cycle_count < len(self.legs):
                self.leg = self.legs[self._leg_i]
                self._gyro_hist.clear()
                self._goto("WEIGHT_SHIFT")

        elif self.state == "WEIGHT_SHIFT":
            partner, direction = self._support_shift()
            gap = direction * self.shift - self.forward[partner]
            self.forward[partner] += float(np.clip(gap, -d, d))
            if abs(gap) < 1e-4:
                self._goto("UNLOAD_LEG")

        elif self.state == "UNLOAD_LEG":
            # retract the leg until the FSR says it carries almost nothing
            self.height_bias[self.leg] = bias - d
            if f_leg < self.unload_threshold:
                self._goto("VERIFY_UNLOAD")

        elif self.state == "VERIFY_UNLOAD":
            # section 32: stay unloaded for a moment before trusting it
            if f_leg >= 2.0 * self.unload_threshold:   # hysteresis, or it chatters
                self._goto("UNLOAD_LEG")
            elif self._t_state > 0.3:
                self._arm_motion_estimate()      # reference taken while still
                self._goto("LIFT")

        elif self.state == "LIFT":
            self.height_bias[self.leg] = max(bias - d, -self.lift_height)
            gap = self.forward_target - self.forward[self.leg]
            self.forward[self.leg] += float(np.clip(gap, -d, d))
            if self.height_bias[self.leg] <= -self.lift_height + 1e-6:
                self._goto("VERIFY_MOTION")   # keep integrating, do not re-arm

        elif self.state == "VERIFY_MOTION":
            # section 33: LEG_FREE needs both conditions, not the servo command
            if self.motion_metric > self.motion_threshold and f_leg < self.contact_threshold:
                self.motion_verified = True
                self._goto("LOWER")

        elif self.state == "LOWER":
            self._gyro_ref = None             # stop integrating; drift is bounded
            self.height_bias[self.leg] = bias + d
            if self.height_bias[self.leg] >= -0.005:
                self._goto("CONTACT_SEARCH")

        elif self.state == "CONTACT_SEARCH":
            self.height_bias[self.leg] = bias + 0.3 * d      # slow feel for the ground
            if f_leg > self.contact_threshold:
                self._goto("LOAD")

        elif self.state == "LOAD":
            self.height_bias[self.leg] = min(bias + 0.3 * d, 0.02)
            if f_leg > 0.6 * self.weight_N / N_LEGS or self._t_state > 2.0:
                self.cycle_count += 1
                self._leg_i += 1
                self._goto("DONE")

        elif self.state == "DONE":
            self.height_bias[self.leg] *= 0.995
            partner, _ = self._support_shift()
            self.forward[partner] *= 0.98        # give the support polygon back
            if self._leg_i < len(self.legs) and self._t_state > self.rest_time:
                self._goto("STAND")

        timeout = STATE_TIMEOUT.get(self.state)
        if timeout is not None and self._t_state > timeout:
            self.aborted_reason = f"timeout in {self.state}"
            self._goto("ABORT")

        if self.state == "ABORT":
            self.height_bias[self.leg] *= 0.99
            self.forward[self.leg] *= 0.99

        # leg_heights() already folded height_bias in; recompute so that the
        # bias edited above takes effect on this same tick.
        heights = np.clip(
            heights - bias_before + self.height_bias,
            self.geom.min_height,
            self.geom.max_height,
        )
        u = self.geom.stance_command(heights, self.forward)
        extras = {
            "e_roll": e_roll,
            "e_pitch": e_pitch,
            "e_twist": self.e_twist,
            "sm_state_index": float(STATE_INDEX[self.state]),
            "sm_target_leg": float(self.leg),
            "sm_target_leg_force": f_leg,
            "sm_motion_metric": self.motion_metric,
            "sm_motion_verified": float(self.motion_verified),
            "sm_height_bias": float(self.height_bias[self.leg]),
            "sm_forward": float(self.forward[self.leg]),
            "sm_cycles": float(self.cycle_count),
            "sm_F_target": float(self.force_target[self.leg]),
        }
        return self._emit(u, self.state, extras)
