"""Controllers.  memo.txt sections 14, 26, 50.

HARD RULE (section 13): nothing in this file may read q, qdot or tau.  The only
input is `Observation`.  `tests/test_observation_isolation.py` greps this file
for the forbidden names and inspects the call signature, so a violation fails
the test suite rather than quietly producing a good-looking result.
"""
from __future__ import annotations

import numpy as np

from . import N_LEGS
from .objective import Objective
from .observer import AttitudeObserver
from .robot import LegGeometry
from .types import ControlOutput, Observation

# leg index -> (sign_x forward=+1, sign_y left=+1, sign_twist FL/RR=+1)
# The twist column matters: a single raised foot loads the FL/RR diagonal and
# unloads the FR/RL one *without tilting the body*, so roll and pitch feedback
# alone cannot see it, let alone correct it.  Four feet on a rigid body are
# statically indeterminate and the twist mode is the null space.
LEG_SIGNS = np.array(
    [[+1.0, +1.0, +1.0], [+1.0, -1.0, -1.0], [-1.0, +1.0, -1.0], [-1.0, -1.0, +1.0]]
)


class BaseController:
    """Common plumbing: attitude observer, objective, command slew limit."""

    mode = "base"

    def __init__(self, cfg: dict, geom: LegGeometry):
        self.cfg = cfg
        self.geom = geom
        self.control_dt = float(cfg["control_dt"])
        self.observer = AttitudeObserver(cfg["sensors"]["observer"], self.control_dt)
        self.objective = Objective(cfg["objective"])
        self.gains = cfg["controller"].get("gains", {})
        self.rate_limit = float(self.gains.get("cmd_rate_limit", 1e9))
        rb = cfg["robot"]
        total_mass = float(rb["body"]["mass"]) + 4 * (
            float(rb["leg"]["upper_mass"]) + float(rb["leg"]["lower_mass"])
        )
        self.weight_N = 9.81 * total_mass
        self.force_target: np.ndarray | None = None   # F*, section "足を一本上げる"
        self.u = geom.nominal_command()
        self._u_prev = self.u.copy()
        self.terms = None

    # -- helpers -----------------------------------------------------------
    def _observe(self, obs: Observation):
        self.observer.update(obs.body_accel, obs.body_gyro)
        roll, pitch, roll_rate, pitch_rate = self.observer.state
        self.terms = self.objective(
            roll, pitch, obs.foot_force, obs.servo_current, self.force_target
        )
        return roll, pitch, roll_rate, pitch_rate

    def _emit(self, u: np.ndarray, state: str, extras: dict | None = None) -> ControlOutput:
        u = self.geom.clip_command(u)
        max_step = self.rate_limit * self.control_dt
        u = np.clip(u, self._u_prev - max_step, self._u_prev + max_step)
        self._u_prev = u.copy()
        self.u = u
        ex = dict(self.terms.as_dict()) if self.terms else {}
        roll, pitch, rr, pr = self.observer.state
        ex.update(
            body_roll_est=roll, body_pitch_est=pitch,
            roll_rate_est=rr, pitch_rate_est=pr,
        )
        if extras:
            ex.update(extras)
        return ControlOutput(u=u, state=state, extras=ex)

    def step(self, obs: Observation) -> ControlOutput:  # pragma: no cover - interface
        raise NotImplementedError


class HoldController(BaseController):
    """Experiment 01: hold the nominal stance, no feedback at all (section 24)."""

    mode = "hold"

    def step(self, obs: Observation) -> ControlOutput:
        self._observe(obs)
        return self._emit(self.geom.nominal_command(), "STAND")


class PostureController(BaseController):
    """Experiment 02: attitude + load-balance feedback.  memo.txt section 26.

    Per-leg vertical extension command:

        e_roll  = (F_FL + F_RL) - (F_FR + F_RR)      left  - right
        e_pitch = (F_FL + F_FR) - (F_RL + F_RR)      front - rear
        e_twist = (F_FL + F_RR) - (F_FR + F_RL)      diagonal (not in memo.txt)

        d_ext_i = -(k_roll  roll  + k_wr roll_rate  + k_fr e_roll ) * sign_y_i
                  +(k_pitch pitch + k_wp pitch_rate) * sign_x_i
                  -(k_fp e_pitch) * sign_x_i
                  + k_h (W - sum F)

    Extending a leg pushes that corner of the body up, hence the signs.
    """

    mode = "posture"

    def __init__(self, cfg: dict, geom: LegGeometry):
        super().__init__(cfg, geom)
        g = self.gains
        self.k_roll = float(g["k_roll"])
        self.k_roll_rate = float(g["k_roll_rate"])
        self.k_force_roll = float(g["k_force_roll"])
        self.k_pitch = float(g["k_pitch"])
        self.k_pitch_rate = float(g["k_pitch_rate"])
        self.k_force_pitch = float(g["k_force_pitch"])
        self.height_gain = float(g["height_gain"])
        # memo_full.txt: dividing by the total load makes the balance signal
        # immune to per-FSR gain error.  Rescaled by weight so the gains keep
        # their units.
        self.normalized_force_error = bool(g.get("normalized_force_error", True))
        self.k_force_twist = float(g.get("k_force_twist", 0.0))
        self.e_twist = 0.0
        self.forward = np.zeros(N_LEGS)
        self.height_bias = np.zeros(N_LEGS)     # used by the state machine
        # Legs the posture loop is allowed to move.  A leg being unloaded or
        # lifted must be excluded: otherwise the tilt caused by unloading it is
        # read as a posture error and "corrected" by extending that same leg,
        # which puts the load straight back on.
        self.posture_mask = np.ones(N_LEGS)
        self.d_limit = float(self.gains.get("posture_limit", 0.030))

    def leg_heights(self, obs: Observation) -> tuple[np.ndarray, float, float]:
        roll, pitch, roll_rate, pitch_rate = self._observe(obs)
        f = np.asarray(obs.foot_force, dtype=float)
        ref = f if self.force_target is None else f - np.asarray(self.force_target, float)
        # A leg the posture loop is not driving must not contribute a load error
        # either: its residual would otherwise be blamed on -- and "corrected"
        # with -- the legs that are still supporting.
        ref = ref * self.posture_mask
        e_roll = (ref[0] + ref[2]) - (ref[1] + ref[3])
        e_pitch = (ref[0] + ref[1]) - (ref[2] + ref[3])
        e_twist = (ref[0] + ref[3]) - (ref[1] + ref[2])
        if self.normalized_force_error:
            s_total = max(float(f.sum()), 0.25 * self.weight_N)
            scale = self.weight_N / s_total
            e_roll, e_pitch, e_twist = e_roll * scale, e_pitch * scale, e_twist * scale

        sx, sy, st = LEG_SIGNS[:, 0], LEG_SIGNS[:, 1], LEG_SIGNS[:, 2]
        d = np.zeros(N_LEGS)
        d += -(self.k_roll * roll + self.k_roll_rate * roll_rate) * sy
        d += -(self.k_force_roll * e_roll) * sy
        d += (self.k_pitch * pitch + self.k_pitch_rate * pitch_rate) * sx
        d += -(self.k_force_pitch * e_pitch) * sx
        d += -(self.k_force_twist * e_twist) * st
        d += self.height_gain * (self.weight_N - float(f.sum()))

        d = np.clip(d, -self.d_limit, self.d_limit) * self.posture_mask
        heights = np.clip(
            self.geom.nominal_height + d + self.height_bias,
            self.geom.min_height,
            self.geom.max_height,
        )
        self.e_twist = e_twist
        return heights, e_roll, e_pitch

    def step(self, obs: Observation) -> ControlOutput:
        heights, e_roll, e_pitch = self.leg_heights(obs)
        u = self.geom.stance_command(heights, self.forward)
        return self._emit(
            u, "STAND", {"e_roll": e_roll, "e_pitch": e_pitch, "e_twist": self.e_twist}
        )
