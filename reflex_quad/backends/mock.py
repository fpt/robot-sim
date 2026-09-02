"""A dependency-free reduced-order stand-in for Isaac Sim.

Why it exists: everything except the physics -- servo model, sensor models,
observer, objective, dither search, state machine, fault monitor, logging and
the whole evaluation pipeline -- can be written, run and regression-tested on a
laptop.  When the CUDA machine is ready, only the backend changes.

What it models (see docs/MOCK_BACKEND.md):
    * full 6-DOF body: x, y, z, roll, pitch, yaw and their rates
    * 8 joints as second-order systems driven by servo torque and contact load
    * each foot as a unilateral spring-damper (vertical) plus a viscous,
      Coulomb-capped friction force (horizontal) against a height field
    * IMU specific force by numerical differentiation, as the real IMU does

Layer 2 of docs/reflex_quad_12dof_trot_plan.md ("mock 拡張"): before this,
mock modelled only heave/roll/pitch, small-angle, and could not represent
horizontal motion, yaw, or a support-diagonal inverted-pendulum tip at all --
the 12DOF morphology's whole premise (yaw-driven propulsion, roll recovering
a lateral tip) needs those to even be checkable on a laptop.  The 12DOF
joints themselves (roll pivots, yaw joints) are NOT added here -- JOINT_NAMES
and config/robot.yaml stay at the current 8; this is the body-dynamics
prerequisite the plan calls out before touching the joint layout, so an
8-joint robot with a 2-point diagonal stance can already show the tip and
recovery the plan needs measured (theta_max) before anything else changes.

What it does NOT model, and what therefore needs Isaac to answer:
    * true stick-slip friction (this is viscous + a Coulomb cap, which has no
      static "stuck" regime and likely overestimates slip relative to a real
      foot pad -- the next candidate refinement if a trot experiment's foot
      slip looks implausible; see the `mu`/tangential parameters below)
    * link inertia coupling, self-collision, foot geometry
    * Euler-rate != body-rate for compound rotations (this backend still
      integrates roll_rate/pitch_rate/yaw_rate directly into roll/pitch/yaw,
      exact only for a single-axis rotation -- adequate for the tilts this
      project's recovery envelopes describe, degrades entering true tumbling)
So: a green run here means the control logic and the plumbing are sound.  It
does not mean the robot stands up in Isaac Sim, and the friction numbers
specifically are not to be trusted -- see the plan's own "simulation
confidence rules": geometry and structure yes, contact absolute values no.
"""
from __future__ import annotations

import numpy as np

from .. import N_JOINTS, N_LEGS
from ..robot import LegGeometry
from .base import SimState

GRAVITY = 9.81


class Terrain:
    """Flat ground plus axis-aligned raised blocks (memo.txt section 25)."""

    def __init__(self, spec: dict):
        self.type = spec.get("type", "flat")
        self.blocks = spec.get("blocks", []) or []

    def height(self, x: float, y: float) -> float:
        h = 0.0
        for b in self.blocks:
            if (
                abs(x - float(b["x"])) <= float(b["size_x"]) / 2
                and abs(y - float(b["y"])) <= float(b["size_y"]) / 2
            ):
                h = max(h, float(b["height"]))
        return h


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body -> world, ZYX (yaw-pitch-roll) Tait-Bryan, matching the
    convention IsaacLabBackend._quat_to_rpy extracts (roll=atan2(...),
    pitch=asin(...), yaw=atan2(...)) -- kept consistent across backends on
    purpose, since a lot of this project's value is mock/Isaac agreement.
    Proper trig, not the small-angle linearisation the pre-Layer-2 version
    used: yaw's own working range (+-30 deg planned) and this project's own
    recoverable-tilt envelopes are well outside where sin(x) ~= x holds.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


class MockBackend:
    def __init__(self, cfg: dict, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.dt = float(cfg["physics_dt"])
        mock = cfg.get("mock", {})
        self.substeps = int(mock.get("substeps", 4))
        self.h = self.dt / self.substeps

        rb = cfg["robot"]
        self.geom = LegGeometry(rb)
        self.leg_mass = float(rb["leg"]["upper_mass"]) + float(rb["leg"]["lower_mass"])
        self.mass = float(rb["body"]["mass"]) + N_LEGS * self.leg_mass
        hx, hy = self.geom.hip_xy[:, 0], self.geom.hip_xy[:, 1]
        self.hx, self.hy = hx, hy
        # Diagonal body-frame inertia approximation, legs treated as point
        # masses at their hip (unchanged from the pre-Layer-2 model for
        # ixx/iyy; izz is new -- same point-mass-at-hip approximation,
        # parallel-axis about the vertical body axis).
        self.ixx = float(rb["body"]["inertia_xx"]) + float(np.sum(self.leg_mass * hy**2))
        self.iyy = float(rb["body"]["inertia_yy"]) + float(np.sum(self.leg_mass * hx**2))
        self.izz = float(rb["body"]["inertia_zz"]) + float(
            np.sum(self.leg_mass * (hx**2 + hy**2))
        )

        self.k_contact = float(mock.get("contact_stiffness", 4000.0))
        self.c_contact = float(mock.get("contact_damping", 120.0))
        # Tangential (friction) spring/damper, viscous + Coulomb-capped -- see
        # the module docstring for what this does not capture (no static
        # stick regime).  Defaults match the vertical contact's own
        # stiffness/damping: no measurement backs a different choice yet.
        self.k_t = float(mock.get("tangential_stiffness", self.k_contact))
        self.c_t = float(mock.get("tangential_damping", self.c_contact))
        # Ground friction coefficient.  Not measured anywhere in this project
        # yet (docs/reflex_quad_sts3215_isaac_eval_plan.md section 5.3 and
        # the 12DOF plan's sensitivity-sweep section both flag friction as
        # the least-trusted number here) -- 0.7 is a generic rubber-pad
        # placeholder, not a measurement.
        self.mu = float(cfg["terrain"].get("friction", 0.7))

        self.joint_inertia = float(cfg["servo"]["joint"]["rotor_inertia"])
        self.joint_damping = float(cfg["servo"]["joint"]["damping"])
        self.terrain = Terrain(cfg["terrain"])

        self.t = 0.0
        self.q = np.zeros(N_JOINTS)
        self.qd = np.zeros(N_JOINTS)
        self.pos = np.zeros(3)          # body position, world frame
        self.vel = np.zeros(3)          # body velocity, world frame
        self.rpy = np.zeros(3)          # roll, pitch, yaw
        self.omega = np.zeros(3)        # body-frame angular rate (see docstring)
        self._foot_vel = np.zeros((N_LEGS, 3))
        self._foot_acc = np.zeros((N_LEGS, 3))
        self._body_acc = np.zeros(3)    # world-frame linear acceleration
        self.contact_force = np.zeros((N_LEGS, 3))
        self._foot_touchdown_xy = np.zeros((N_LEGS, 2))
        self._foot_in_contact = np.zeros(N_LEGS, dtype=bool)
        self._foot_slip = np.zeros(N_LEGS)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> SimState:
        u0 = self.geom.nominal_command()
        self.q = u0.copy()
        self.qd = np.zeros(N_JOINTS)
        self.rpy[:] = 0.0
        self.omega[:] = 0.0
        self.pos[:2] = 0.0
        # start with the feet exactly on the highest ground under them
        ext = np.array([self._extension(i)[0] for i in range(N_LEGS)])
        ground = np.array(
            [self.terrain.height(self.hx[i], self.hy[i]) for i in range(N_LEGS)]
        )
        self.pos[2] = float(np.max(ground + ext)) + 0.0005
        self.vel[:] = 0.0
        self._foot_vel[:] = 0.0
        self._foot_acc[:] = 0.0
        self._foot_in_contact[:] = ground + ext >= self.pos[2] - 1e-9
        for i in range(N_LEGS):
            self._foot_touchdown_xy[i] = [self.hx[i], self.hy[i]]
        self._foot_slip[:] = 0.0
        self.t = 0.0
        return self.state()

    # ------------------------------------------------------------------
    def _extension(self, i: int):
        q0, q1 = self.q[2 * i], self.q[2 * i + 1]
        ext = self.geom.leg_extension(q0, q1)
        j0, j1 = self.geom.extension_jacobian(q0, q1)
        fx, _ = self.geom.foot_offset(q0, q1)
        fx_j0, fx_j1 = self.geom.foot_offset_jacobian(q0, q1)
        return ext, j0, j1, fx, fx_j0, fx_j1

    def _foot_kinematics(self):
        """World-frame foot position and velocity, and the leg's own
        (extension, hip-frame) Jacobian, for all four feet.

        Standard moving-point-on-a-rotating-body kinematics: the foot is a
        point r_body (in the hip/body frame) that itself moves as the leg's
        joints move, attached to a body translating at self.vel and rotating
        at self.omega:
            r_world       = R @ r_body
            foot_pos      = self.pos + r_world
            foot_vel      = self.vel + omega_world x r_world + R @ dr_body/dt
        """
        r = _rotation_matrix(*self.rpy)
        omega_world = r @ self.omega
        foot_pos = np.zeros((N_LEGS, 3))
        foot_vel = np.zeros((N_LEGS, 3))
        jac = np.zeros((N_LEGS, 2))       # d(extension)/dq, for joint torque
        for i in range(N_LEGS):
            ext, j0, j1, fx, fxj0, fxj1 = self._extension(i)
            jac[i] = (j0, j1)
            r_body = np.array([self.hx[i] + fx, self.hy[i], -ext])
            r_world = r @ r_body
            foot_pos[i] = self.pos + r_world
            ext_dot = j0 * self.qd[2 * i] + j1 * self.qd[2 * i + 1]
            fx_dot = fxj0 * self.qd[2 * i] + fxj1 * self.qd[2 * i + 1]
            dr_body = np.array([fx_dot, 0.0, -ext_dot])
            foot_vel[i] = self.vel + np.cross(omega_world, r_world) + r @ dr_body
        return foot_pos, foot_vel, jac

    def _contacts(self):
        """Per-foot contact wrench: vertical spring-damper (normal) plus a
        viscous, Coulomb-capped horizontal force (friction; see the module
        docstring for what this does and does not model)."""
        foot_pos, foot_vel, jac = self._foot_kinematics()
        force = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            pen = self.terrain.height(foot_pos[i, 0], foot_pos[i, 1]) - foot_pos[i, 2]
            if pen <= 0.0:
                continue
            fz = max(0.0, self.k_contact * pen - self.c_contact * foot_vel[i, 2])
            force[i, 2] = fz
            if fz <= 0.0:
                continue
            v_xy = foot_vel[i, :2]
            speed = float(np.linalg.norm(v_xy))
            if speed > 1e-9:
                mag = min(self.mu * fz, self.c_t * speed)
                force[i, :2] = -mag * v_xy / speed
        return force, foot_pos, foot_vel, jac

    def step(self, tau: np.ndarray) -> SimState:
        tau = np.asarray(tau, dtype=float)
        for _ in range(self.substeps):
            self._substep(tau)
        self.t += self.dt
        return self.state()

    def _substep(self, tau: np.ndarray) -> None:
        h = self.h
        force, foot_pos, foot_vel, jac = self._contacts()
        prev_vel = self._foot_vel.copy()
        r = _rotation_matrix(*self.rpy)

        # body translation: Newton's law, world frame
        total_force = np.sum(force, axis=0) + np.array([0.0, 0.0, -self.mass * GRAVITY])
        accel = total_force / self.mass
        self._body_acc = accel
        self.vel += accel * h
        self.pos += self.vel * h

        # body rotation: sum of moments about the body CoM, world frame,
        # rotated into the body frame the (diagonal) inertia is defined in.
        # Ignoring products of inertia and gyroscopic (omega x I omega)
        # coupling -- a standard reduced-order approximation, fine as long
        # as this stays a body with roughly diagonal inertia and moderate
        # spin, which is the regime every experiment here operates in.
        moment_world = np.zeros(3)
        for i in range(N_LEGS):
            r_i = foot_pos[i] - self.pos
            moment_world += np.cross(r_i, force[i])
        moment_body = r.T @ moment_world
        inertia = np.array([self.ixx, self.iyy, self.izz])
        alpha_body = moment_body / inertia
        # Euler-rate = body-rate: exact for a single-axis rotation, an
        # approximation for compound ones -- see the module docstring.
        self.omega += alpha_body * h
        self.rpy += self.omega * h

        # joints: servo torque + reflected contact load (vertical only, as
        # before -- the horizontal/friction force's reflection to the joints
        # is a smaller, second-order effect through the leg's own posture
        # and is not modelled, consistent with mock staying reduced-order)
        tau_load = np.zeros(N_JOINTS)
        for i in range(N_LEGS):
            tau_load[2 * i] = -force[i, 2] * jac[i, 0]
            tau_load[2 * i + 1] = -force[i, 2] * jac[i, 1]
        qdd = (tau + tau_load - self.joint_damping * self.qd) / self.joint_inertia
        self.qd += qdd * h
        self.q += self.qd * h
        self.q[0::2] = np.clip(self.q[0::2], *self.geom.hip_limit)
        self.q[1::2] = np.clip(self.q[1::2], *self.geom.knee_limit)
        hit = (self.q[0::2] <= self.geom.hip_limit[0]) | (self.q[0::2] >= self.geom.hip_limit[1])
        self.qd[0::2] = np.where(hit, 0.0, self.qd[0::2])
        hit = (self.q[1::2] <= self.geom.knee_limit[0]) | (self.q[1::2] >= self.geom.knee_limit[1])
        self.qd[1::2] = np.where(hit, 0.0, self.qd[1::2])

        # foot velocities/accelerations for the foot IMUs, at the
        # post-integration state (same "next-state snapshot" convention the
        # pre-Layer-2 version used for its own foot_vz2/contact_force)
        force2, foot_pos2, foot_vel2, _ = self._contacts()
        self._foot_acc = (foot_vel2 - prev_vel) / h
        self._foot_vel = foot_vel2
        self.contact_force = force2
        self._update_slip(foot_pos2, force2)

    def _update_slip(self, foot_pos: np.ndarray, force: np.ndarray) -> None:
        """Net horizontal displacement of each foot from its own touchdown
        point -- how far it has actually walked away from where it landed,
        not the path length it slid along the way.  Answers "did increasing
        mu reduce slip" directly; see docs/FINDINGS.md and the friction
        model's own caveats in the module docstring (viscous + Coulomb cap,
        not true stick-slip, so this is likely an overestimate)."""
        for i in range(N_LEGS):
            in_contact = force[i, 2] > 0.0
            if in_contact and not self._foot_in_contact[i]:
                self._foot_touchdown_xy[i] = foot_pos[i, :2]
            self._foot_slip[i] = (
                float(np.linalg.norm(foot_pos[i, :2] - self._foot_touchdown_xy[i]))
                if in_contact else 0.0
            )
            self._foot_in_contact[i] = in_contact

    # ------------------------------------------------------------------
    def terrain_height(self, x: float, y: float) -> float:
        return self.terrain.height(x, y)

    def _to_body(self, v_world: np.ndarray) -> np.ndarray:
        return _rotation_matrix(*self.rpy).T @ v_world

    def state(self) -> SimState:
        # specific force = a - g  (an accelerometer at rest reads +1 g up)
        body_sf_world = self._body_acc + np.array([0.0, 0.0, GRAVITY])
        body_sf = self._to_body(body_sf_world)
        foot_sf = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            w = self._foot_acc[i] + np.array([0.0, 0.0, GRAVITY])
            foot_sf[i] = self._to_body(w)
        foot_omega = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            foot_omega[i] = self.omega + [0.0, self.qd[2 * i] + self.qd[2 * i + 1], 0.0]
        foot_pos, _, _ = self._foot_kinematics()
        return SimState(
            t=self.t,
            q=self.q.copy(),
            qd=self.qd.copy(),
            body_pos=self.pos.copy(),
            body_rpy=self.rpy.copy(),
            body_vel=self.vel.copy(),
            body_omega=self.omega.copy(),
            body_accel_body=body_sf,
            foot_pos=foot_pos,
            foot_accel_body=foot_sf,
            foot_omega=foot_omega,
            contact_force=self.contact_force.copy(),
            foot_slip_dist=self._foot_slip.copy(),
        )

    def close(self) -> None:
        pass
