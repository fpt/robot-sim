"""A dependency-free reduced-order stand-in for Isaac Sim.

Why it exists: everything except the physics -- servo model, sensor models,
observer, objective, dither search, state machine, fault monitor, logging and
the whole evaluation pipeline -- can be written, run and regression-tested on a
laptop.  When the CUDA machine is ready, only the backend changes.

What it models (see docs/MOCK_BACKEND.md):
    * body heave z, roll and pitch, small-angle             (3 DOF)
    * 8 joints as second-order systems driven by servo torque and contact load
    * each foot as a unilateral spring-damper against a height field
    * IMU specific force by numerical differentiation, as the real IMU does

What it does NOT model, and what therefore needs Isaac to answer:
    * horizontal body motion, yaw, friction cones, slipping, tipping over
    * link inertia coupling, self-collision, foot geometry
So: a green run here means the control logic and the plumbing are sound.  It
does not mean the robot stands up in Isaac Sim.
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


class MockBackend:
    def __init__(self, cfg: dict, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.dt = float(cfg["physics_dt"])
        self.substeps = int(cfg.get("mock", {}).get("substeps", 4))
        self.h = self.dt / self.substeps

        rb = cfg["robot"]
        self.geom = LegGeometry(rb)
        self.leg_mass = float(rb["leg"]["upper_mass"]) + float(rb["leg"]["lower_mass"])
        self.mass = float(rb["body"]["mass"]) + N_LEGS * self.leg_mass
        hx, hy = self.geom.hip_xy[:, 0], self.geom.hip_xy[:, 1]
        self.hx, self.hy = hx, hy
        self.ixx = float(rb["body"]["inertia_xx"]) + float(np.sum(self.leg_mass * hy**2))
        self.iyy = float(rb["body"]["inertia_yy"]) + float(np.sum(self.leg_mass * hx**2))

        mock = cfg.get("mock", {})
        self.k_contact = float(mock.get("contact_stiffness", 4000.0))
        self.c_contact = float(mock.get("contact_damping", 120.0))
        self.joint_inertia = float(cfg["servo"]["joint"]["rotor_inertia"])
        self.joint_damping = float(cfg["servo"]["joint"]["damping"])
        self.terrain = Terrain(cfg["terrain"])

        self.t = 0.0
        self.q = np.zeros(N_JOINTS)
        self.qd = np.zeros(N_JOINTS)
        self.z = 0.0
        self.vz = 0.0
        self.roll = self.pitch = 0.0
        self.roll_rate = self.pitch_rate = 0.0
        self._foot_vel = np.zeros((N_LEGS, 3))
        self._foot_acc = np.zeros((N_LEGS, 3))
        self._body_acc_z = 0.0
        self.contact_force = np.zeros((N_LEGS, 3))
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> SimState:
        u0 = self.geom.nominal_command()
        self.q = u0.copy()
        self.qd = np.zeros(N_JOINTS)
        # start with the feet exactly on the highest ground under them
        ext = np.array([self._extension(i)[0] for i in range(N_LEGS)])
        ground = np.array(
            [self.terrain.height(self.hx[i], self.hy[i]) for i in range(N_LEGS)]
        )
        self.z = float(np.max(ground + ext)) + 0.0005
        self.vz = 0.0
        self.roll = self.pitch = self.roll_rate = self.pitch_rate = 0.0
        self._foot_vel[:] = 0.0
        self._foot_acc[:] = 0.0
        self.t = 0.0
        return self.state()

    # ------------------------------------------------------------------
    def _extension(self, i: int):
        q0, q1 = self.q[2 * i], self.q[2 * i + 1]
        ext = self.geom.leg_extension(q0, q1)
        j0, j1 = self.geom.extension_jacobian(q0, q1)
        fx, _ = self.geom.foot_offset(q0, q1)
        return ext, j0, j1, fx

    def _contacts(self):
        """Vertical contact force per foot plus the geometry it came from."""
        f = np.zeros(N_LEGS)
        foot_z = np.zeros(N_LEGS)
        foot_x = np.zeros(N_LEGS)
        foot_vz = np.zeros(N_LEGS)
        jac = np.zeros((N_LEGS, 2))
        for i in range(N_LEGS):
            ext, j0, j1, fx = self._extension(i)
            jac[i] = (j0, j1)
            hip_z = self.z - self.pitch * self.hx[i] + self.roll * self.hy[i]
            hip_vz = self.vz - self.pitch_rate * self.hx[i] + self.roll_rate * self.hy[i]
            ext_dot = j0 * self.qd[2 * i] + j1 * self.qd[2 * i + 1]
            foot_x[i] = self.hx[i] + fx
            foot_z[i] = hip_z - ext
            foot_vz[i] = hip_vz - ext_dot
            pen = self.terrain.height(foot_x[i], self.hy[i]) - foot_z[i]
            if pen > 0.0:
                f[i] = max(0.0, self.k_contact * pen - self.c_contact * foot_vz[i])
        return f, foot_z, foot_x, foot_vz, jac

    def step(self, tau: np.ndarray) -> SimState:
        tau = np.asarray(tau, dtype=float)
        for _ in range(self.substeps):
            self._substep(tau)
        self.t += self.dt
        return self.state()

    def _substep(self, tau: np.ndarray) -> None:
        h = self.h
        f, foot_z, foot_x, foot_vz, jac = self._contacts()
        prev_vel = self._foot_vel.copy()

        # body: heave + two rotations, small angle
        az = float(np.sum(f)) / self.mass - GRAVITY
        ddroll = float(np.sum(f * self.hy)) / self.ixx
        # moment arm is where the foot actually is: the hip joints move the feet
        # in x, and that is the only way this morphology can reshape its support
        # polygon (there is no lateral degree of freedom).
        ddpitch = -float(np.sum(f * foot_x)) / self.iyy
        self._body_acc_z = az
        self.vz += az * h
        self.z += self.vz * h
        self.roll_rate += ddroll * h
        self.roll += self.roll_rate * h
        self.pitch_rate += ddpitch * h
        self.pitch += self.pitch_rate * h

        # joints: servo torque + reflected contact load
        tau_load = np.zeros(N_JOINTS)
        for i in range(N_LEGS):
            tau_load[2 * i] = -f[i] * jac[i, 0]
            tau_load[2 * i + 1] = -f[i] * jac[i, 1]
        qdd = (tau + tau_load - self.joint_damping * self.qd) / self.joint_inertia
        self.qd += qdd * h
        self.q += self.qd * h
        self.q[0::2] = np.clip(self.q[0::2], *self.geom.hip_limit)
        self.q[1::2] = np.clip(self.q[1::2], *self.geom.knee_limit)
        hit = (self.q[0::2] <= self.geom.hip_limit[0]) | (self.q[0::2] >= self.geom.hip_limit[1])
        self.qd[0::2] = np.where(hit, 0.0, self.qd[0::2])
        hit = (self.q[1::2] <= self.geom.knee_limit[0]) | (self.q[1::2] >= self.geom.knee_limit[1])
        self.qd[1::2] = np.where(hit, 0.0, self.qd[1::2])

        # foot velocities/accelerations for the foot IMUs
        f2, foot_z2, foot_x2, foot_vz2, _ = self._contacts()
        vel = np.zeros((N_LEGS, 3))
        vel[:, 0] = (foot_x2 - foot_x) / h
        vel[:, 2] = foot_vz2
        self._foot_acc = (vel - prev_vel) / h
        self._foot_vel = vel
        self.contact_force[:, 2] = f2

    # ------------------------------------------------------------------
    def terrain_height(self, x: float, y: float) -> float:
        return self.terrain.height(x, y)

    def _to_body(self, v_world: np.ndarray) -> np.ndarray:
        """R^T v for small roll/pitch."""
        r, p = self.roll, self.pitch
        rt = np.array([[1.0, 0.0, -p], [0.0, 1.0, r], [p, -r, 1.0]])
        return rt @ v_world

    def state(self) -> SimState:
        # specific force = a - g  (an accelerometer at rest reads +1 g up)
        body_sf_world = np.array([0.0, 0.0, self._body_acc_z + GRAVITY])
        body_sf = self._to_body(body_sf_world)
        foot_sf = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            w = self._foot_acc[i] + np.array([0.0, 0.0, GRAVITY])
            foot_sf[i] = self._to_body(w)
        foot_omega = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            foot_omega[i] = [
                self.roll_rate,
                self.pitch_rate + self.qd[2 * i] + self.qd[2 * i + 1],
                0.0,
            ]
        foot_pos = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            ext, _, _, fx = self._extension(i)
            hip_z = self.z - self.pitch * self.hx[i] + self.roll * self.hy[i]
            foot_pos[i] = [self.hx[i] + fx, self.hy[i], hip_z - ext]
        return SimState(
            t=self.t,
            q=self.q.copy(),
            qd=self.qd.copy(),
            body_pos=np.array([0.0, 0.0, self.z]),
            body_rpy=np.array([self.roll, self.pitch, 0.0]),
            body_vel=np.array([0.0, 0.0, self.vz]),
            body_omega=np.array([self.roll_rate, self.pitch_rate, 0.0]),
            body_accel_body=body_sf,
            foot_pos=foot_pos,
            foot_accel_body=foot_sf,
            foot_omega=foot_omega,
            contact_force=self.contact_force.copy(),
        )

    def close(self) -> None:
        pass
