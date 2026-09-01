"""Cheap-servo model.  memo.txt sections 19, 20, 21, 22.

    tau = Kp (u - q) - Kd qdot

`q` and `qdot` come from the simulator and are visible *inside this module only*.
The only thing that leaves here toward the controller is the measured current.

Phases (section 20) are cumulative:
    A  PD only
    B  + torque and velocity ceilings
    C  + command deadband and transport delay
    D  + coulomb/viscous friction and backlash
    E  + supply-voltage dependent torque
"""
from __future__ import annotations

from collections import deque

import numpy as np

from . import N_JOINTS

PHASE_ORDER = "ABCDE"


class ServoBank:
    def __init__(self, servo_cfg: dict, dt: float, rng: np.random.Generator):
        self.cfg = servo_cfg
        self.dt = float(dt)
        self.rng = rng
        self.phase = str(servo_cfg.get("phase", "A")).upper()
        self._level = PHASE_ORDER.index(self.phase)

        self.kp = np.full(N_JOINTS, float(servo_cfg["pd"]["kp"]))
        self.kd = np.full(N_JOINTS, float(servo_cfg["pd"]["kd"]))
        self.tau_max = np.full(N_JOINTS, float(servo_cfg["limits"]["tau_max"]))
        self.qd_max = np.full(N_JOINTS, float(servo_cfg["limits"]["qd_max"]))
        self.friction_scale = np.ones(N_JOINTS)
        self.tau_scale = np.ones(N_JOINTS)

        nd = servo_cfg["nonideal"]
        self.deadband = float(nd["deadband"])
        base_delay = float(nd["command_delay"])
        self.extra_delay = np.zeros(N_JOINTS)
        self._base_delay = base_delay

        fr = servo_cfg["friction"]
        self.coulomb = float(fr["coulomb"])
        self.viscous = float(fr["viscous"])
        self.backlash = float(fr["backlash"])
        # Regularisation width for the Coulomb term.  tanh(qd/0.02) is a nearly
        # ideal step at 500 Hz: with a 1.5e-3 kg m^2 rotor a 0.2 Nm friction
        # torque reverses the joint velocity inside one timestep, so the joint
        # chatters and "friction x10" shows up as MORE foot motion instead of
        # less.  0.2 rad/s is smooth at this timestep and still saturates well
        # below the 8 rad/s free-running speed.
        self.stick_velocity = float(fr.get("stick_velocity", 0.2))

        cur = servo_cfg["current"]
        self.i_idle = float(cur["i_idle"])
        self.k_tau = float(cur["k_tau"])
        self.i_noise = float(cur["noise_std"])
        self.i_quant = float(cur["quantization"])
        self.i_bias = rng.normal(0.0, float(cur["bias_std"]), N_JOINTS)

        sup = servo_cfg["supply"]
        self.v_nominal = float(sup["nominal_voltage"])
        self.v_sag = float(sup["voltage_sag_per_amp"])
        self.torque_per_volt = float(sup["torque_per_volt"])

        # transport delay buffer, sized for the largest delay we may ever need
        self._max_buf = max(1, int(round((base_delay + 0.5) / self.dt)) + 2)
        self._buf: deque[np.ndarray] = deque(maxlen=self._max_buf)
        self._lash = np.zeros(N_JOINTS)   # backlash internal state
        self.voltage = self.v_nominal
        self.tau_cmd = np.zeros(N_JOINTS)
        self.current_true = np.zeros(N_JOINTS)

    # -- phase gates -------------------------------------------------------
    def _has(self, phase: str) -> bool:
        return self._level >= PHASE_ORDER.index(phase)

    # -- command path ------------------------------------------------------
    def _delayed_command(self, u: np.ndarray) -> np.ndarray:
        self._buf.append(np.asarray(u, dtype=float).copy())
        if not self._has("C"):
            return self._buf[-1]
        out = np.empty(N_JOINTS)
        for j in range(N_JOINTS):
            n = int(round((self._base_delay + self.extra_delay[j]) / self.dt))
            n = min(max(n, 0), len(self._buf) - 1)
            out[j] = self._buf[-1 - n][j]
        return out

    def step(self, u: np.ndarray, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """One physics tick.  Returns the torque applied to each joint.

        `q`/`qd` are simulator truth; they never leave this method.
        """
        u_eff = self._delayed_command(u)
        err = u_eff - q

        if self._has("C"):
            err = np.sign(err) * np.maximum(np.abs(err) - self.deadband, 0.0)

        # The motor torque and the output torque are not the same thing.  Friction
        # is a load the motor has to push *through*, so it is subtracted at the
        # output while the current still reflects the full motor torque.  Getting
        # this backwards (subtracting friction first, then deriving current from
        # what is left) makes the memo.txt section 35 case B fault invisible: a
        # gearbox seizing up would show no current rise at all.
        tau_motor = self.kp * err - self.kd * qd

        if self._has("E"):
            gain = 1.0 + self.torque_per_volt * (self.voltage - self.v_nominal)
            tau_motor = tau_motor * max(gain, 0.05)

        if self._has("B"):
            # velocity ceiling: no driving torque past the free-running speed
            over = np.abs(qd) > self.qd_max
            tau_motor = np.where(over & (np.sign(tau_motor) == np.sign(qd)), 0.0, tau_motor)
            limit = self.tau_max * self.tau_scale
            tau_motor = np.clip(tau_motor, -limit, limit)

        tau = tau_motor
        if self._has("D"):
            friction = self.friction_scale * (
                self.coulomb * np.tanh(qd / self.stick_velocity) + self.viscous * qd
            )
            tau = tau_motor - friction

        self.tau_cmd = tau
        self._update_current(tau_motor)
        return tau

    # -- current and power -------------------------------------------------
    def _update_current(self, tau: np.ndarray) -> None:
        self.current_true = self.i_idle + self.k_tau * np.abs(tau)
        if self._has("E"):
            total = float(np.sum(self.current_true))
            self.voltage = self.v_nominal - self.v_sag * total

    def measured_current(self) -> np.ndarray:
        i = self.current_true + self.i_bias
        if self.i_noise > 0:
            i = i + self.rng.normal(0.0, self.i_noise, N_JOINTS)
        i = np.maximum(i, 0.0)
        if self.i_quant > 0:
            i = np.round(i / self.i_quant) * self.i_quant
        return i

    def power(self) -> tuple[np.ndarray, float, float]:
        """(per-servo power W, total power W, total current A).  Section 22."""
        p = self.voltage * self.current_true
        return p, float(np.sum(p)), float(np.sum(self.current_true))

    def apply_backlash(self, q_motor: np.ndarray) -> np.ndarray:
        """Phase D: dead zone between motor and output shaft."""
        if not self._has("D") or self.backlash <= 0:
            return q_motor
        half = self.backlash / 2.0
        self._lash = np.clip(self._lash, q_motor - half, q_motor + half)
        return self._lash
