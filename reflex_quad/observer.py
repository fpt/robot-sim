"""Attitude estimation from the body IMU only.  memo.txt section 15.

Complementary filter: integrate the gyro, and pull the estimate slowly toward
the accelerometer's idea of "down".  The simulator's true orientation is never
consulted -- that comparison happens offline in eval/ (section 43).
"""
from __future__ import annotations

import numpy as np

GRAVITY = 9.81


class AttitudeObserver:
    def __init__(self, cfg: dict, dt: float):
        self.dt = float(dt)
        self.tau = float(cfg["tau_complementary"])
        self.trust_window = float(cfg["accel_trust_window"])
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self._initialised = False

    def update(self, accel: np.ndarray, gyro: np.ndarray) -> None:
        ax, ay, az = (float(v) for v in np.asarray(accel).reshape(3))
        wx, wy, _ = (float(v) for v in np.asarray(gyro).reshape(3))

        # accelerometer attitude: valid only when specific force is close to 1 g
        norm = float(np.linalg.norm([ax, ay, az]))
        accel_ok = abs(norm - GRAVITY) < self.trust_window and norm > 1e-6
        roll_acc = np.arctan2(ay, az) if accel_ok else self.roll
        pitch_acc = np.arctan2(-ax, np.hypot(ay, az)) if accel_ok else self.pitch

        if not self._initialised:
            self.roll, self.pitch = roll_acc, pitch_acc
            self._initialised = True

        # gyro rates -> euler rates (small angle: the two coincide)
        self.roll_rate, self.pitch_rate = wx, wy
        roll_gyro = self.roll + wx * self.dt
        pitch_gyro = self.pitch + wy * self.dt

        beta = self.dt / (self.tau + self.dt) if accel_ok else 0.0
        self.roll = (1 - beta) * roll_gyro + beta * roll_acc
        self.pitch = (1 - beta) * pitch_gyro + beta * pitch_acc

    @property
    def state(self) -> tuple[float, float, float, float]:
        return self.roll, self.pitch, self.roll_rate, self.pitch_rate
