"""Sensor models: body IMU, four foot IMUs, four FSRs, battery.

memo.txt sections 15, 16, 17, 18.  Every model here turns simulator truth into
the degraded thing a cheap part would actually report.  The degraded value is
what the controller sees; the truth goes to the ground-truth log only.
"""
from __future__ import annotations

import numpy as np

from . import N_LEGS

GRAVITY = 9.81


class ImuModel:
    def __init__(self, cfg: dict, count: int, rng: np.random.Generator):
        self.count = count
        self.rng = rng
        self.accel_noise = float(cfg["accel_noise_std"])
        self.gyro_noise = float(cfg["gyro_noise_std"])
        self.accel_bias = rng.normal(0.0, float(cfg["accel_bias_std"]), (count, 3))
        self.gyro_bias = rng.normal(0.0, float(cfg["gyro_bias_std"]), (count, 3))
        self.fault_accel_bias = np.zeros((count, 3))

    def read(self, accel_true: np.ndarray, gyro_true: np.ndarray):
        a = np.atleast_2d(accel_true).astype(float) + self.accel_bias + self.fault_accel_bias
        g = np.atleast_2d(gyro_true).astype(float) + self.gyro_bias
        a = a + self.rng.normal(0.0, self.accel_noise, a.shape)
        g = g + self.rng.normal(0.0, self.gyro_noise, g.shape)
        return a, g


class FsrModel:
    """Foot force sensor.  Section 18:  F = clamp(Fz, 0, Fmax)."""

    def __init__(self, cfg: dict, rng: np.random.Generator, dt: float):
        self.f_max = float(cfg["f_max"])
        self.noise = float(cfg["noise_std"])
        self.quant = float(cfg["quantization"])
        self.contact_threshold = float(cfg["contact_threshold"])
        self.zero = rng.normal(0.0, float(cfg["zero_drift_std"]), N_LEGS)
        self.gain = 1.0 + rng.normal(0.0, float(cfg["gain_spread"]), N_LEGS)
        self.rng = rng
        lp = float(cfg.get("lowpass_hz", 0.0))
        self.alpha = 1.0 if lp <= 0 else min(1.0, 2 * np.pi * lp * dt)
        self._filtered = np.zeros(N_LEGS)
        self.stuck = np.zeros(N_LEGS, dtype=bool)   # fault case D
        self._last = np.zeros(N_LEGS)

    def read(self, fz_true: np.ndarray) -> np.ndarray:
        raw = np.maximum(np.asarray(fz_true, dtype=float), 0.0)
        self._filtered += self.alpha * (raw - self._filtered)
        f = self.gain * self._filtered + self.zero
        f = f + self.rng.normal(0.0, self.noise, N_LEGS)
        f = np.clip(f, 0.0, self.f_max)
        if self.quant > 0:
            f = np.round(f / self.quant) * self.quant
        out = np.where(self.stuck, self._last, f)
        self._last = out
        return out


class Battery:
    def __init__(self, cfg: dict, rng: np.random.Generator):
        self.noise = float(cfg["noise_std"])
        self.rng = rng

    def read(self, v_true: float) -> float:
        return float(v_true + self.rng.normal(0.0, self.noise))


class SensorSuite:
    """All sensors of section 14 in one place."""

    def __init__(self, sensor_cfg: dict, dt: float, rng: np.random.Generator):
        self.body_imu = ImuModel(sensor_cfg["body_imu"], 1, rng)
        self.foot_imu = ImuModel(sensor_cfg["foot_imu"], N_LEGS, rng)
        self.fsr = FsrModel(sensor_cfg["foot_force"], rng, dt)
        self.battery = Battery(sensor_cfg["battery"], rng)
