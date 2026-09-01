"""Fault injection (section 35) and observation-only fault detection (36, 37).

The injector reaches into the servo and sensor models -- it is part of the
world, not part of the robot.  The monitor is part of the robot and therefore
sees `Observation` and the commands it issued, nothing else.

    command sent + current high + foot motion low   ->  mechanical resistance
    command sent + current low  + foot motion low   ->  motor / drive failure
    foot force high + foot IMU vibration high       ->  unstable / slipping
    foot force perfectly constant while others move ->  force sensor stuck
    |foot accel| far from 1 g while standing        ->  foot IMU bias
    current lags the command by more than usual     ->  sluggish servo (delay)
"""
from __future__ import annotations

from collections import deque

import numpy as np

from . import N_JOINTS, N_LEGS
from .types import Observation

FLAG = {
    "mechanical_resistance": 1,
    "drive_failure": 2,
    "unstable_contact": 4,
    "force_sensor_stuck": 8,
    "foot_imu_bias": 16,
    "sluggish_response": 32,
}
FLAG_NAME = {v: k for k, v in FLAG.items()}
GRAVITY = 9.81


class FaultInjector:
    """Applies the memo.txt section 35 fault cases at a given simulation time."""

    def __init__(self, spec_list: list[dict], library: dict):
        self.pending = []
        for spec in spec_list or []:
            case = library[str(spec["case"]).upper()]
            self.pending.append(
                {
                    "t": float(spec.get("t_start", 0.0)),
                    "joint": int(spec.get("joint", 0)),
                    "leg": int(spec.get("leg", int(spec.get("joint", 0)) // 2)),
                    "case": str(spec["case"]).upper(),
                    "def": case,
                }
            )
        self.applied: list[dict] = []

    @property
    def expected(self) -> list[tuple[float, str, int]]:
        """[(t_start, expected_class, joint)] for the evaluator."""
        return [
            (p["t"], p["def"].get("expected_class", ""), p["joint"])
            for p in self.pending + self.applied
        ]

    def update(self, t: float, servos, sensors) -> list[str]:
        fired = []
        still = []
        for p in self.pending:
            if t >= p["t"]:
                self._apply(p, servos, sensors)
                self.applied.append(p)
                fired.append(f"{p['case']}:{p['def']['name']}")
            else:
                still.append(p)
        self.pending = still
        return fired

    @staticmethod
    def _apply(p: dict, servos, sensors) -> None:
        d = p["def"]
        prm = d.get("params", {})
        j, leg = p["joint"], p["leg"]
        if d["target"] == "servo":
            if "tau_scale" in prm:
                servos.tau_scale[j] = float(prm["tau_scale"])
            if "friction_scale" in prm:
                servos.friction_scale[j] = float(prm["friction_scale"])
            if "extra_delay" in prm:
                servos.extra_delay[j] = float(prm["extra_delay"])
        elif d["target"] == "foot_force":
            sensors.fsr.stuck[leg] = True
        elif d["target"] == "foot_imu":
            sensors.foot_imu.fault_accel_bias[leg] = np.asarray(
                prm.get("accel_bias", [0, 0, 0]), dtype=float
            )


class FaultMonitor:
    """Residual monitor.  Section 37: mean/std from normal data, no ML needed."""

    def __init__(self, cfg: dict, control_dt: float):
        self.enabled = bool(cfg.get("enabled", True))
        self.active_states = tuple(cfg.get("active_states", ["STAND", "DITHER"]))
        # `sluggish_response` is off by default: the command-to-current lag is
        # estimated opportunistically from whatever the robot happened to be
        # doing, and on the mock backend its scatter (+-0.042 s) is comparable to
        # the 0.100 s fault it is meant to see.  Detecting a pure delay wants the
        # controlled square-wave probe of the section 38 self-check, not passive
        # monitoring.  See docs/FINDINGS.md.
        self.detectors = tuple(cfg.get("detectors", [
            "mechanical_resistance", "drive_failure", "unstable_contact",
            "force_sensor_stuck", "foot_imu_bias",
        ]))
        self.dt = float(control_dt)
        self.baseline_time = float(cfg["baseline_time"])
        self.k = float(cfg["k_sigma"])
        self.min_consecutive = int(cfg["min_consecutive"])
        self.cmd_activity_min = float(cfg["cmd_activity_min"])
        self.max_lag = float(cfg["max_lag"])
        self.lag_margin = float(cfg["lag_margin"])
        self.current_ratio_high = float(cfg.get("current_ratio_high", 1.15))
        self.current_ratio_low = float(cfg.get("current_ratio_low", 0.90))
        self.motion_ratio_max = float(cfg.get("motion_ratio_max", 0.70))
        self.motion_ratio_slip = float(cfg.get("motion_ratio_slip", 5.0))
        self.motion_ratio_soft = float(cfg.get("motion_ratio_soft", 0.90))
        self.dynamic_current_margin = float(cfg.get("dynamic_current_margin", 0.03))
        self.moving_threshold = float(cfg.get("moving_threshold", 0.05))
        n = max(4, int(float(cfg["window"]) / self.dt))
        self.n = n
        self.u_hist: deque[np.ndarray] = deque(maxlen=n)
        self.i_hist: deque[np.ndarray] = deque(maxlen=n)
        self.f_hist: deque[np.ndarray] = deque(maxlen=n)
        self.a_hist: deque[np.ndarray] = deque(maxlen=n)
        self._base_samples: list[dict] = []
        self.baseline: dict | None = None
        self._streak = np.zeros((N_JOINTS, len(FLAG)), dtype=int)
        self.flags = 0
        self.detections: list[tuple[float, str, int]] = []   # (t, class, joint)

    # ------------------------------------------------------------------
    def _features(self) -> dict:
        u = np.stack(self.u_hist)          # (n, 8)
        i = np.stack(self.i_hist)          # (n, 8)
        f = np.stack(self.f_hist)          # (n, 4)
        a = np.stack(self.a_hist)          # (n, 4, 3)
        return {
            "cmd_activity": np.std(u, axis=0),
            "current_dynamic": self._current_dynamic(u, i),
            "current": np.mean(i, axis=0),
            "current_var": np.std(i, axis=0),
            "force": np.mean(f, axis=0),
            "force_var": np.std(f, axis=0),
            "motion": np.sqrt(np.mean(np.sum((a - a.mean(axis=0)) ** 2, axis=2), axis=0)),
            "accel_mag": np.mean(np.linalg.norm(a, axis=2), axis=0),
            "lag": self._lag(u, i),
        }

    def _current_dynamic(self, u: np.ndarray, i: np.ndarray) -> np.ndarray:
        """Extra current drawn while the joint is actually moving.

        Friction is a *motion* cost: at rest the position loop settles at the
        same current whether the gearbox is clean or seized.  Averaged over a
        window that is mostly settling time, "friction x10" is invisible (0.872
        vs 0.876 of baseline, measured).  Comparing the current during command
        transitions against the current while the command is held recovers it.
        """
        du = np.abs(np.diff(u, axis=0, prepend=u[:1])) / self.dt
        out = np.zeros(N_JOINTS)
        for j in range(N_JOINTS):
            moving = du[:, j] > self.moving_threshold
            if moving.sum() < 2 or (~moving).sum() < 2:
                continue
            out[j] = float(i[moving, j].mean() - i[~moving, j].mean())
        return out

    def _lag(self, u: np.ndarray, i: np.ndarray) -> np.ndarray:
        """Cross-correlation lag (s) between |du/dt| and current, per joint."""
        max_k = max(1, int(self.max_lag / self.dt))
        du = np.abs(np.diff(u, axis=0, prepend=u[:1]))
        out = np.zeros(N_JOINTS)
        for j in range(N_JOINTS):
            x = du[:, j] - du[:, j].mean()
            y = i[:, j] - i[:, j].mean()
            if np.std(x) < 1e-9 or np.std(y) < 1e-9:
                continue
            best, best_k = -np.inf, 0
            for k in range(0, min(max_k, len(x) - 2)):
                c = float(np.dot(x[: len(x) - k], y[k:])) / (len(x) - k)
                if c > best:
                    best, best_k = c, k
            out[j] = best_k * self.dt
        return out

    def _z(self, name: str, value: np.ndarray) -> np.ndarray:
        mu = self.baseline[name + "_mean"]
        sd = np.maximum(self.baseline[name + "_std"], 1e-6)
        return (value - mu) / sd

    def update(self, t: float, obs: Observation, u_cmd: np.ndarray, state: str = "") -> int:
        """memo_full.txt defines the residual as r = y - y_hat(u, contact state).
        The contact state part is not optional: a leg that is deliberately in the
        air has no load and a freely swinging foot, which looks exactly like the
        signature of a broken one.  So the monitor only learns and only judges
        while the robot is in a quasi-static supported state.
        """
        if not self.enabled:
            return 0
        if state and state not in self.active_states:
            self._streak[:] = 0
            return self.flags
        self.u_hist.append(np.asarray(u_cmd, dtype=float))
        self.i_hist.append(np.asarray(obs.servo_current, dtype=float))
        self.f_hist.append(np.asarray(obs.foot_force, dtype=float))
        self.a_hist.append(np.asarray(obs.foot_accel, dtype=float))
        if len(self.u_hist) < self.n:
            return self.flags

        feat = self._features()
        if t < self.baseline_time:
            self._base_samples.append(feat)
            return self.flags
        if self.baseline is None:
            self._freeze_baseline()

        self._evaluate(t, feat)
        return self.flags

    def _freeze_baseline(self) -> None:
        keys = self._base_samples[0].keys()
        self.baseline = {}
        for k in keys:
            arr = np.stack([s[k] for s in self._base_samples])
            self.baseline[k + "_mean"] = arr.mean(axis=0)
            self.baseline[k + "_std"] = arr.std(axis=0)

    def _evaluate(self, t: float, feat: dict) -> None:
        """Ratio and absolute tests, not z-scores.

        A z-test needs a baseline sigma that is small compared to the effect.
        Measured on the mock backend the baseline scatter of these features is
        the same order as their mean (e.g. force variance 0.12 +- 0.15), so
        `z < -4` is unreachable no matter how broken the robot is.  Ratios
        against the baseline mean, and one absolute floor per test, separate the
        five memo.txt section 35 cases cleanly.
        """
        base_i = np.maximum(self.baseline["current_mean"], 1e-3)
        base_m = np.maximum(self.baseline["motion_mean"], 1e-3)
        current_ratio = feat["current"] / base_i
        base_dyn = self.baseline["current_dynamic_mean"]
        dyn_excess = feat["current_dynamic"] - base_dyn
        motion_ratio = feat["motion"] / base_m
        driven = feat["cmd_activity"] > self.cmd_activity_min
        baseline_driven = self.baseline["cmd_activity_mean"] > self.cmd_activity_min
        lag_excess = (
            feat["lag"] - self.baseline["lag_mean"] > self.lag_margin
        ) & baseline_driven

        for j in range(N_JOINTS):
            leg = j // 2
            stalled = motion_ratio[leg] < self.motion_ratio_max
            # section 36: command sent + working harder + moving less
            self._vote(t, j, "mechanical_resistance",
                       bool(driven[j] and stalled
                            and (current_ratio[j] > self.current_ratio_high
                                 or dyn_excess[j] > self.dynamic_current_margin)))
            # section 36: command sent + not working + not moving
            self._vote(t, j, "drive_failure",
                       bool(driven[j] and stalled
                            and current_ratio[j] < self.current_ratio_low
                            and dyn_excess[j] <= self.dynamic_current_margin))
            if "sluggish_response" in self.detectors:
                self._vote(t, j, "sluggish_response",
                           bool(driven[j] and lag_excess[j] and not stalled))

        others = feat["force_var"]
        for leg in range(N_LEGS):
            j = 2 * leg
            rest = np.median(np.delete(others, leg))
            self._vote(t, j, "force_sensor_stuck",
                       bool(others[leg] < 1e-6 and rest > 5e-3))
            drift = abs(feat["accel_mag"][leg] - self.baseline["accel_mag_mean"][leg])
            self._vote(t, j, "foot_imu_bias",
                       bool(drift > max(self.k * self.baseline["accel_mag_std"][leg], 0.25)))
            # a leg the robot is deliberately dithering moves more than baseline
            # by construction: that is a probe, not a slip
            leg_driven = bool(driven[2 * leg] or driven[2 * leg + 1])
            self._vote(t, j, "unstable_contact",
                       bool(not leg_driven
                            and feat["force"][leg] > 0.5 * self.baseline["force_mean"][leg]
                            and motion_ratio[leg] > self.motion_ratio_slip))

    def _vote(self, t: float, joint: int, cls: str, condition: bool) -> None:
        col = list(FLAG).index(cls)
        if condition:
            self._streak[joint, col] += 1
            if self._streak[joint, col] == self.min_consecutive:
                self.flags |= FLAG[cls]
                self.detections.append((t, cls, joint))
        else:
            self._streak[joint, col] = 0
