"""Turn one run directory into a flat dict of numbers.

Every key here is quotable in config/criteria.yaml.  Keep them scalar and keep
the names stable -- the criteria file is the contract.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

RAD = 180.0 / math.pi
LEGS = ("FL", "FR", "RL", "RR")


@dataclass
class Run:
    path: Path
    control: dict[str, np.ndarray]
    truth: dict[str, np.ndarray]
    events: list[dict]
    meta: dict
    states: list[str] = field(default_factory=list)

    @property
    def t(self) -> np.ndarray:
        return self.control["timestamp"]

    def has(self, key: str) -> bool:
        return key in self.control


def _read_csv(path: Path) -> tuple[dict[str, np.ndarray], list[str]]:
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return {}, []
    cols: dict[str, np.ndarray] = {}
    states: list[str] = []
    for key in rows[0]:
        raw = [r[key] for r in rows]
        if key == "state":
            states = raw
            continue
        try:
            cols[key] = np.array([float(v) if v not in ("", None) else np.nan for v in raw])
        except ValueError:
            continue
    return cols, states


def load_run(path: Path | str) -> Run:
    path = Path(path)
    control, states = _read_csv(path / "control.csv")
    truth, _ = _read_csv(path / "truth.csv")
    events = []
    ev = path / "events.jsonl"
    if ev.exists():
        events = [json.loads(line) for line in ev.read_text().splitlines() if line.strip()]
    meta = json.loads((path / "meta.json").read_text())
    return Run(path=path, control=control, truth=truth, events=events, meta=meta, states=states)


# ----------------------------------------------------------------------
def _tail(a: np.ndarray, t: np.ndarray, seconds: float = 2.0) -> np.ndarray:
    mask = t >= (t[-1] - seconds)
    return a[mask]


def _settling_time(sig: np.ndarray, t: np.ndarray, band: float) -> float:
    """First time after which |sig - final| stays inside `band` for good."""
    final = float(np.mean(_tail(sig, t)))
    outside = np.abs(sig - final) > band
    idx = np.where(outside)[0]
    if idx.size == 0:
        return 0.0
    return float(t[min(idx[-1] + 1, len(t) - 1)])


def _mann_kendall(x: np.ndarray) -> tuple[float, float]:
    """Trend test.  Returns (S normalised to [-1, 1], two-sided p).

    The series is subsampled to at most 200 points first: consecutive control
    samples are strongly autocorrelated and Mann-Kendall assumes independence,
    so on the raw 100 Hz series any trend at all comes out as p = 0.
    """
    if len(x) > 200:
        x = x[np.linspace(0, len(x) - 1, 200).astype(int)]
    n = len(x)
    if n < 10:
        return 0.0, 1.0
    s = 0
    for i in range(n - 1):
        s += int(np.sum(np.sign(x[i + 1:] - x[i])))
    var = n * (n - 1) * (2 * n + 5) / 18.0
    if var <= 0:
        return 0.0, 1.0
    z = (s - np.sign(s)) / math.sqrt(var)
    p = math.erfc(abs(z) / math.sqrt(2))
    return float(s / (n * (n - 1) / 2)), float(p)


def compute_metrics(run: Run) -> dict:
    c, tr, t = run.control, run.truth, run.control["timestamp"]
    m: dict = {"experiment_id": run.meta.get("experiment_id"), "samples": len(t),
               "duration_s": float(t[-1]) if len(t) else 0.0}

    # -- sanity ---------------------------------------------------------
    stacked = np.vstack([v for v in c.values() if v.ndim == 1 and len(v) == len(t)])
    m["nan_count"] = int(np.count_nonzero(~np.isfinite(stacked)))

    roll = c["body_roll_est"] * RAD
    pitch = c["body_pitch_est"] * RAD
    tilt = np.hypot(roll, pitch)
    currents = np.vstack([c[f"I_{j}"] for j in range(8)])
    forces = np.vstack([c[f"F_{n}"] for n in LEGS])
    body_z = tr.get("body_z", np.zeros_like(t))

    m["current_max_A"] = float(np.nanmax(currents))
    m["current_mean_A"] = float(np.nanmean(currents))
    m["power_mean_W"] = float(np.nanmean(c["power_total_W"]))
    m["max_abs_tilt_deg"] = float(np.nanmax(tilt))
    m["fell_over"] = bool(m["max_abs_tilt_deg"] > 45.0)
    m["diverged"] = bool(
        m["nan_count"] > 0
        or m["max_abs_tilt_deg"] > 60.0
        or np.nanmax(np.abs(body_z)) > 1.0
        or m["current_max_A"] > 20.0
    )

    # -- posture --------------------------------------------------------
    m["final_abs_roll_deg"] = float(abs(np.mean(_tail(roll, t))))
    m["final_abs_pitch_deg"] = float(abs(np.mean(_tail(pitch, t))))
    m["final_abs_tilt_deg"] = float(np.mean(_tail(tilt, t)))
    m["settling_time_s"] = _settling_time(tilt, t, band=max(0.5, 0.2 * float(np.nanmax(tilt))))
    early = tilt[t <= 0.3 * t[-1]]
    m["peak_early_tilt_deg"] = float(np.nanmax(early)) if early.size else 0.0
    peak_roll = float(np.nanmax(np.abs(roll[t <= 0.3 * t[-1]]))) if early.size else 0.0
    m["roll_improvement_ratio"] = float(peak_roll / max(m["final_abs_roll_deg"], 1e-3))
    m["tilt_improvement_ratio"] = float(
        m["peak_early_tilt_deg"] / max(m["final_abs_tilt_deg"], 1e-3))

    # -- load distribution ----------------------------------------------
    total = forces.sum(axis=0)
    m["total_force_mean_N"] = float(np.mean(_tail(total, t)))
    final_f = np.array([np.mean(_tail(forces[i], t)) for i in range(4)])
    m["final_force_cv"] = float(np.std(final_f) / max(np.mean(final_f), 1e-6))
    m["min_foot_force_final_N"] = float(np.min(final_f))
    m["final_forces_N"] = [round(float(v), 3) for v in final_f]

    # -- objective -------------------------------------------------------
    if "J_total" in c:
        j = c["J_total"]
        n = max(1, len(j) // 5)
        m["J_first_quintile"] = float(np.mean(j[:n]))
        m["J_last_quintile"] = float(np.mean(j[-n:]))
        m["J_improvement_ratio"] = float(m["J_first_quintile"] / max(m["J_last_quintile"], 1e-9))
        tau, p = _mann_kendall(j)
        m["J_trend_tau"] = tau
        m["J_trend_p_value"] = p
        m["J_final"] = float(np.mean(_tail(j, t)))

    m.update(_dither_metrics(run))
    m.update(_state_machine_metrics(run))
    m.update(_fault_metrics(run))
    m.update(_truth_comparison(run))
    return m


def _dither_metrics(run: Run) -> dict:
    c = run.control
    if "dither_grad" not in c or "dither_updates" not in c:
        return {}
    upd = c["dither_updates"]
    change = np.where(np.diff(upd) > 0)[0]
    if change.size == 0:
        return {"dither_update_count": 0, "gradient_sign_consistency": 0.0}
    grads = c["dither_grad"][change]
    joints = c["dither_joint"][change].astype(int)

    def consistency(sl: slice) -> float:
        consist, weight = 0.0, 0
        for j in np.unique(joints[sl]):
            g = np.sign(grads[sl][joints[sl] == j])
            g = g[g != 0]
            if g.size < 2:
                continue
            consist += float(max(np.sum(g > 0), np.sum(g < 0)))
            weight += g.size
        return float(consist / weight) if weight else 0.0

    # Judge the search phase, not the whole run.  Once the offset sits at the
    # optimum the gradient sign SHOULD alternate -- that is a converged dither
    # hunting around the minimum, and scoring it as inconsistency would punish
    # exactly the behaviour we are trying to produce.
    half = max(1, int(0.6 * len(grads)))
    return {
        "dither_update_count": int(upd[-1]),
        "gradient_sign_consistency": consistency(slice(0, half)),
        "gradient_sign_consistency_full": consistency(slice(0, len(grads))),
        "dither_offset_norm_final": float(c["u_offset_norm"][-1]) if "u_offset_norm" in c else 0.0,
    }


def _state_machine_metrics(run: Run) -> dict:
    c, tr, t = run.control, run.truth, run.control["timestamp"]
    if "sm_target_leg_force" not in c:
        return {}
    states = np.array(run.states)
    leg = int(c["sm_target_leg"][0]) if "sm_target_leg" in c else 0
    lifted = np.isin(states, ["LIFT", "VERIFY_MOTION"])
    out = {
        "sm_min_target_leg_force_N": float(np.nanmin(c["sm_target_leg_force"])),
        "sm_final_target_leg_force_N": float(np.mean(_tail(c["sm_target_leg_force"], t))),
        "sm_motion_verified": bool(run.meta.get("summary", {}).get("motion_verified", False)),
        "sm_cycles": int(run.meta.get("summary", {}).get("cycles", 0)),
        "sm_aborted": bool(run.meta.get("summary", {}).get("aborted_reason", "")),
        "sm_returned_to_stand": bool(np.any(np.isin(states, ["DONE"]))),
        "sm_states_visited": sorted(set(states.tolist())),
        "sm_max_motion_metric": float(np.nanmax(c["sm_motion_metric"])),
    }
    tilt = np.hypot(c["body_roll_est"], c["body_pitch_est"]) * RAD
    out["max_abs_tilt_during_lift_deg"] = float(np.nanmax(tilt[lifted])) if lifted.any() else 0.0
    key = f"foot_z_{LEGS[leg]}"
    if key in tr and lifted.any():
        out["sm_max_foot_lift_m"] = float(np.nanmax(tr[key][lifted]) - np.nanmin(tr[key][~lifted]))
    if "sm_forward" in c:
        out["sm_foot_forward_displacement_m"] = float(np.mean(_tail(c["sm_forward"], t)))
    return out


def _fault_metrics(run: Run) -> dict:
    plan = run.meta.get("fault_plan", [])
    det = [e for e in run.events if e["kind"] == "fault_detected"]
    out = {
        "fault_planned": len(plan),
        "fault_detection_count": len(det),
        "fault_detected": bool(det),
        "fault_false_alarm_count": 0,
        "fault_class_correct": False,
        "fault_joint_correct": False,
        "fault_detection_latency_s": float("inf"),
        "fault_classes_detected": sorted({e["detected_class"] for e in det}),
    }
    if not plan:
        out["fault_false_alarm_count"] = len(det)
        out["fault_detected"] = bool(det)
        return out
    t0 = min(p["t"] for p in plan)
    expected = {p["expected_class"] for p in plan}
    joints = {p["joint"] for p in plan}
    out["fault_false_alarm_count"] = sum(1 for e in det if e["t"] < t0)
    hits = [e for e in det if e["t"] >= t0 and e["detected_class"] in expected]
    if hits:
        out["fault_class_correct"] = True
        out["fault_detection_latency_s"] = float(hits[0]["t"] - t0)
        out["fault_joint_correct"] = bool(hits[0]["joint"] in joints)
    elif det:
        after = [e for e in det if e["t"] >= t0]
        if after:
            out["fault_detection_latency_s"] = float(after[0]["t"] - t0)
    return out


def _truth_comparison(run: Run) -> dict:
    """memo.txt section 43 -- only ever done after the fact."""
    c, tr = run.control, run.truth
    if "body_roll" not in tr:
        return {}
    n = min(len(tr["body_roll"]), len(c["body_roll_est"]))
    dr = (c["body_roll_est"][:n] - tr["body_roll"][:n]) * RAD
    dp = (c["body_pitch_est"][:n] - tr["body_pitch"][:n]) * RAD
    out = {
        "roll_estimate_rmse_deg": float(np.sqrt(np.nanmean(dr**2))),
        "pitch_estimate_rmse_deg": float(np.sqrt(np.nanmean(dp**2))),
        "roll_estimate_bias_deg": float(np.nanmean(dr)),
        "pitch_estimate_bias_deg": float(np.nanmean(dp)),
    }
    fz = np.vstack([tr[f"Fz_{n_}"] for n_ in LEGS if f"Fz_{n_}" in tr])
    fs = np.vstack([c[f"F_{n_}"] for n_ in LEGS])
    k = min(fz.shape[1], fs.shape[1])
    out["foot_force_rmse_N"] = float(np.sqrt(np.nanmean((fs[:, :k] - fz[:, :k]) ** 2)))
    return out
