"""Run logging.  memo.txt sections 41, 42.

Two separate files, on purpose:

    control.csv   everything the controller was allowed to see, plus J and state
    truth.csv     q, qdot, tau, true poses, true contact forces

Nothing reads truth.csv at run time.  It exists so that section 43 -- "how far
could we estimate without joint feedback?" -- can be answered after the fact.
"""
from __future__ import annotations

import csv
import json
import platform
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

from . import JOINT_NAMES, LEG_NAMES

CONTROL_COLUMNS = [
    "timestamp",
    "body_roll_est", "body_pitch_est", "roll_rate_est", "pitch_rate_est",
    "body_ax", "body_ay", "body_az", "body_wx", "body_wy", "body_wz",
    *[f"F_{n}" for n in LEG_NAMES],
    *[f"foot_acc_{n}" for n in LEG_NAMES],
    *[f"foot_gyro_{n}" for n in LEG_NAMES],
    *[f"I_{i}" for i in range(8)],
    *[f"u_{i}" for i in range(8)],
    "battery_voltage", "power_total_W", "current_total_A",
    "J_pose", "J_force", "J_power", "J_total",
    "state", "fault_flag",
]

TRUTH_COLUMNS = [
    "timestamp",
    *[f"q_{n}" for n in JOINT_NAMES],
    *[f"qd_{n}" for n in JOINT_NAMES],
    *[f"tau_{n}" for n in JOINT_NAMES],
    "body_x", "body_y", "body_z",
    "body_roll", "body_pitch", "body_yaw",
    "body_vx", "body_vy", "body_vz",
    "body_wx", "body_wy", "body_wz",
    *[f"foot_x_{n}" for n in LEG_NAMES],
    *[f"foot_y_{n}" for n in LEG_NAMES],
    *[f"foot_z_{n}" for n in LEG_NAMES],
    *[f"Fx_{n}" for n in LEG_NAMES],
    *[f"Fy_{n}" for n in LEG_NAMES],
    *[f"Fz_{n}" for n in LEG_NAMES],
    # net horizontal displacement of each foot from its own touchdown point,
    # 0 while airborne -- see MockBackend._update_slip / IsaacLabBackend
    # ._update_slip.
    *[f"foot_slip_{n}" for n in LEG_NAMES],
]


def _git_hash() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL, text=True
        ).strip()
    except Exception:
        return "nogit"


class RunLogger:
    def __init__(self, root: Path, experiment_id: str, cfg: dict, tag: str = ""):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"{experiment_id}_{stamp}" + (f"_{tag}" if tag else "")
        self.dir = Path(root) / name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.experiment_id = experiment_id
        self._extra_cols: list[str] = []
        self._rows: list[dict] = []
        self._truth_rows: list[dict] = []
        self.events: list[dict] = []
        self.meta = {
            "experiment_id": experiment_id,
            "started": stamp,
            "git": _git_hash(),
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "numpy": np.__version__,
            "config": _jsonable(cfg),
        }

    # ------------------------------------------------------------------
    def event(self, t: float, kind: str, **fields) -> None:
        self.events.append({"t": round(float(t), 6), "kind": kind, **_jsonable(fields)})

    def log(self, control_row: dict, truth_row: dict) -> None:
        for k in control_row:
            if k not in CONTROL_COLUMNS and k not in self._extra_cols:
                self._extra_cols.append(k)
        self._rows.append(control_row)
        self._truth_rows.append(truth_row)

    def close(self, summary: dict | None = None) -> Path:
        self._write(self.dir / "control.csv", CONTROL_COLUMNS + self._extra_cols, self._rows)
        self._write(self.dir / "truth.csv", TRUTH_COLUMNS, self._truth_rows)
        with open(self.dir / "events.jsonl", "w") as fh:
            for e in self.events:
                fh.write(json.dumps(e) + "\n")
        self.meta["samples"] = len(self._rows)
        self.meta["finished"] = datetime.now().strftime("%Y%m%d_%H%M%S")
        if summary:
            self.meta["summary"] = _jsonable(summary)
        with open(self.dir / "meta.json", "w") as fh:
            json.dump(self.meta, fh, indent=2, sort_keys=True)
        return self.dir

    @staticmethod
    def _write(path: Path, columns: list[str], rows: list[dict]) -> None:
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow({k: _fmt(r.get(k, "")) for k in columns})


def _fmt(v):
    if isinstance(v, (float, np.floating)):
        return f"{float(v):.6g}"
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, Path):
        return str(obj)
    return obj
