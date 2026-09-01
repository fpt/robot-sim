"""The six graphs memo.txt section 44 asks for after every run, plus two more.

Graph 7 (estimate vs truth) and graph 8 (state timeline) are not in the memo but
answer section 43 -- "how far could we get without joint feedback?" -- at a
glance, which is the whole point of the project.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

RAD = 180.0 / np.pi
LEGS = ("FL", "FR", "RL", "RR")
LEG_COLORS = {"FL": "#1f77b4", "FR": "#d62728", "RL": "#2ca02c", "RR": "#9467bd"}


def _save(fig, out: Path, name: str) -> Path:
    out.mkdir(parents=True, exist_ok=True)
    path = out / name
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return path


def make_plots(run, out_dir: Path | None = None) -> list[Path]:
    c, tr, t = run.control, run.truth, run.control["timestamp"]
    out = Path(out_dir or (run.path / "plots"))
    made = []

    # 1 - attitude
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.plot(t, c["body_roll_est"] * RAD, label="roll (estimated)", color="#1f77b4")
    ax.plot(t, c["body_pitch_est"] * RAD, label="pitch (estimated)", color="#ff7f0e")
    if "body_roll" in tr:
        n = min(len(t), len(tr["body_roll"]))
        ax.plot(t[:n], tr["body_roll"][:n] * RAD, ":", color="#1f77b4", lw=1, label="roll (truth)")
        ax.plot(t[:n], tr["body_pitch"][:n] * RAD, ":", color="#ff7f0e", lw=1,
                label="pitch (truth)")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel("angle [deg]")
    ax.set_title("1. Body attitude (dotted = truth, never seen by the controller)")
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)
    made.append(_save(fig, out, "01_attitude.png"))

    # 2 - foot forces
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for n_ in LEGS:
        ax.plot(t, c[f"F_{n_}"], label=f"F_{n_}", color=LEG_COLORS[n_], lw=1)
    if "sm_F_target" in c:
        ax.plot(t, c["sm_F_target"], "k--", lw=1, label="F* (target leg)")
    ax.set_xlabel("t [s]"); ax.set_ylabel("force [N]")
    ax.set_title("2. Foot forces (FSR model, saturates at 9.8 N)")
    ax.legend(fontsize=8, ncol=5); ax.grid(alpha=0.3)
    made.append(_save(fig, out, "02_foot_forces.png"))

    # 3 - servo currents
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for j in range(8):
        ax.plot(t, c[f"I_{j}"], lw=0.8, label=f"I_{j}")
    ax.set_xlabel("t [s]"); ax.set_ylabel("current [A]")
    ax.set_title("3. Servo currents")
    ax.legend(fontsize=7, ncol=8); ax.grid(alpha=0.3)
    made.append(_save(fig, out, "03_currents.png"))

    # 4 - command vs current
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.4))
    for j, ax in zip((1, 3), axes, strict=True):
        ax.scatter(c[f"u_{j}"], c[f"I_{j}"], s=2, alpha=0.35, c=t, cmap="viridis")
        ax.set_xlabel(f"u_{j} [rad]"); ax.set_ylabel(f"I_{j} [A]")
        ax.set_title(f"joint {j}"); ax.grid(alpha=0.3)
    fig.suptitle("4. Servo command vs current (colour = time)")
    made.append(_save(fig, out, "04_command_vs_current.png"))

    # 5 - objective
    if "J_total" in c:
        fig, ax = plt.subplots(figsize=(9, 3.2))
        ax.plot(t, c["J_total"], label="J", color="k", lw=1)
        for key, color in (("J_pose", "#1f77b4"), ("J_force", "#2ca02c"), ("J_power", "#d62728")):
            if key in c:
                ax.plot(t, c[key], lw=0.8, alpha=0.7, label=key, color=color)
        ax.set_yscale("symlog", linthresh=1e-3)
        ax.set_xlabel("t [s]"); ax.set_ylabel("J (unweighted terms)")
        ax.set_title("5. Objective J over time")
        ax.legend(fontsize=8, ncol=4); ax.grid(alpha=0.3)
        made.append(_save(fig, out, "05_objective.png"))

    # 6 - dither probe vs response
    if "dither_dJ" in c and "dither_updates" in c:
        change = np.where(np.diff(c["dither_updates"]) > 0)[0]
        if change.size:
            fig, axes = plt.subplots(1, 2, figsize=(9, 3.4))
            joints = c["dither_joint"][change].astype(int)
            axes[0].scatter(np.arange(change.size), c["dither_dJ"][change],
                            c=joints, cmap="tab10", s=14)
            axes[0].axhline(0, color="k", lw=0.5)
            axes[0].set_xlabel("gradient update #"); axes[0].set_ylabel("J(+d) - J(-d)")
            axes[0].set_title("probe response (colour = joint)")
            axes[1].scatter(c["dither_dI"][change], c["dither_dJ"][change],
                            c=joints, cmap="tab10", s=14)
            axes[1].axhline(0, color="k", lw=0.5); axes[1].axvline(0, color="k", lw=0.5)
            axes[1].set_xlabel("I(+d) - I(-d)  [A]"); axes[1].set_ylabel("J(+d) - J(-d)")
            axes[1].set_title("current difference vs objective difference")
            for ax in axes:
                ax.grid(alpha=0.3)
            fig.suptitle("6. Dither direction vs delta-J (memo section 27, 28)")
            made.append(_save(fig, out, "06_dither.png"))

    # 7 - estimator error
    if "body_roll" in tr:
        n = min(len(t), len(tr["body_roll"]))
        fig, ax = plt.subplots(figsize=(9, 3.0))
        ax.plot(t[:n], (c["body_roll_est"][:n] - tr["body_roll"][:n]) * RAD, label="roll error")
        ax.plot(t[:n], (c["body_pitch_est"][:n] - tr["body_pitch"][:n]) * RAD, label="pitch error")
        ax.set_xlabel("t [s]"); ax.set_ylabel("estimate - truth [deg]")
        ax.set_title("7. IMU-only attitude estimate vs simulator truth (section 43)")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
        made.append(_save(fig, out, "07_estimator_error.png"))

    # 8 - state timeline
    if run.states and len(set(run.states)) > 1:
        labels = sorted(set(run.states))
        idx = np.array([labels.index(s) for s in run.states])
        fig, ax = plt.subplots(figsize=(9, 2.6))
        ax.step(t, idx, where="post", lw=1.2)
        ax.set_yticks(range(len(labels))); ax.set_yticklabels(labels, fontsize=8)
        ax.set_xlabel("t [s]"); ax.set_title("8. Controller state")
        ax.grid(alpha=0.3)
        made.append(_save(fig, out, "08_states.png"))

    return made
