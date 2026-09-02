"""The experiment loop that ties everything together.

    servo model  <- command u          (500 Hz, sees q/qdot, hides them)
    backend      <- joint torque       (500 Hz)
    sensor models<- simulator truth    (500 Hz, section 12)
    controller   <- Observation only   (100 Hz)
    logger       <- both streams       (100 Hz, section 41)

The controller is constructed with the config and the leg geometry and nothing
else.  It is never handed the backend, the servo bank or a SimState, so it
cannot reach joint truth even if someone edits controller.py carelessly.
"""
from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np

from . import JOINT_NAMES, LEG_NAMES, N_JOINTS
from .backends import make_backend
from .config import Config, apply_fidelity, load_experiment
from .controller import HoldController, PostureController
from .dither import DitherController
from .faults import FaultInjector, FaultMonitor
from .logger import RunLogger
from .robot import LegGeometry
from .sensors import SensorSuite
from .servo_model import ServoBank
from .state_machine import LegCycleController
from .types import Observation

CONTROLLERS = {
    "hold": HoldController,
    "posture": PostureController,
    "dither": DitherController,
    "statemachine": LegCycleController,
}
COMMAND_HISTORY = 16

# Sanity bounds for _assert_sane_initial_state, not a research tuning value --
# every backend's reset() is contractually supposed to reproduce
# geom.nominal_command() exactly, so these are slack for float/solver noise,
# not something an experiment should ever need to change.
INITIAL_STATE_TOL = {
    "joint_pos_rad": 0.01,       # ~0.6 deg
    "foot_penetration_m": 0.005,  # 5 mm into the terrain
    "foot_gap_m": 0.05,           # 5 cm floating above it
}


def make_controller(cfg: dict, geom: LegGeometry):
    mode = cfg["controller"]["mode"]
    if mode not in CONTROLLERS:
        raise ValueError(f"unknown controller mode {mode!r}; have {sorted(CONTROLLERS)}")
    return CONTROLLERS[mode](cfg, geom)


def _assert_isolated(controller, backend, servos) -> None:
    """memo.txt section 13, enforced at run time as well as in the tests."""
    for name, value in vars(controller).items():
        if value is backend or value is servos:
            raise AssertionError(
                f"controller.{name} holds a reference to simulator internals"
            )


def _assert_sane_initial_state(sim, geom: LegGeometry, backend) -> None:
    """Catch a broken reset() before spending a run on it.

    Every backend's reset() is supposed to put the robot at
    geom.nominal_command() with the feet resting on the terrain under them.
    IsaacLabBackend.reset() silently violated that -- it wrote the physics
    scene's default state but never actually applied it, so the robot spawned
    at the URDF's raw rest pose instead (straight legs, ~4 cm of foot
    penetration) -- and it took a full run of tumbling truth.csv to notice.
    See docs/FINDINGS.md #15.  This is cheap (t=0 only) and backend-agnostic:
    it reads only what SimState already returns, the same truth every backend
    logs to truth.csv anyway, not something a controller may see.
    """
    u0 = geom.nominal_command()
    if not np.all(np.isfinite(sim.q)):
        raise AssertionError(f"reset(): non-finite joint_pos {sim.q}")
    err = np.abs(sim.q - u0)
    bad = err > INITIAL_STATE_TOL["joint_pos_rad"]
    if np.any(bad):
        bad_joints = [JOINT_NAMES[i] for i in np.flatnonzero(bad)]
        raise AssertionError(
            "reset(): joint_pos does not match nominal_command() -- the "
            f"backend did not apply init_state.  q={sim.q}, expected={u0}, "
            f"off by {err[bad]} rad on {bad_joints}"
        )
    hip_lo, hip_hi = geom.hip_limit
    knee_lo, knee_hi = geom.knee_limit
    for i, name in enumerate(JOINT_NAMES):
        lo, hi = (hip_lo, hip_hi) if i % 2 == 0 else (knee_lo, knee_hi)
        if not (lo <= sim.q[i] <= hi):
            raise AssertionError(
                f"reset(): {name}={sim.q[i]:.4f} rad outside its limit [{lo}, {hi}]"
            )

    if not np.all(np.isfinite(sim.foot_pos)):
        raise AssertionError(f"reset(): non-finite foot_pos {sim.foot_pos}")
    for i, name in enumerate(LEG_NAMES):
        x, y, z = (float(v) for v in sim.foot_pos[i])
        ground = backend.terrain_height(x, y)
        gap = z - ground
        if gap < -INITIAL_STATE_TOL["foot_penetration_m"]:
            raise AssertionError(
                f"reset(): foot {name} starts {-gap * 1000:.1f} mm inside the "
                f"terrain at ({x:.3f}, {y:.3f}) -- foot_z={z:.4f}, terrain={ground:.4f}"
            )
        if gap > INITIAL_STATE_TOL["foot_gap_m"]:
            raise AssertionError(
                f"reset(): foot {name} starts {gap * 1000:.1f} mm above the "
                f"terrain at ({x:.3f}, {y:.3f}) -- foot_z={z:.4f}, terrain={ground:.4f}"
            )


def run_experiment(
    exp_id: str,
    *,
    log_root: Path | str = "logs",
    seed: int | None = None,
    duration: float | None = None,
    backend_name: str | None = None,
    fidelity: int | None = None,
    tag: str = "",
    progress: bool = True,
    close_backend: bool = True,
) -> Path:
    cfg: Config = load_experiment(exp_id)
    if seed is not None:
        cfg["seed"] = int(seed)
    if duration is not None:
        cfg["duration"] = float(duration)
    if backend_name is not None:
        cfg["backend"] = backend_name
    rng = np.random.default_rng(int(cfg["seed"]))
    if fidelity is not None:
        apply_fidelity(cfg, fidelity, rng)

    geom = LegGeometry(cfg["robot"])
    backend = make_backend(cfg["backend"], cfg, rng)
    servos = ServoBank(cfg["servo"], cfg["physics_dt"], rng)
    sensors = SensorSuite(cfg["sensors"], cfg["physics_dt"], rng)
    controller = make_controller(cfg, geom)
    monitor = FaultMonitor(cfg["fault_monitor"], cfg["control_dt"])
    injector = FaultInjector(cfg.get("faults", []), cfg["fault_library"])
    _assert_isolated(controller, backend, servos)

    logger = RunLogger(Path(log_root), exp_id, cfg, tag=tag)
    logger.meta["backend"] = cfg["backend"]
    logger.meta["fidelity"] = cfg.get("fidelity", {"stage": None})
    logger.meta["fault_plan"] = [
        {"t": t, "expected_class": c, "joint": j} for t, c, j in injector.expected
    ]

    dt = float(cfg["physics_dt"])
    control_every = max(1, int(round(float(cfg["control_dt"]) / dt)))
    n_steps = int(round(float(cfg["duration"]) / dt))
    hist = deque([geom.nominal_command().copy()] * COMMAND_HISTORY, maxlen=COMMAND_HISTORY)
    u = geom.nominal_command()

    sim = backend.reset()
    _assert_sane_initial_state(sim, geom, backend)
    last_state = "INIT"
    try:
        for k in range(n_steps):
            t = k * dt
            for fired in injector.update(t, servos, sensors):
                logger.event(t, "fault_injected", detail=fired)

            tau = servos.step(u, sim.q, sim.qd)
            sim = backend.step(tau)

            body_a, body_g = sensors.body_imu.read(sim.body_accel_body, sim.body_omega)
            foot_a, foot_g = sensors.foot_imu.read(sim.foot_accel_body, sim.foot_omega)
            foot_f = sensors.fsr.read(sim.contact_force[:, 2])
            current = servos.measured_current()
            voltage = sensors.battery.read(servos.voltage)

            if k % control_every == 0:
                obs = Observation(
                    t=t,
                    body_accel=body_a[0],
                    body_gyro=body_g[0],
                    foot_accel=foot_a,
                    foot_gyro=foot_g,
                    foot_force=foot_f,
                    servo_current=current,
                    battery_voltage=voltage,
                    command_history=np.stack(hist),
                )
                out = controller.step(obs)
                flags = monitor.update(t, obs, out.u, out.state)
                for det_t, cls, joint in monitor.detections[-4:]:
                    if abs(det_t - t) < 1e-9:
                        logger.event(t, "fault_detected", detected_class=cls, joint=joint)
                if out.state != last_state:
                    logger.event(t, "state", frm=last_state, to=out.state)
                    last_state = out.state
                u = out.u
                hist.append(u.copy())
                _, p_total, i_total = servos.power()
                logger.log(
                    _control_row(t, obs, out, p_total, i_total, flags),
                    _truth_row(t, sim, tau),
                )
            if progress and n_steps > 1000 and k % (n_steps // 10) == 0:
                pct = 100 * k // n_steps
                print(f"  [{exp_id}] {pct:3d}%  t={t:6.2f}s  state={last_state}", flush=True)
    except Exception:
        # best-effort cleanup on a failed run; see the note below on why this
        # cannot be a blanket `finally` around the loop.
        backend.close()
        raise

    summary = {
        "duration": float(cfg["duration"]),
        "fault_flags": int(monitor.flags),
        "detections": [
            {"t": t, "class": c, "joint": j} for t, c, j in monitor.detections
        ],
    }
    if hasattr(controller, "offset"):
        summary["dither_offset"] = controller.offset.tolist()
        summary["dither_updates"] = int(controller.update_count)
    if hasattr(controller, "motion_verified"):
        summary["motion_verified"] = bool(controller.motion_verified)
        summary["cycles"] = int(controller.cycle_count)
        summary["aborted_reason"] = controller.aborted_reason
    run_dir = logger.close(summary)
    print(f"  -> {run_dir}")
    # backend.close(), if called at all, must be the last thing this function
    # does. On Isaac, SimulationApp.close() runs with fastShutdown=True
    # (Isaac's own default) and hard-terminates the OS process instead of
    # returning to Python -- a run reported exit 0, the logger's directory
    # existed, and it was empty. `mock`'s close() is a no-op so this never
    # showed up against it; found by running the isaaclab backend for real,
    # see docs/FINDINGS.md.
    #
    # close_backend=False exists for exactly this reason: cli.py's --eval
    # reads run_dir off disk in the *same process* after this call returns,
    # and --repeat reuses the one `ensure_app()` Kit instance across
    # iterations -- either would die here on Isaac if we closed. The process
    # is short-lived either way, so skipping the close and letting the OS
    # reclaim the GPU/Kit resources at exit is the simpler correct choice.
    if close_backend:
        backend.close()
    return run_dir


def _control_row(t, obs: Observation, out, p_total, i_total, flags) -> dict:
    row = {
        "timestamp": t,
        "body_ax": obs.body_accel[0], "body_ay": obs.body_accel[1],
        "body_az": obs.body_accel[2],
        "body_wx": obs.body_gyro[0], "body_wy": obs.body_gyro[1],
        "body_wz": obs.body_gyro[2],
        "battery_voltage": obs.battery_voltage,
        "power_total_W": p_total, "current_total_A": i_total,
        "state": out.state, "fault_flag": int(flags),
    }
    for i, name in enumerate(LEG_NAMES):
        row[f"F_{name}"] = obs.foot_force[i]
        row[f"foot_acc_{name}"] = float(np.linalg.norm(obs.foot_accel[i]))
        row[f"foot_gyro_{name}"] = float(np.linalg.norm(obs.foot_gyro[i]))
    for j in range(N_JOINTS):
        row[f"I_{j}"] = obs.servo_current[j]
        row[f"u_{j}"] = out.u[j]
    row.update(out.extras)
    return row


def _truth_row(t, sim, tau) -> dict:
    row = {"timestamp": t}
    for j, name in enumerate(JOINT_NAMES):
        row[f"q_{name}"] = sim.q[j]
        row[f"qd_{name}"] = sim.qd[j]
        row[f"tau_{name}"] = tau[j]
    row.update(
        body_x=sim.body_pos[0], body_y=sim.body_pos[1], body_z=sim.body_pos[2],
        body_roll=sim.body_rpy[0], body_pitch=sim.body_rpy[1], body_yaw=sim.body_rpy[2],
        body_vz=sim.body_vel[2], body_wx=sim.body_omega[0], body_wy=sim.body_omega[1],
    )
    for i, name in enumerate(LEG_NAMES):
        row[f"foot_z_{name}"] = sim.foot_pos[i, 2]
        row[f"Fz_{name}"] = sim.contact_force[i, 2]
    return row
