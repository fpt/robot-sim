"""Isaac Sim 5.1 / Isaac Lab backend.

This is the backend the project actually cares about; `mock` exists so that
everything around it can be built and tested without a GPU.  The two are
interchangeable: same `SimBackend` interface, same servo model, same sensors,
same controller, same logs.

Written against the Isaac Lab 2.x API that ships with Isaac Sim 5.1.  Every
assumption about an attribute name is marked `# VERIFY` and is checked one at a
time by `scripts/isaac_preflight.py`, which is step 4 of runbooks/RB-03.  Run
that before the first experiment: it fails fast and tells you which name moved,
instead of a run dying twenty minutes in.

Design notes:
  * The articulation is driven by *effort*, not position.  The servo model in
    reflex_quad/servo_model.py is the position loop, so Isaac's own drive is
    switched off (stiffness = damping = 0).  Using Isaac's implicit PD instead
    would mean the servo we are trying to study is the one we cannot see.
  * Joint order is reindexed to reflex_quad's fixed FL/FR/RL/RR x hip/knee
    order; Isaac sorts joints its own way and that order is not stable.
  * One environment, no cameras, no RTX sensors: an RTX 4070 12 GB is below the
    documented Isaac Sim minimum (RTX 4080 16 GB), so the scene stays cheap.
"""
from __future__ import annotations

import numpy as np

from .. import JOINT_NAMES, LEG_NAMES, N_JOINTS
from ..isaac_boot import ensure_app
from ..robot import LegGeometry
from .base import SimState

GRAVITY = 9.81


class IsaacLabBackend:
    def __init__(self, cfg: dict, rng: np.random.Generator):
        self.cfg = cfg
        self.dt = float(cfg["physics_dt"])
        self.geom = LegGeometry(cfg["robot"])
        isaac_cfg = cfg.get("isaac", {})
        ensure_app(headless=bool(isaac_cfg.get("headless", True)))

        # imports must come after ensure_app()
        import isaaclab.sim as sim_utils
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import Articulation, ArticulationCfg
        from isaaclab.sensors import ContactSensor, ContactSensorCfg, Imu, ImuCfg
        from isaaclab.sim import SimulationCfg, SimulationContext

        self._sim_utils = sim_utils
        usd_path = self._ensure_usd(isaac_cfg)

        self.sim = SimulationContext(
            SimulationCfg(dt=self.dt, device=isaac_cfg.get("device", "cuda:0"))
        )
        self._build_scene(sim_utils, isaac_cfg)

        robot_cfg = ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=usd_path,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False, max_depenetration_velocity=1.0
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False, solver_position_iteration_count=8,
                    solver_velocity_iteration_count=1,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, float(cfg["robot"]["stance"]["height"]) + 0.02),
                joint_pos=self._initial_joint_pos(),
            ),
            actuators={
                # stiffness/damping zero: our own servo model provides the loop
                "all": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip", ".*_knee"],
                    effort_limit=float(cfg["servo"]["limits"]["tau_max"]) * 3.0,
                    velocity_limit=float(cfg["servo"]["limits"]["qd_max"]) * 3.0,
                    stiffness=0.0, damping=0.0,
                ),
            },
        )
        self.robot = Articulation(robot_cfg)

        self.body_imu = Imu(ImuCfg(prim_path="/World/Robot/body", update_period=0.0))
        self.foot_imus = [
            Imu(ImuCfg(prim_path=f"/World/Robot/{n}_foot", update_period=0.0))
            for n in LEG_NAMES
        ]
        self.foot_contacts = [
            ContactSensor(ContactSensorCfg(
                prim_path=f"/World/Robot/{n}_foot", update_period=0.0, history_length=0,
            ))
            for n in LEG_NAMES
        ]

        self.sim.reset()
        self._joint_index = self._build_joint_index()
        self.t = 0.0
        self._last_state: SimState | None = None

    # -- setup helpers ---------------------------------------------------
    def _ensure_usd(self, isaac_cfg: dict) -> str:
        """Convert assets/reflex_quad.urdf to USD (cached on disk)."""
        from pathlib import Path

        from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

        from ..asset_builder import write_urdf

        urdf = Path(isaac_cfg.get("urdf", "assets/reflex_quad.urdf"))
        if not urdf.exists():
            urdf = write_urdf(urdf)
        out_dir = Path(isaac_cfg.get("usd_dir", "assets/usd"))
        out_dir.mkdir(parents=True, exist_ok=True)
        converter = UrdfConverter(UrdfConverterCfg(   # VERIFY: cfg field names
            asset_path=str(urdf.resolve()),
            usd_dir=str(out_dir.resolve()),
            usd_file_name="reflex_quad.usd",
            fix_base=False,
            merge_fixed_joints=False,      # keep the foot links: sensors attach there
            force_usd_conversion=bool(isaac_cfg.get("force_convert", False)),
        ))
        return converter.usd_path

    def _build_scene(self, sim_utils, isaac_cfg: dict) -> None:
        sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
        light = sim_utils.DomeLightCfg(intensity=2000.0)
        light.func("/World/light", light)
        for i, block in enumerate(self.cfg["terrain"].get("blocks", []) or []):
            h = float(block["height"])
            cube = sim_utils.CuboidCfg(
                size=(float(block["size_x"]), float(block["size_y"]), h),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                rigid_props=None,          # static
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.4, 0.2)),
            )
            cube.func(f"/World/block_{i}", cube,
                      translation=(float(block["x"]), float(block["y"]), h / 2))

    def _initial_joint_pos(self) -> dict[str, float]:
        u0 = self.geom.nominal_command()
        return {name: float(u0[i]) for i, name in enumerate(JOINT_NAMES)}

    def _build_joint_index(self) -> np.ndarray:
        """Map our fixed joint order onto Isaac's."""
        names = list(self.robot.joint_names)             # VERIFY
        missing = [n for n in JOINT_NAMES if n not in names]
        if missing:
            raise RuntimeError(
                f"articulation is missing joints {missing}; it has {names}"
            )
        return np.array([names.index(n) for n in JOINT_NAMES], dtype=int)

    # -- SimBackend ------------------------------------------------------
    def reset(self) -> SimState:
        self.robot.reset()
        self.sim.reset()
        self.t = 0.0
        return self.state()

    def step(self, tau: np.ndarray) -> SimState:
        import torch

        efforts = torch.zeros((1, N_JOINTS), device=self.robot.device)
        efforts[0, self._joint_index] = torch.as_tensor(
            np.asarray(tau, dtype=np.float32), device=self.robot.device
        )
        self.robot.set_joint_effort_target(efforts)       # VERIFY
        self.robot.write_data_to_sim()
        self.sim.step()
        self.robot.update(self.dt)
        self.body_imu.update(self.dt)
        for s in self.foot_imus + self.foot_contacts:
            s.update(self.dt)
        self.t += self.dt
        return self.state()

    def state(self) -> SimState:
        d = self.robot.data
        idx = self._joint_index
        q = d.joint_pos[0].cpu().numpy()[idx]
        qd = d.joint_vel[0].cpu().numpy()[idx]
        pos = d.root_pos_w[0].cpu().numpy()
        quat = d.root_quat_w[0].cpu().numpy()             # (w, x, y, z)
        rpy = _quat_to_rpy(quat)
        vel = d.root_lin_vel_w[0].cpu().numpy()
        omega = d.root_ang_vel_b[0].cpu().numpy()

        body_acc = self.body_imu.data.lin_acc_b[0].cpu().numpy()      # VERIFY
        body_gyro = self.body_imu.data.ang_vel_b[0].cpu().numpy()
        foot_acc = np.stack([s.data.lin_acc_b[0].cpu().numpy() for s in self.foot_imus])
        foot_gyro = np.stack([s.data.ang_vel_b[0].cpu().numpy() for s in self.foot_imus])
        contact = np.stack([
            s.data.net_forces_w[0, 0].cpu().numpy() for s in self.foot_contacts  # VERIFY
        ])
        foot_pos = np.stack([
            self._body_pos(f"{n}_foot") for n in LEG_NAMES
        ])
        return SimState(
            t=self.t, q=q, qd=qd, body_pos=pos, body_rpy=rpy, body_vel=vel,
            body_omega=body_gyro if omega is None else omega,
            body_accel_body=body_acc, foot_pos=foot_pos,
            foot_accel_body=foot_acc, foot_omega=foot_gyro, contact_force=contact,
        )

    def _body_pos(self, name: str) -> np.ndarray:
        names = list(self.robot.body_names)               # VERIFY
        i = names.index(name)
        return self.robot.data.body_pos_w[0, i].cpu().numpy()

    def terrain_height(self, x: float, y: float) -> float:
        h = 0.0
        for b in self.cfg["terrain"].get("blocks", []) or []:
            if (abs(x - float(b["x"])) <= float(b["size_x"]) / 2
                    and abs(y - float(b["y"])) <= float(b["size_y"]) / 2):
                h = max(h, float(b["height"]))
        return h

    def close(self) -> None:
        from ..isaac_boot import close_app

        close_app()


def _quat_to_rpy(q: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sp, -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])
