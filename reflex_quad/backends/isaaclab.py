"""Isaac Sim 6.0 / Isaac Lab backend.

This is the backend the project actually cares about; `mock` exists so that
everything around it can be built and tested without a GPU.  The two are
interchangeable: same `SimBackend` interface, same servo model, same sensors,
same controller, same logs.

Originally written against the Isaac Lab 2.x API that ships with Isaac Sim
5.1; moved to Isaac Lab 3.0.0 (main) / Isaac Sim 6.0.1.0 on 2026-09-02 because
Isaac Sim 5.1's RTX renderer crashes outright on NVIDIA driver 610.x, which
this project's GPU also needs for other work -- see docs/ISAAC_NOTES.md for
the root-cause chase. `scripts/isaac_preflight.py`'s scene check caught one
real API-surface change from that jump (the URDF importer's link layout,
fixed below); every remaining assumption about an attribute name is marked
`# VERIFY` and is checked one at a time by that script, which is step 4 of
runbooks/RB-03.  Run it before the first experiment: it fails fast and tells
you which name moved, instead of a run dying twenty minutes in.

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
                # Not `activate_contact_sensors=True` here: that sweep
                # (isaaclab.sim.schemas.activate_contact_sensors) walks the prim
                # tree and stops descending the instant it finds a RigidBodyAPI,
                # on the assumption that rigid bodies never nest.  URDF importer
                # 3.0's link-per-USD-parent layout (see the comment below) breaks
                # that assumption: it stops at "body" -- itself a rigid body --
                # and never reaches FL_upper/FL_lower/FL_foot underneath it, so
                # every ContactSensor still fails "no bodies with contact
                # reporter API".  Activated explicitly per foot below instead,
                # after `Articulation(robot_cfg)` has spawned the prims.
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False, solver_position_iteration_count=8,
                    solver_velocity_iteration_count=1,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, self._spawn_height()),  # see _spawn_height()
                joint_pos=self._initial_joint_pos(),
            ),
            actuators={
                # stiffness/damping zero: our own servo model provides the loop
                "all": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip", ".*_knee"],
                    # *_limit_sim, not *_limit: Isaac Lab 3.0 deprecated the
                    # unsuffixed fields for implicit actuators (still accepted,
                    # but warns on every run) in favor of these.
                    effort_limit_sim=float(cfg["servo"]["limits"]["tau_max"]) * 3.0,
                    velocity_limit_sim=float(cfg["servo"]["limits"]["qd_max"]) * 3.0,
                    stiffness=0.0, damping=0.0,
                ),
            },
        )
        self.robot = Articulation(robot_cfg)

        # URDF importer 3.0 (Isaac Sim 6.0 / Isaac Lab 3.0) runs an asset-transformer
        # pass by default that re-authors the converted USD into a layered asset:
        # links move under a "Geometry" scope, nested by the URDF's own kinematic
        # tree, instead of sitting flat under the articulation root as they did on
        # Isaac Sim 5.1.  `scripts/isaac_preflight.py`'s scene check caught this --
        # a sensor prim_path of "/World/Robot/{n}_foot" no longer resolves.  See
        # docs/ISAAC_NOTES.md.  Feet are nested body/{n}_upper/{n}_lower/{n}_foot
        # because that is the URDF's own link chain (asset_builder.py); this is
        # derived from our URDF, not a guess at Isaac's naming.
        from isaaclab.sim.schemas import activate_contact_sensors

        for n in LEG_NAMES:
            activate_contact_sensors(self._foot_prim_path(n))

        self.body_imu = Imu(ImuCfg(prim_path="/World/Robot/Geometry/body", update_period=0.0))
        self.foot_imus = [
            Imu(ImuCfg(prim_path=self._foot_prim_path(n), update_period=0.0))
            for n in LEG_NAMES
        ]
        self.foot_contacts = [
            ContactSensor(ContactSensorCfg(
                prim_path=self._foot_prim_path(n), update_period=0.0, history_length=0,
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
            # no usd_file_name: URDF importer 3.0's UrdfConverter.__init__ always
            # overrides it to "{urdf_stem}/{urdf_stem}.usda" and ignores what is
            # passed here -- setting it was a silent no-op on Isaac Lab 3.0.
            fix_base=False,
            merge_fixed_joints=False,      # keep the foot links: sensors attach there
            force_usd_conversion=bool(isaac_cfg.get("force_convert", False)),
        ))
        return converter.usd_path

    @staticmethod
    def _foot_prim_path(leg: str) -> str:
        """Path to a foot link under the articulation root.

        URDF importer 3.0 nests links by the URDF's own kinematic chain under a
        "Geometry" scope (see the comment in __init__) rather than laying them
        flat; each leg's foot is body/{leg}_upper/{leg}_lower/{leg}_foot because
        that is the parent chain asset_builder.py writes into the URDF.
        """
        return f"/World/Robot/Geometry/body/{leg}_upper/{leg}_lower/{leg}_foot"

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

    def _spawn_height(self) -> float:
        """Body z at spawn: clear of the tallest terrain under any nominal foot.

        Mirrors MockBackend.reset()'s convention (`z = max(ground + ext) +
        margin`) rather than mock's own fixed 0.02 m constant, which assumed
        flat ground and put the feet 15 mm into a 20 mm terrain block on
        02_uneven_ground -- caught by runner.py's _assert_sane_initial_state,
        see docs/FINDINGS.md #15.  `+ body.height / 2` accounts for the URDF
        mounting each hip that far below the body link's own origin
        (asset_builder.py), which the reduced-order mock model has no term
        for at all (hip_z == body_z there).
        """
        q0, q1 = self.geom.nominal_command()[0:2]  # same angles on all 4 legs
        ext = self.geom.leg_extension(q0, q1)
        fx, _ = self.geom.foot_offset(q0, q1)
        ground = max(
            self.terrain_height(hx + fx, hy)
            for hx, hy in self.geom.hip_xy
        )
        return ground + ext + float(self.cfg["robot"]["body"]["height"]) / 2.0 + 0.005

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
        # Articulation.reset() only resets actuator-internal state (delay
        # queues etc.) -- it does NOT re-apply init_state.  Isaac Lab's own
        # convention is to write the default root/joint state back to sim
        # explicitly after sim.reset(); skipping this left the articulation
        # at its raw USD rest pose (all-zero joint angles) instead of
        # nominal_command(), a ~4 cm foot penetration at spawn that showed up
        # as a violent depenetration launch and permanent tumble -- see
        # docs/FINDINGS.md #15.
        self.sim.reset()
        self.robot.write_root_pose_to_sim_index(root_pose=self.robot.data.default_root_pose)
        self.robot.write_root_velocity_to_sim_index(root_velocity=self.robot.data.default_root_vel)
        self.robot.write_joint_position_to_sim_index(position=self.robot.data.default_joint_pos)
        self.robot.write_joint_velocity_to_sim_index(velocity=self.robot.data.default_joint_vel)
        self.robot.reset()
        self.t = 0.0
        # Isaac Lab 3.0's Imu sets self._dt only inside update(dt); state() below
        # reads .data on each IMU, which lazily recomputes on first access with
        # no update() ever having run yet, raising AttributeError on _dt.  step()
        # always calls update() before state(), so this only bites the very
        # first frame -- prime it here the same way.
        for s in [self.body_imu] + self.foot_imus + self.foot_contacts:
            s.update(self.dt)
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
