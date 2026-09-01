# Isaac Sim / Isaac Lab notes

## The GPU

`memo.txt` section 1 targets an RTX 4070 12 GB.  The documented Isaac Sim 5.1
minimum is an RTX 4080 with 16 GB.  A 4070 is below that line, which is
survivable for this project only because the scene is deliberately cheap:

* **headless** by default (`isaac.headless: true`)
* **no cameras, no LiDAR, no RTX sensors** -- physics and Python only
* **one environment**, not a vectorised batch
* box/capsule geometry, no meshes, no textures

If Isaac refuses to start or the GPU runs out of memory, that ordering is the
thing to protect: keep it headless and single-environment before giving anything
else up.

## Why the servo is ours and not Isaac's

Isaac Lab has `DCMotor` and `DelayedPDActuator`, and they are good models.  This
project deliberately does not use them for the position loop: the whole
experiment is about what a cheap servo does and what its current says, so that
model has to be ours, visible, and swappable for measured values in
`config/servo.yaml` (memo section 49).

The articulation is therefore driven by **effort**, with Isaac's implicit drive
set to `stiffness = 0, damping = 0`, and `reflex_quad/servo_model.py` is the only
position loop in the system.

## Timestep

memo section 12: physics 500 Hz, control 100 Hz.  Isaac Lab's IMU differentiates
numerically and its documentation recommends at least 200 Hz for that, so 500 Hz
is comfortable.  Move to 1000 Hz (`physics_dt: 0.001`) only if contact chatters;
it roughly doubles the wall-clock cost.

## Joint order

Isaac sorts joints its own way and that order is not guaranteed stable across
versions.  `IsaacLabBackend._build_joint_index()` maps Isaac's `joint_names` onto
this project's fixed order (`FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee,
RR_hip, RR_knee`) and raises if a joint is missing.  Never index Isaac's arrays
directly.

## URDF, not USD

`reflex_quad/asset_builder.py` generates `assets/reflex_quad.urdf` from
`config/robot.yaml`, and the backend converts it to USD with Isaac Lab's
`UrdfConverter` on first use.  URDF is plain text, reviewable in a diff and
independent of the USD schema version; a hand-written USD would have to track
Isaac's schema.

`merge_fixed_joints` is **off** on purpose: the foot links must survive the
conversion, because the contact sensors and foot IMUs attach to them.

## The `# VERIFY` markers

`reflex_quad/backends/isaaclab.py` marks every assumption it makes about an
Isaac Lab attribute name:

* `robot.joint_names`, `robot.body_names`
* `robot.set_joint_effort_target(...)`, `write_data_to_sim()`, `update(dt)`
* `robot.data.joint_pos / joint_vel / root_pos_w / root_quat_w / body_pos_w`
* `Imu.data.lin_acc_b`, `Imu.data.ang_vel_b`
* `ContactSensor.data.net_forces_w`
* `UrdfConverterCfg` field names

`scripts/isaac_preflight.py` checks each one separately and prints which failed.
Run it first (runbooks/RB-03), fix what moved, and only then start experiments.
If a name has changed, fix it in the backend and note the version in the
experiment note -- do not work around it in the controller.

## Things not to add yet

memo section 1 is explicit, and it is right: no cameras, no LiDAR, no RL, no
parallel environments, no ROS 2 until phase 1 passes.  Every one of them changes
what a failure means.
