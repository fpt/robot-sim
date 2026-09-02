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

## RB-01 on Ubuntu 26.04 (2026-09-02, driver 610.43.02, CUDA 13.3)

The `~/robotics` install on this host is a first case of the "safe ground is
22.04/24.04" warning above being wrong for the actual machine.  Two problems
so far, one fixed and one open.

**Fixed -- missing shared libraries.**  Isaac Sim 5.1's bundled native
extensions (`omni.kit.asset_converter`, `isaacsim.asset.importer.urdf`,
`isaacsim.asset.importer.mjcf`, the iray/MDL material system) are linked
against `libGLU.so.1` and `libxml2.so.2`.  26.04 ships neither: `libglu1-mesa`
is not installed by default (fixed with `sudo apt install libglu1-mesa`), and
26.04's `libxml2` package was renamed `libxml2-16` with SONAME bumped from 2
to 16 -- there is no compat package in the 26.04 repos at all. Fix: pulled
`libxml2_2.9.14+dfsg-1.3ubuntu3.8_amd64.deb` (24.04/noble-security) and
extracted just `libxml2.so.2.9.14` into `~/robotics/compat-libs/`, plus its
own `libicuuc.so.74` / `libicudata.so.74` dependency (already present on disk,
bundled inside `isaacsim/extscache/omni.kit.converter.hoops_core-*/bin/`, just
not on the loader's path). `liblzma.so.5` and `libz.so.1` needed no fix --
26.04's sonames for those happen to still match. Any Isaac launch on this host
needs `LD_LIBRARY_PATH=~/robotics/compat-libs` set first, or the same three
library-not-found errors come back and the URDF-to-USD conversion step in
`IsaacLabBackend._ensure_usd` cannot run (it depends on
`isaacsim.asset.importer.urdf`, which is one of the extensions that failed to
load). Do **not** apt-install a same-named `libxml2` package as a "real" fix --
26.04 does not offer the old SONAME and forcing one in would fight the system
package Isaac Lab's own extensions were not tested against Isaac Sim's now
either way; keeping the compat copy isolated in `~/robotics/compat-libs` and
only on `LD_LIBRARY_PATH` for Isaac launches avoids touching the system's own
`libxml2-16` that everything else on the host links against.

**Open -- RTX renderer segfaults on startup, even headless.**  With the
library fix above, `isaacsim.SimulationApp({"headless": True})` gets further
(extensions load cleanly, MDL/iray load fine) but crashes ~150ms into startup
inside NVIDIA's own plugin: `librtx.scenedb.plugin.so!carbOnPluginStartup`,
a `std::vector` reallocation, SIGSEGV, no dump uploaded (`UploadSuccessful =
'0'`). `vulkaninfo --summary` on the same driver enumerates the 4070 cleanly,
so the Vulkan ICD itself is not the problem -- this is inside Isaac's RTX
render plugin specifically. `dmesg` shows no Xid or GPU fault around the crash
time, so the GPU/kernel driver pairing is not resetting; this looks like an
Isaac Sim / driver version mismatch, not a hardware problem. `SimulationApp`
only exposes two renderer choices, `RaytracedLighting` and `PathTracing`
(`isaacsim.simulation_app.simulation_app.SimulationApp.DEFAULT_LAUNCHER_CONFIG`),
both routed through the same crashing RTX plugin -- there is no
non-RTX/software Hydra fallback to switch to from inside Isaac Sim, even
though this project never enables cameras or RTX sensors (section 1); the
base app experience (`isaacsim.exp.base.kit`) still loads `omni.hydra.rtx`.
Driver 610.43.02 (built 2026-05-19) is far newer than what Isaac Sim 5.1 would
have been validated against; the host has 535/550/560/570/575/580/590/595
metapackages available in `apt-cache` as older alternatives, untested. A
driver change is a machine-wide, likely-reboot change (this host's desktop
session runs on the same GPU) and was not made without asking first --
`scripts/isaac_preflight.py` and `RB-01`/`RB-02` are blocked on this until it
is resolved one way or another; note the resolution here and in the run's
experiment note once it lands.

**Confirmed and root-caused, 2026-09-02.** This is a known upstream bug, not
anything specific to this host or the 26.04 OS mismatch:

* Reproduced the identical crash (same stack, same symbol offsets --
  `_M_realloc_insert+0x123ef` etc.) inside NVIDIA's own official
  `nvcr.io/nvidia/isaac-sim:5.1.0` container (Ubuntu 24.04 base, the
  documented-supported OS), on this same driver, via
  `docker run --runtime=nvidia --gpus all`.  A container cannot route around
  this: `nvidia-container-toolkit` bind-mounts the *host's* driver libraries
  in (`nvidia-smi` inside the container reports the same 610.43.02); the
  kernel module and userspace driver version must match, so the container
  always runs whatever driver the host kernel module is.  This rules out the
  26.04 OS/library issues above as the cause of the RTX crash specifically
  (they were real and are fixed, but independent).
* NVIDIA driver 610.x (and the 590/595 branches) are a known regression
  against Isaac Sim 5.1's RTX renderer -- multiple reports on RTX
  4070/4090/5080/5090, Linux and Windows:
  [isaac-sim/IsaacSim#651](https://github.com/isaac-sim/IsaacSim/issues/651),
  [#650](https://github.com/isaac-sim/IsaacSim/issues/650),
  [Discussion #648](https://github.com/isaac-sim/IsaacSim/discussions/648).
  No software workaround is reported to work (people have tried building from
  source, disabling IOMMU, other Isaac Sim point releases -- same crash every
  time). Isaac Sim 5.1's validated Linux driver is **580.65.06** (or
  580.159.03).
* Net effect: on this host, running Isaac Sim 5.1 needs the driver downgraded
  to the 580 branch.  There is no way to keep 610 active for other GPU work
  on this machine and run Isaac Sim at the same time -- only one driver
  version can be the loaded kernel module.  If this host needs to serve both
  a 610-driver workload and Isaac Sim, that is a scheduling/dual-boot/second-
  machine decision, not something fixable in this repository.

**Resolved -- moved to Isaac Sim 6.0.1.0, driver untouched, 2026-09-02.** This
host also runs `../rs-gallium`, which needs the current 610 driver, so
downgrading was off the table.  Isaac Lab's own `main` branch (cloned at
`bffdce9`, `VERSION` file reads `3.0.0`) already targets
`isaacsim[all,extscache]==6.0.1.0`
(`tools/wheel_builder/res/python_packages.toml`), and multiple reports
(e.g. [IsaacSim#689](https://github.com/isaac-sim/IsaacSim/discussions/689))
say Isaac Sim 6.0 / Isaac Lab 3.0.0-beta2 runs cleanly on driver 610.x.
Confirmed on this host: `isaacsim[all,extscache]==6.0.1.0` +
`LD_LIBRARY_PATH=~/robotics/compat-libs` (the libxml2/libGLU fix above is
still needed -- that one is 26.04-specific, independent of the Isaac Sim
version) gets a full `Simulation App Startup Complete` / `app ready`, Warp
reports the RTX 4070 with CUDA available, clean shutdown.  No RTX crash.

Two things changed for this project's runbooks by moving to 6.0.1.0, both
because Isaac Lab's `main` (3.0.0) pins them, not by choice here:

* **Python 3.12, not 3.11.**  `isaacsim==6.0.1.0` ships no `cp311` wheels
  (uv's resolver fails outright with "no wheels with a matching Python
  implementation tag" on a 3.11 venv). `env_isaaclab` is now
  `uv venv --python 3.12 --seed`. RB-01 step 3 needs updating; the mock-only
  `.venv` in the repo stays 3.11 per `pyproject.toml`'s `requires-python`,
  since the mock backend has no Isaac dependency to force the bump.
* **No separate CUDA torch install.**  `isaacsim[all]==6.0.1.0` already
  depends on a CUDA-13-built torch (`2.11.0+cu130`) that reports
  `torch.cuda.is_available() == True` against driver 610 out of the box.
  RB-01 step 5 (`pip install torch --index-url .../cu128`) is not needed on
  this version and was skipped; installing an older cu128 wheel over it would
  be a downgrade, not a fix, so do not run that step when installing
  `isaacsim>=6.0`.

Still open: `reflex_quad/backends/isaaclab.py`'s docstring and `# VERIFY`
markers were written against "Isaac Lab 2.x API that ships with Isaac Sim
5.1" -- Isaac Lab jumped a major version (2.x -> 3.0.0-beta) to pair with
Isaac Sim 6.0, so those assumptions are unconfirmed against the new API.
`scripts/isaac_preflight.py` is what checks them; run it before trusting the
backend, and expect to fix at least the version comment even if every name
still resolves.
