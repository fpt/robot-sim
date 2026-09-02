# The mock backend

`reflex_quad/backends/mock.py` is a reduced-order physics model with no
dependencies beyond numpy.  It exists so that the servo model, sensor models,
observer, objective, dither search, state machine, fault monitor, logging and
the entire evaluation pipeline can be written, run and regression-tested without
a GPU.  Swapping to Isaac changes the backend and nothing else.

## What it models

| | |
|---|---|
| body | full 6-DOF: `x, y, z, roll, pitch, yaw` and their rates -- proper trig, not small-angle |
| joints | 8 second-order systems: `I qdd = tau_servo + tau_load - b qd`, at joint limits |
| contact, vertical | per foot, unilateral spring-damper against a height field (4000 N/m, 120 Ns/m) |
| contact, horizontal | viscous friction against foot slip velocity, Coulomb-capped at `mu * F_normal` (`terrain.friction`, default 0.7, unmeasured -- see below) |
| body rotation | sum of `r x F` per foot about the body CoM, in world frame, rotated into the body's own (diagonal) inertia frame |
| IMU | specific force by numerical differentiation of foot/body motion, rotated into the body frame |
| integration | semi-implicit Euler, 4 substeps per 2 ms physics tick |

Static sag under 1.4 kg is 0.9 mm per foot; the vertical mode is ~107 rad/s,
comfortably inside the 0.5 ms substep.

This is Layer 2 of `docs/reflex_quad_12dof_trot_plan.md` ("mock 拡張"): before
it, mock modelled only heave/roll/pitch, small-angle, with no horizontal
motion, no yaw, and no friction at all -- none of which the 12DOF morphology's
premise (yaw-driven propulsion, roll recovering a lateral tip, a
support-diagonal inverted-pendulum tip) could even be checked against.  The
robot itself is still the current 8-joint one; JOINT_NAMES and
`config/robot.yaml` are unchanged.  A moving-point-on-a-rotating-body
kinematic chain (`_foot_kinematics`) and full cross-product moment sum
replaced the old per-axis lever-arm formulas -- verified to reduce to those
exact formulas in the small-angle, no-friction, flat-stance limit before
trusting anything downstream of it.

## What it does NOT model

Read this list before trusting any result from it.

* **True stick-slip friction.** The horizontal contact force is viscous
  (proportional to slip speed) with a Coulomb cap, not a real static/dynamic
  friction model -- there is no "stuck" regime with zero slip under a
  sustained sub-limit force.  This likely *overestimates* how much a real
  foot pad would slide.  A stateful anchor/bristle model is the natural next
  refinement if a trot experiment's slip looks implausible; not built yet.
* **The friction coefficient is not measured.** `terrain.friction` (default
  0.7) is a generic rubber-pad placeholder.  Per the plan's own trust rule,
  the geometry and structure of what friction enables (propulsion without
  slip, a recoverable tip) are meant to be trusted here; the coefficient's
  actual value is not, until
  `docs/reflex_quad_sts3215_isaac_eval_plan.md` section 5.3's bench
  measurement or an Isaac sensitivity sweep replaces it.
* **Euler-rate = body-rate.** `roll_rate/pitch_rate/yaw_rate` integrate
  directly into `roll/pitch/yaw`, exact for a single-axis rotation and an
  approximation for a compound one.  Fine for the tilts this project's
  recovery envelopes (`theta_max`, low tens of degrees) describe; degrades
  approaching true tumbling.
* **Link inertia coupling.** Legs contribute mass and inertia to the body but
  their own dynamics are not simulated as a chain.
* **Collisions** of any kind other than foot-to-ground.
* **Foot geometry.** A foot is a point, not a sphere with a contact patch --
  this is also why `IsaacLabBackend`'s spawn height needs a correction mock
  has no equivalent for (docs/FINDINGS.md #15): mock's `reset()` places the
  foot *point* exactly on the ground, with nothing to say where a real
  10 mm-radius foot's surface would be relative to that.
* **Gyroscopic and off-diagonal inertia coupling.** Body rotation uses
  `alpha = M / diag(ixx, iyy, izz)`, no `omega x I*omega` term and no
  products of inertia -- fine at the moderate spin rates every experiment
  here produces, not a general rigid-body simulator.

## What that means for the experiments

| experiment | mock verdict means | still needs Isaac for |
|---|---|---|
| 01 stand | the plumbing and the servo model are sound | real contact, real inertia |
| 02 uneven | the section 26 + twist control law converges | tipping, slipping |
| 03 dither | **the active-sensing loop works** -- the central claim | whether it works against real contact noise |
| 04 unload | the state machine sequences correctly and the IMU check is sound | whether the robot actually stays up on three legs |
| 05 fault | the residual monitor separates the cases it claims to | realistic current/vibration structure |
| 06 step | the cycle completes with a forward foot offset, and the body's own x/y now moves under it | real ground friction and slip, since mock's own is unmeasured |

A green mock run means the control logic and the plumbing are right.  It does
not mean the robot stands up in Isaac Sim.  That is the point of the split, and
it is why every runbook re-checks its criteria on the Isaac backend.

**A green mock run before Layer 2 could also mean the physics was not rich
enough to show the problem.** `04_leg_unload` and `03_dither` both regressed
against their existing `config/criteria.yaml` thresholds once mock could
represent the real coupling a single-leg lift or an uncorrected disturbance
produces (docs/FINDINGS.md #17) -- not because Layer 2 broke anything, but
because the pre-Layer-2 model structurally could not show it failing.  Read
that finding before assuming a newly-red check is a regression to chase; it
may be the mock catching up to what Isaac would already have shown.
