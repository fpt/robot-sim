# The mock backend

`reflex_quad/backends/mock.py` is a reduced-order physics model with no
dependencies beyond numpy.  It exists so that the servo model, sensor models,
observer, objective, dither search, state machine, fault monitor, logging and
the entire evaluation pipeline can be written, run and regression-tested without
a GPU.  Swapping to Isaac changes the backend and nothing else.

## What it models

| | |
|---|---|
| body | heave `z`, roll, pitch -- 3 DOF, small angle |
| joints | 8 second-order systems: `I qdd = tau_servo + tau_load - b qd`, at joint limits |
| contact | per foot, unilateral spring-damper against a height field (4000 N/m, 120 Ns/m) |
| moment arms | roll from the **hip** y (fixed), pitch from the **foot** x (moves with the hip joint) |
| IMU | specific force by numerical differentiation of foot/body motion, rotated into the body frame |
| integration | semi-implicit Euler, 4 substeps per 2 ms physics tick |

Static sag under 1.4 kg is 0.9 mm per foot; the vertical mode is ~107 rad/s,
comfortably inside the 0.5 ms substep.

## What it does NOT model

Read this list before trusting any result from it.

* **Horizontal body motion and yaw.** The body's x, y and yaw are pinned.  So:
  a step does not translate the robot, weight cannot shift by moving the CoM,
  and nothing can tip over sideways.
* **Friction cones and slipping.** Contact is vertical only; a foot never
  slides.  `unstable_contact` therefore cannot be produced honestly here.
* **Link inertia coupling.** Legs contribute mass and inertia to the body but
  their own dynamics are not simulated as a chain.
* **Collisions** of any kind other than foot-to-ground.
* **Foot geometry.** A foot is a point, not a sphere with a contact patch.

## What that means for the experiments

| experiment | mock verdict means | still needs Isaac for |
|---|---|---|
| 01 stand | the plumbing and the servo model are sound | real contact, real inertia |
| 02 uneven | the section 26 + twist control law converges | tipping, slipping |
| 03 dither | **the active-sensing loop works** -- the central claim | whether it works against real contact noise |
| 04 unload | the state machine sequences correctly and the IMU check is sound | whether the robot actually stays up on three legs |
| 05 fault | the residual monitor separates the cases it claims to | realistic current/vibration structure |
| 06 step | the cycle completes with a forward foot offset | **the body actually moving** -- impossible here |

A green mock run means the control logic and the plumbing are right.  It does
not mean the robot stands up in Isaac Sim.  That is the point of the split, and
it is why every runbook re-checks its criteria on the Isaac backend.
