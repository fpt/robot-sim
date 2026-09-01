# Specification

The source requirements, by section number, so that every `memo.txt section N`
reference in this repository resolves.  Condensed from the original design memo
and the discussion that produced it.  Where the implementation departs from a
section, the reason is in `docs/FINDINGS.md` and is linked here.

---

## Purpose (section 0)

A quadruped whose controller is **not** given joint angle `q`, joint velocity
`qd`, joint acceleration or joint torque `tau`.  The simulator keeps them for
physics; the control program never sees them.

The controller may use exactly:

1. servo position commands `u[8]`
2. one body IMU
3. four foot IMUs
4. four foot force sensors
5. eight servo currents
6. supply voltage

The first goal is **not walking**.  It is: stand on unknown uneven ground and,
by small servo movements that explore the robot's own body and the floor,
converge to a level body and a sensible load distribution.

## Machine and scope (section 1)

Ubuntu, RTX 4070 12 GB, 128 GB RAM, Python 3.11, Isaac Sim 5.1, Isaac Lab, CUDA
PyTorch.  Deliberately **not** used at first: cameras, LiDAR, RTX sensors,
reinforcement learning, parallel environments, ROS 2.  Physics and Python only.

## Environment set-up (sections 2-8)

2. Record OS, kernel, driver, GPU, VRAM, RAM before anything else.
3. Base packages, a working directory.
4. Python 3.11 virtual environment.
5. `isaacsim[all,extscache]==5.1.0` from `pypi.nvidia.com`; torch 2.7.0 + cu128.
   **If `torch.cuda.is_available()` is False, stop and fix the GPU environment.**
6. Launch Isaac Sim once on its own.
7. Clone and install Isaac Lab, stock configuration.
8. Run one bundled sample; record the state that works.

-> `runbooks/RB-00` .. `RB-02`.

## Project layout (section 9)

A separate repository that imports Isaac Lab rather than modifying it, with
`config/`, `assets/`, the package itself, `experiments/` and `logs/`.

## Robot (sections 10, 11)

Body 300 x 180 x 60 mm, 1.0 kg.  Four legs, two links each, 120 + 120 mm.  One
degree of freedom per joint: **hip and knee move in the same sagittal plane**,
no lateral freedom.  Eight actuators: FL/FR/RL/RR x hip/knee.

The constraint is deliberate: the first question is whether changing *leg length*
can control body attitude.

-> `config/robot.yaml`, `reflex_quad/robot.py`.  FINDINGS #13 widened the hip
limit to +-80 deg.

## Rates (section 12)

Physics 500 Hz (`dt = 0.002`), sensors 500 Hz, servo model 500 Hz, controller
100 Hz.  Raise physics to 1 kHz later if needed.

## The rule (sections 13, 14)

The simulator holds `q_true`, `qd_true`, `tau_true`, true body and foot poses and
true contact forces.  **The controller may not read any of them.**  The
observation is exactly: body IMU, four foot IMUs, four foot forces, eight servo
currents, battery voltage, and the history of its own commands.  Joint angle is
not in the list.

-> `reflex_quad/types.py`, `eval/isolation.py`,
`tests/test_observation_isolation.py`.

## Body IMU (section 15)

Accelerometer and gyro.  An internal estimator produces roll, pitch, roll rate,
pitch rate.  The simulator's true orientation is never handed over.

-> `reflex_quad/observer.py`.

## Foot IMUs (section 16)

One per foot: linear acceleration and angular velocity.  Used to answer: did the
foot actually move, in which direction, did it hit something, is it vibrating, is
it slipping, did it respond to the command.

-> FINDINGS #8: the reliable signal is the **integrated gyro**, not acceleration.

## Foot force (sections 17, 18)

Contact force from the simulator, reduced to what a cheap FSR would report:
`F = clamp(Fz, 0, Fmax)` with `Fmax = 9.8 N`, roughly: 0 N no contact, 0.3 N weak
contact, 2-7 N normal, 9.8 N saturated.  Non-linearity, hysteresis, noise,
per-sensor gain differences and zero drift come later.  The controller never
gets the full force vector.

-> `reflex_quad/sensors.py`, `config/sensors.yaml`.

## Servo model (sections 19, 20)

`tau = Kp (u - q) - Kd qd`, where `q` is visible **only inside the servo model**.
Realism is added in phases: A PD only; B torque and velocity ceilings;
C deadband and command delay; D gear friction and backlash; E temperature and
supply-voltage dependence.  Not all at once.

-> `reflex_quad/servo_model.py`, `config/servo.yaml`.  FINDINGS #9 covers where
friction is subtracted and why it matters.

## Current and power (sections 21, 22)

`I = I_idle + k_tau |tau|`, plus measurement noise.  **The controller gets
`I_measured` and never `tau`.**  Per-servo power `P_j = V I_j`, plus totals.

## Objective (section 23)

```
J_pose  = roll^2 + pitch^2
J_force = sum_i (F_i - F_i*)^2          F* defaults to the mean
J_power = sum_j I_j^2
J       = w_pose J_pose + w_force J_force + w_power J_power
```

-> `reflex_quad/objective.py`.  FINDINGS #4 for the weights.

## Experiment 01 - flat ground (section 24)

Ten seconds, flat floor, four feet down, no control.  Log time, roll, pitch,
four forces, eight currents, eight commands.  Pass: no NaN, no divergence, no
fall, plausible contact forces.

-> `runbooks/RB-04`.

## Experiment 02 - unseen step (sections 25, 26)

Raise the floor under the front-left foot by 20 mm.  Do not tell the controller.
It sees only the body IMU, the four forces and the eight currents.

```
e_roll  = (F_FL + F_RL) - (F_FR + F_RR)
e_pitch = (F_FL + F_FR) - (F_RL + F_RR)

u_roll  = -Kr roll  - Kwr roll_rate  - Kfr e_roll
u_pitch = -Kp pitch - Kwp pitch_rate - Kfp e_pitch
```

distributed to the legs with the appropriate signs.

-> `runbooks/RB-05`.  FINDINGS #1 adds the missing diagonal term.

## Experiment 03 - dither (sections 27-31)

The centre of the project.  For a joint,

```
u+ = u + delta      u- = u - delta          delta from 0.5 deg
```

measure `I+`, `I-`, the body IMU, the foot IMUs and the foot forces; record
`D_I = I+ - I-`.  With the objective,

```
dJ/du ~= (J(u+delta) - J(u-delta)) / (2 delta)
u <- u - alpha dJ/du
```

Start with **one joint** (FL knee, section 29), then sweep all eight one at a
time (section 30).

Pass (section 31): without being told about the 20 mm step, roll -> ~0 and
pitch -> ~0, the load distribution does not become extreme, and `J(t)` decreases
statistically.

-> `runbooks/RB-06`.  FINDINGS #2, #3 for probe size and the update rule.

## Experiment 04 - unload one leg (sections 32-34)

Ramp the front-left target load to zero, moving the load to the other three.
Confirm `F_FL -> 0` before lifting.

Section 33 is the important one: **do not trust the servo command.**  A leg
counts as free only when the foot IMU shows motion **and** the foot force is
below the contact threshold.

State machine (section 34):

```
STAND -> UNLOAD_LEG -> VERIFY_UNLOAD -> LIFT -> VERIFY_MOTION
      -> LOWER -> CONTACT_SEARCH -> LOAD -> STAND
```

-> `runbooks/RB-07`.  FINDINGS #5, #6, #7 -- the posture mask, the reachable
load target, and the `WEIGHT_SHIFT` state the cycle turns out to need.

## Experiment 05 - faults (sections 35-37)

Inject mid-run: A max torque x0.3; B joint friction x10; C servo delay +100 ms;
D foot sensor stuck; E foot IMU bias.

Signatures (section 36):

```
command sent + current high + foot motion low  ->  mechanical resistance
command sent + current low  + foot motion low  ->  motor / drive failure
foot force high + foot IMU vibration high      ->  unstable or slipping contact
```

Residual (section 37): `r = y - y_hat(command, previous observation, contact
state)`.  No machine learning needed at first -- mean and standard deviation from
normal operation is enough.

-> `runbooks/RB-08`.  FINDINGS #10, #11, #12.

## Self-check (section 38)

While standing, one leg at a time: unload, lift, dither hip, dither knee, lower,
contact, reload -- recording the current, foot IMU and contact force response and
comparing against the healthy signature.  A `SELF_CALIBRATION` mode.

-> `experiments/07_self_check.py`.  This is also the missing piece for the two
fault cases the passive monitor cannot see (FINDINGS #11).

## Experiment 06 and gait (sections 39, 40)

One leg: unload, lift, forward, lower, contact, load, while the other three keep
the body up.  Then a static crawl gait in an order such as FL, RR, FR, RL --
with transitions driven by **contact, load and IMU events, not by time**.

-> `runbooks/RB-09`.

## Logging (sections 41, 42)

Two separate streams.  **Control observation**: body IMU, foot IMUs, foot forces,
servo currents, servo commands.  **Ground truth**: `q`, `qd`, `tau`, true body
pose, true foot poses, true contact forces.  Ground truth is never read by the
controller.

CSV at minimum: timestamp; estimated roll and pitch and body rates; four foot
forces; eight currents; eight commands; `J_pose`, `J_force`, `J_power`,
`J_total`; state; fault flag.

-> `reflex_quad/logger.py`.

## Ground truth comparison (section 43)

**After** a run only: estimated body pose against true body pose, estimated load
direction against true joint torque.  This quantifies how far the robot got
without measuring joints.

-> `eval/metrics.py::_truth_comparison`.

## Required plots (section 44)

1 roll/pitch vs time; 2 four foot forces; 3 eight servo currents; 4 command vs
current; 5 `J` vs time; 6 dither direction vs delta-J.

-> `eval/plots.py`, which adds estimate-vs-truth and a state timeline.

## Phase gates (sections 45-48)

* **Phase 1** -- on an unknown step, with no `q`, `qd` or `tau`, using only body
  IMU + four foot forces + eight currents + its own commands, roll and pitch
  converge to zero.
* **Phase 2** -- one leg: unload, lift, confirm motion by IMU, lower, detect
  contact, reload.
* **Phase 3** -- an artificial servo fault is detected purely from inconsistency
  between command, current, foot IMU and foot force.
* **Phase 4** -- phase 2 on all four legs, forward motion with a static crawl.

-> `config/criteria.yaml`, `runbooks/RB-10`.

## Phase 5 - real parameters (section 49)

Buy one cheap servo and measure: no-load current, holding current, stall current,
maximum velocity, deadband, command delay, backlash, current against external
load.  Measure the FSR too: ADC against load, hysteresis, noise, saturation.  Put
the measured numbers into `config/servo.yaml` and `config/sensors.yaml`.

## The principle (section 50)

Do not tell the robot what angle its joints are at.  Let it observe what came
back from its body and its environment when it issued a command.  The unit of
control is `command -> response`, and the loop is:

```
small perturbation -> observe response -> evaluate -> move toward a better state
```

---

## Additional design notes from the discussion

These are not numbered sections but are referenced as `memo_full.txt`:

* **Normalised load ratios.**  `r_roll = (F_FL + F_RL - F_FR - F_RR) / S`, and
  likewise for pitch, so that absolute FSR accuracy stops mattering.
  -> `controller.gains.normalized_force_error`.
* **Target load distribution `F*`**, ramped rather than stepped when a leg is to
  be freed.  -> `reflex_quad/support.py`.
* **Synchronous detection** of the dither response, to reject drift.
  -> the ABBA probe ordering in `reflex_quad/dither.py`.
* **Hardware**: FSR such as the Alpha MF01A (0.1-9.8 N) with a mechanical
  stop to protect it; INA226 for per-servo current; five IMUs.
* **Four-stage verification ladder**: ideal PD -> torque/speed limits ->
  delay/friction/backlash with noisy sensors -> parameter randomisation.
  -> `config/fidelity.yaml`.
* **The 4070 is below the documented Isaac Sim minimum** (RTX 4080, 16 GB).
  Stay headless, camera-free and single-environment.  -> `docs/ISAAC_NOTES.md`.
