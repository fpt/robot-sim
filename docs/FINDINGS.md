# Findings

Things that were discovered while building this repository.  1-14 are from the
mock backend, before any GPU time was spent; each is either a deviation from
`memo.txt` or a constraint the memo does not mention, and every one of them
would otherwise have been found the expensive way: mid-experiment on the CUDA
machine.  15 on is from real Isaac Sim time, once the bring-up in
`docs/ISAAC_NOTES.md` reached a working `isaaclab` backend.

They are all reproducible: the number in brackets is the test or experiment that
demonstrates it.

---

## 1. Roll and pitch feedback cannot see the load imbalance a step creates

A single foot on a 20 mm block loads the **FL/RR diagonal** and unloads the
FR/RL one, with the body **almost level**.  Four feet on a rigid body are
statically indeterminate; the twist mode is the null space, and `memo.txt`
section 26 has no term for it.

Measured with the section 26 controller alone: final tilt 0.09 deg roll,
0.43 deg pitch -- excellent -- with foot forces `[6.86, 0.60, 1.04, 5.78] N`,
a coefficient of variation of 0.79.  It looks like a success and is not one.

**Change**: added a third balance term to `PostureController`,

```
e_twist = (F_FL + F_RR) - (F_FR + F_RL)
```

fed to the diagonal pairs with opposite sign.  CV drops from 0.79 to about 0.17.
Gain `controller.gains.k_force_twist`.

*The dither search finds this mode on its own* -- it is a plain descent
direction of `J_force` -- which is a small but real demonstration that active
sensing recovers a control mode the hand-written law was missing.

## 2. A 0.5 degree dither is below the noise floor

`memo.txt` section 27 suggests starting at `delta = 0.5 deg`.  Measured
statically, that probe moves `J` by 0.014 while the transient scatter of `J`
during the run is +-0.1.  Signal-to-noise below 0.15; the estimated gradient is
noise and the sign flips at random.

**Change**: three of them.

* `delta` default 0.020 rad (1.15 deg).
* Each gradient estimate averages `repeats` probe pairs in **ABBA order**
  (`+ - - +`), which cancels drift that is linear over the cycle.  This is the
  discrete form of the synchronous detection `memo_full.txt` asks for.
* Settle 0.25 s / measure 0.20 s rather than 0.10 / 0.06: the body is still
  ringing at 0.10 s.

Result on experiment 03 (one joint, FL knee, unknown 20 mm step):
`J` 2.57 -> 0.008, roll 3.2 deg -> 0.07 deg.

## 3. `alpha * dJ/du` cannot be tuned in advance; sign descent can

The section 28 update needs the scale of `J`, which changes with the objective
weights, the terrain and the sensor calibration.  With the weights below, the
measured gradient is around 29 J/rad, so `alpha = 0.35` asks for a 10 rad step.

**Change**: default `dither.update_rule: rprop` -- per-joint adaptive step size
driven by the **sign** of the finite difference only, which is the part of a
noisy probe you can trust.  The literal section 28 rule is still available as
`update_rule: gradient`.

## 4. Objective weights need three orders of magnitude of scaling

Raw terms on a 20 mm step: `J_force ~ 30 N^2`, `J_pose ~ 0.02 rad^2`,
`J_power ~ 0.36 A^2`.  With the memo's `w = (1, 0.02, 0.05)` the objective is
dominated by power, whose minimum is "collapse and carry nothing".

**Change**: `w_pose 10.0, w_force 0.05, w_power 0.005`.  Posture and load
balance land within ~5x of each other and power stays a regulariser.

## 5. A leg cannot be unloaded while its own posture loop is still driving it

Unloading tilts the body; the posture loop reads the tilt as an error and
extends the leg being unloaded, putting the load straight back on.  Observed:
`F_FL` stuck at 1.2 N with the leg retracted to its 60 mm limit, body rolled
8.9 deg.

**Change**: `posture_mask`.  A leg in the lift cycle is removed both from the
posture correction and from the load-error sum -- its residual must not be
charged to the legs that are still supporting.

## 6. The uniform load target `W/3` is physically unreachable

`memo.txt` section 32 says to move the load to the other three legs.  Splitting
it evenly is impossible: with one leg up, the three equilibrium equations force
the **diagonal partner to carry about half the weight**.  A load-balance loop
chasing W/3 saturates and destroys the posture.

**Change**: `reflex_quad/support.py` solves for the least-norm load split that
still balances the body, from the *commanded* foot positions (no joint
measurement).  For four feet it returns W/4 each, as the memo expects.

## 7. With the CoM at the geometric centre, no leg can be lifted at all

The CoM sits exactly **on the line** between the two adjacent feet, so the
three-leg support is neutrally stable: the body rotates about that line until the
foot touches down again.  These legs have no lateral freedom, so the only
available weight shift is along x.

**Change**: a `WEIGHT_SHIFT` state moves the *other foot at the same end*
outward by 50 mm before unloading, which rotates the support line clear of the
CoM.  Statics and simulation agree: RR goes from 0.33 N (marginal) to 1.33 N of
margin at a 50 mm shift.

**Implication for the real robot**: the machine will want either a CoM offset
from the geometric centre, a lateral hip degree of freedom, or this x-shift
trick built into the gait.  This is a mechanical design decision, and it is
forced.

## 8. "Did the foot move?" is the wrong question to ask an accelerometer

Section 33 requires proof of motion from the foot IMU.  Three metrics were
tried:

| metric | result |
|---|---|
| RMS of foot acceleration | **wrong sign** -- a planted foot rattles at ~4 m/s^2, a lifted one at ~1.3 |
| double-integrated acceleration | **drifts** -- a 29 mm lift measured as 405 mm |
| integrated foot **gyro** | **works** -- 0.163 rad measured for a 30 mm lift, bias over the window ~0.01 rad |

**Change**: `motion_threshold` is now 0.10 rad of integrated foot rotation.
`memo_full.txt` says the same thing in words: an IMU gives attitude, angular
rate, and start/stop of motion -- not position.

## 9. Friction must be subtracted at the output, not before the current

The first servo model computed `tau` after subtracting friction and derived the
current from that, so "gearbox friction x10" produced **no current rise at all**
and section 35 case B was invisible by construction.  The motor pushes *through*
friction.

**Change**: `tau_motor` (drives current) and `tau_out = tau_motor - friction`
(drives the joint) are separate.  Also `tanh(qd/0.02)` was chattering at 500 Hz
with a 1.5e-3 kg m^2 rotor -- friction reversed the joint velocity inside one
timestep, so "more friction" showed up as *more* foot motion.  Regularisation
width is now `friction.stick_velocity = 0.2 rad/s`.

## 10. Static current says nothing about friction

Even with the model fixed, friction is invisible in a window-averaged current:
at rest the position loop settles at the same current whether the gearbox is
clean or seized (0.872 vs 0.876 of baseline).

**Change**: the fault monitor's `current_dynamic` feature -- current while the
command is moving minus current while it is held.

## 11. Two of the five fault cases are blind spots at the default probe speed

Measured, with the fault monitor as it stands:

| case | fault | detected | latency |
|---|---|---|---|
| A | max torque x0.3 | yes, `drive_failure`, correct joint | 7.2 s |
| B | friction x10 | **no** | -- |
| C | +100 ms command delay | **no** | -- |
| D | foot FSR stuck | yes, `force_sensor_stuck` | 1.4 s |
| E | foot IMU bias | yes, `foot_imu_bias` | 0.9 s |

False alarms before injection: 0 in all five cases.

* **B** is a velocity cost and the section 27 dither moves the joint at only
  ~0.08 rad/s, where 10x friction shifts the moving-vs-holding current by
  0.038 A against a 0.037 A noise floor.
* **C** needs a lag measurement; estimated opportunistically its scatter
  (+-0.042 s) is comparable to the 0.100 s fault.  `sluggish_response` is
  therefore **off by default** -- with it on it produced hundreds of false
  positives.

Both want the same thing: the **deliberate, fast, known-waveform probe** of the
section 38 self-check, rather than passive monitoring of whatever the robot
happened to be doing.  That is the next piece of work, and until it exists both
cases are marked `expected_detectable: false` in `config/faults.yaml` so the
evaluator reports them as known gaps instead of silently passing.

The A latency of 7.2 s is not detector slowness: it is how long until the
8-joint sweep probes that joint again.  Detection is only possible while the
joint is being driven.

## 12. The fault monitor must know the contact state

`memo_full.txt` defines the residual as `r = y - y_hat(u, contact state)`.  The
contact-state part is not optional: a leg deliberately in the air has no load
and a freely swinging foot, which is exactly the signature of a broken one.
Running the monitor through experiment 04 unfiltered produced 260 detections.

**Change**: `fault_monitor.active_states` -- learn and judge only in
quasi-static supported states.  260 -> 0.

## 13. A +-60 degree hip limit clips ordinary poses

At the nominal 180 mm stance the symmetric leg already sits at -41 deg of hip,
leaving 19 deg of stride margin, and a 130 mm crouch with a 40 mm reach is
already outside the limit.  Widened to +-80 deg, which a 300 deg-range hobby
servo gives easily.  [`tests/test_robot.py::test_ik_fk_round_trip`]

## 14. Gradient sign consistency must be scored during the search only

A converged dither alternates its gradient sign on purpose -- that is what
hunting around a minimum looks like.  Scored over the whole run it reads 0.66;
over the first 60% of updates, 0.83.  The criterion uses the search phase.

## 15. `01_stand` stands on mock and falls over on real Isaac physics

`docs/MOCK_BACKEND.md` says outright that mock has no horizontal motion, no
yaw, no tipping sideways -- so a mock `[PASS]` was never proof the robot
stands, only that the control logic and plumbing are sound.  First real run
on the `isaaclab` backend, once bring-up was working end to end
(`docs/ISAAC_NOTES.md`), confirms it is not enough:

```
python -m reflex_quad 01_stand --backend isaaclab --duration 5 --eval
=== 01_stand  [FAIL]  logs/01_stand_20260902_120202
   [PASS   ] no_nan                   nan_count = 0
   [FAIL   ] no_divergence            diverged = True
   [FAIL   ] no_fall                  fell_over = True
   [FAIL   ] contact_sane             total_force_mean_N = 0.4012  (need >= 5.0)
   [PASS   ] settled                  settling_time_s = 2.31       (need <= 3.0)
   [FAIL   ] quiet_roll               final_abs_roll_deg = 64.01   (need <= 2.0)
   [FAIL   ] quiet_pitch              final_abs_pitch_deg = 130.3  (need <= 2.0)
   [FAIL   ] even_load                final_force_cv = 1.051       (need <= 0.15)
```

Same config the mock run passes with `total_force_mean_N = 14.16`,
`final_abs_roll_deg = 0.17`, `final_abs_pitch_deg = 0.12`.  Pitch beyond 130
degrees and near-zero mean contact force say the robot is on its side or back
by the end of the window, not standing with a large tilt -- `fell_over`
(tilt > 45 deg) trips well before `diverged` (tilt > 60 deg) does.

**Confirmed downstream before the fix, not a fluke of one run** (2026-09-02,
same session): `02_uneven_ground` and `03_dither` both failed the same way on
`isaaclab`, as RB-04 warns they would ("if this fails, nothing downstream
means anything").

```
02_uneven_ground  [FAIL]  final_abs_roll_deg=643.3  final_abs_pitch_deg=614.1
                          min_foot_force_final_N=0.048 (a foot came off)
03_dither         [FAIL]  final_abs_roll_deg=199.8  final_abs_pitch_deg=149
```

`02`'s roll/pitch past 360 degrees meant it was not one fall and rest -- the
body kept tumbling for the whole 15 s window, unlike `01`'s single topple.
Worth noting for whoever hits something like this again: `03_dither`'s
`j_decreases` check still **passed** (Mann-Kendall p = 1.1e-55) while the
robot was down and `no_fall` failed -- `J`'s pose and force terms both
legitimately shrink once a fallen body stops moving, so a passing
`j_decreases` on Isaac is not by itself evidence the dither search is doing
anything; check `no_fall` first.

**Root cause found and fixed, same day.** Not the servo, not mass/CoM, not
mock-tuned gains not transferring -- the robot was never spawning in
`nominal_command()`'s pose at all. `truth.csv` at `t=0` showed every joint at
~0 rad (URDF's raw rest pose: hip straight down, knee straight) instead of
the commanded -41.4/82.8 deg crouch, and `tau_*_knee` pinned at exactly
`-1.6` (`servo.limits.tau_max`) from the first tick -- the servo trying, and
failing, to snap a straight leg to a bent one against a real 3D body it
cannot fully support from that pose. A straight leg (`q0=q1=0`) also reaches
0.24 m (`l1+l2`), but the body was only spawned `stance.height + 0.02 = 0.20`
m up, so the feet started ~4 cm *into* the ground; the "launch" in the
original trace (`body_z` 0.218 -> 0.395 m in 0.19 s) was PhysX's
depenetration solver firing on that overlap, not a fall.

The bug: `IsaacLabBackend.reset()` called `self.robot.reset()` then
`self.sim.reset()`, expecting `ArticulationCfg(init_state=...)` to already be
in effect. `Articulation.reset()` only resets actuator-internal state (delay
queues); it never re-applies `init_state`. Isaac Lab's own convention is that
whoever calls `reset()` must explicitly write the default root pose/velocity
and joint position/velocity back into the sim afterward
(`write_root_pose_to_sim_index`, `write_joint_position_to_sim_index`, etc.,
from `robot.data.default_*`) -- this project's backend never did, so every
Isaac run started from the raw USD pose regardless of `init_state`.

Fixed in `IsaacLabBackend.reset()`: `sim.reset()` first, then the four
`write_*_to_sim_index` calls from `robot.data.default_*`, then
`robot.reset()`.  Result, same config, same host:

```
01_stand           [PASS]  all 10 checks, total_force_mean_N=14.32,
                            final_abs_roll_deg=0.033, final_abs_pitch_deg=0.69
02_uneven_ground    [FAIL]  9/10 -- only level_roll misses, 4.59 deg (need <= 3)
                            no_fall/no_divergence both pass, roll_improvement_ratio=2.95
```

```
03_dither           [FAIL]  6/11 -- no_fall/no_divergence pass, stands at
                             final_abs_roll_deg=0.37, final_abs_pitch_deg=0.30
                             j_improvement=6.72 (need >= 1.5) passes strongly
                             j_decreases p=0.95, gradient_consistent=0.56 fail
```

`01_stand` matches the mock's numbers closely. `02_uneven_ground` and
`03_dither` are now genuine, small-margin tuning questions, not instability --
which is the state this project's Isaac work was always supposed to reach.
`02`'s miss (`level_roll`) is RB-05's own "Tuning, if Isaac disagrees" section
(`k_force_twist`/`k_roll`). `03`'s misses are exactly what RB-06 already warns
about ("Read `docs/FINDINGS.md` #2 and #3 first; the probe parameters were
tuned against the mock backend and Isaac's contact noise will be different")
-- `j_improvement` passing by 4x the threshold while the trend/consistency
checks fail says the search is finding a real direction and moving, just not
as cleanly as the mock's lower-noise contact model let it; `delta` and
`repeats` are the first two knobs to try, per RB-06.

**A second, smaller spawn bug, caught immediately instead of by more
tumbling.** With the `reset()` fix above, `runner.py` now runs
`_assert_sane_initial_state(sim, geom, backend)` right after every
`backend.reset()` -- checks `q` is finite, within its joint limits, and
within `0.01` rad of `nominal_command()`, and that every foot is within a few
cm of the terrain height under it, on *any* backend, since it only reads what
`SimState` already returns. First real use, on `isaaclab`, immediately caught
a second bug the eval criteria alone had let through as a `[PASS]`: `q` was
now correct, but `foot_z` was still 10 mm inside flat ground.
`IsaacLabBackend`'s spawn height was `stance.height + 0.02`, a constant
carried over from before the `reset()` fix and never revisited -- it assumes
`hip_z == body_z`, true in the mock's reduced-order model but not in the real
URDF, which mounts each hip `body.height / 2` (30 mm) below the body link's
own origin (`asset_builder.py`). On `02_uneven_ground` the same formula was
worse -- 15 mm into the 20 mm block -- because it doesn't look at terrain at
all, unlike `MockBackend.reset()`'s own `z = max(ground + ext) + margin`.
Fixed with a `_spawn_height()` method that does the same max-over-feet
terrain lookup mock does, plus the real hip offset. Both are one-line-cause,
easy-to-miss-by-eye bugs that a criteria-only `[PASS]` does not surface --
`02_uneven_ground`'s `total_force_mean_N=14.35` and `no_fall`/`no_divergence`
both passing, reported earlier in this entry, were **already** on top of the
15 mm foot penetration; the eval numbers alone gave no reason to suspect it.
Final numbers, clean spawn confirmed by the check on all three:

```
01_stand           [PASS]  all 10 checks (unchanged by this fix)
02_uneven_ground    9/10   level_roll now passes (1.42 vs 4.59 deg before);
                           level_pitch/improved_roll now the narrow misses
                           (3.65 vs <=3, 1.88 vs >=2.0) -- still tuning, not
                           instability, just a different close margin
03_dither           6/11   unchanged in shape: stands (roll 0.53, pitch 0.09),
                           j_improvement=3.99 passes, trend checks still miss
```

The lesson for next time this project touches a backend's `reset()`: check
`q` and foot placement against the model at `t=0`, in code, not by reading a
criteria table. A passing `contact_sane`/`no_fall` measures the robot's state
several seconds in, after the servo has had time to (mostly) recover from a
bad spawn -- it does not mean the spawn was clean.
