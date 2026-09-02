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

**Not yet root-caused.** Candidates, in the order they'd be cheapest to check:
gains tuned against the mock's reduced-order (heave/roll/pitch only, no
horizontal freedom) dynamics not holding once the body can actually translate
and yaw; the effort-driven joints (section on "why the servo is ours" in
`docs/ISAAC_NOTES.md`) behaving differently from mock's second-order joint
model under real link inertia and self-contact; or the initial spawn height
(`stance.height + 0.02`) putting the feet in penetration or free-fall against
PhysX's actual collision geometry in a way the mock's unilateral spring-damper
foot never has to resolve. Whichever it is, this is the first concrete number
this project has for the gap `docs/MOCK_BACKEND.md` always said would be
there -- log it against the fix, not against a v2 of this finding.
