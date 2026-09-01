# RB-07 - Experiment 04: unload, lift, verify, lower, reload

**Goal**: get one leg off the ground and back, and *know* it happened from the
foot's own IMU rather than from the fact that you commanded it.
**Time**: 1 h.  **Needs**: RB-06 passed.

memo.txt sections 32, 33, 34.

## The state machine

```
STAND -> WEIGHT_SHIFT -> UNLOAD_LEG -> VERIFY_UNLOAD -> LIFT
      -> VERIFY_MOTION -> LOWER -> CONTACT_SEARCH -> LOAD -> DONE
```

`WEIGHT_SHIFT` is not in the memo's list and the cycle does not work without it.
**Read `docs/FINDINGS.md` #7.**  With the CoM at the geometric centre it sits
exactly on the line between the two adjacent feet, so three-leg support is
neutrally stable and the body just rotates about that line until the foot lands
again.  These legs have no lateral freedom, so the shift is along x: the other
foot at the same end moves 50 mm outward, which rotates the support line clear
of the CoM.  Statics and simulation agree (RR margin 0.33 N -> 1.33 N).

Two other things the memo leaves open, both in FINDINGS:

* the target load distribution is **not** W/3 each (#6) -- the diagonal partner
  must carry about half.  `reflex_quad/support.py` solves for it.
* the leg in the cycle is removed from the posture loop *and* from the load
  error (#5), or the posture loop undoes the unload.

## Run it

```bash
.venv/bin/python -m reflex_quad 04_leg_unload --backend mock --eval
python -m reflex_quad 04_leg_unload --backend isaaclab --eval
```

## Pass criteria

| check | threshold | memo |
|---|---|---|
| `unload_reached` | target foot force <= 0.3 N | 32 |
| `verified_by_imu` | `sm_motion_verified` true | **33** |
| `cycle_completed` | reached DONE | 46 |
| `posture_held` | tilt during the lift <= 8 deg | |
| `reloaded` | target foot force >= 1 N at the end | |

## The section 33 check

`LEG_FREE` requires **both**:

* foot force below the contact threshold, and
* integrated foot **gyro** past `motion_threshold` (0.10 rad)

Not RMS acceleration.  A planted foot rattles harder than a hanging one, and
double-integrated acceleration drifts -- a 29 mm lift measured as 405 mm.  See
FINDINGS #8.  A 30 mm lift rotates hip+knee by ~0.37 rad, so the margin is more
than 3x.

## Expected (mock)

```text
t= 2.0  WEIGHT_SHIFT    F_FL 3.9 -> 0.2 N
t= 4.5  UNLOAD_LEG
t= 5.1  LIFT            foot rises to +29 mm
t= 6.6  VERIFY_MOTION   motion metric 0.163 rad  (threshold 0.10)
t= 7.8  CONTACT_SEARCH
t= 8.6  LOAD
t= 9.3  DONE            F_FL back to ~3.5 N, roll +0.1 deg, pitch -0.7 deg
```

## If it aborts

Every state has a timeout; `meta.json` -> `summary.aborted_reason` names the one
that ran out.

| aborted in | look at |
|---|---|
| `WEIGHT_SHIFT` | `state_machine.weight_shift` too small, or a hip limit is clipping the shifted foot |
| `UNLOAD_LEG` | the classic one.  Is the posture loop fighting it (`posture_mask`)?  Is `F*` reachable (FINDINGS #6)?  Is `stance.min_height` deep enough to retract? |
| `VERIFY_MOTION` | `motion_threshold` too high for the lift height, or the foot IMU is not on the foot link (URDF `merge_fixed_joints` must be false) |
| `CONTACT_SEARCH` | `contact_threshold` below the FSR noise floor |

**Expect this to be harder on Isaac.**  The mock cannot tip sideways and cannot
translate; Isaac can do both.  If the robot tips during the lift, that is a real
result, not a bug -- it is FINDINGS #7 showing up with the full six degrees of
freedom, and the honest fixes are a bigger weight shift, a slower unload
(`unload_rate`), a lower stance, or a CoM offset in the mechanical design.

## Record

The state timeline (`08_states.png`), the motion metric at VERIFY_MOTION, peak
tilt during the lift, and whether the weight shift was enough.
