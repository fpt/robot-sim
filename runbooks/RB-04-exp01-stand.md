# RB-04 - Experiment 01: standing on flat ground

**Goal**: the robot stands, on flat ground, with **no feedback at all**.
**Time**: 20 min.  **Needs**: RB-03 passed.

memo.txt section 24.  This is the "is the physics sane" experiment.  The
controller holds the nominal stance and does nothing else; if this fails,
nothing downstream means anything.

## Run it

```bash
# mock first, always
.venv/bin/python -m reflex_quad 01_stand --backend mock --eval

# then Isaac
source ~/robotics/env_isaaclab/bin/activate
python -m reflex_quad 01_stand --backend isaaclab --eval
```

## Pass criteria

From `config/criteria.yaml`, checked automatically:

| check | threshold | why |
|---|---|---|
| `no_nan` | 0 NaN | memo 24 |
| `no_divergence` | tilt < 60 deg, z < 1 m, I < 20 A | memo 24 |
| `no_fall` | peak tilt < 45 deg | memo 24 |
| `contact_sane` | 5 N <= total foot force <= 30 N | the four feet carry the robot's weight (1.4 kg -> 13.7 N) |
| `current_sane` | max current <= 5 A | a cheap servo stalls near 2 A |
| `settled` | settling time <= 3 s | it should stop moving |
| `quiet_roll` / `quiet_pitch` | <= 2 deg | flat ground, symmetric robot |
| `even_load` | force CV <= 0.15 | the four feet share the load |

## Look at these plots

`logs/<run>/plots/`:

* `01_attitude.png` -- solid is the IMU-only estimate, dotted is simulator truth.
  They should sit on top of each other.  This is the first evidence for memo
  section 43 and it costs nothing to check every run.
* `02_foot_forces.png` -- four traces settling to about a quarter of the weight
  each, after one bounce.
* `03_currents.png` -- knees carry the load, hips should be near idle.  On the
  symmetric leg the vertical load produces no hip torque; a hip drawing as much
  as a knee means the leg parametrisation is wrong.

## Expected numbers (mock backend, for comparison)

```text
final_abs_roll_deg     0.14      total_force_mean_N   14.2
final_abs_pitch_deg    0.11      final_force_cv       0.011
settling_time_s        0.0       roll_estimate_rmse   0.13 deg
```

Isaac will differ -- real contact, real inertia -- but the *shape* should match:
level, weight supported, quiet.

## If it fails

| symptom | look at |
|---|---|
| sinks slowly | contact stiffness (Isaac: solver iterations); servo `tau_max` too low to hold |
| bounces forever | contact damping; try `physics_dt: 0.001` |
| tips immediately | joint sign convention -- compare `assets/reflex_quad.urdf` axes against `reflex_quad/robot.py` |
| force ~0 on one foot | that foot is not touching: check the URDF conversion kept the foot links (`merge_fixed_joints: false`) |
| currents at the limit | `tau_max` too low for the robot's weight, or the stance is too extended |

## Record

Experiment note: the eight criteria results, the four final foot forces, the
attitude RMSE, and which backend.  This run is the baseline every later one is
compared against.
