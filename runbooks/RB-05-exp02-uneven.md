# RB-05 - Experiment 02: unseen 20 mm step

**Goal**: the robot levels itself on ground it cannot see, using the section 26
control law.
**Time**: 30 min.  **Needs**: RB-04 passed.

memo.txt sections 25 and 26.  The front-left foot stands on a 20 mm block.  The
controller is **not told**.  All it has is the body IMU, four foot forces, eight
currents and its own commands.

## Run it

```bash
.venv/bin/python -m reflex_quad 02_uneven_ground --backend mock --eval
python -m reflex_quad 02_uneven_ground --backend isaaclab --eval    # Isaac env
```

To change the obstacle, edit `config/experiment.yaml`:

```yaml
  "02_uneven_ground":
    terrain:
      blocks:
        - {x: 0.150, y: 0.090, size_x: 0.12, size_y: 0.12, height: 0.020}
```

## What the controller is doing

memo section 26, plus one term the memo does not have:

```
e_roll  = (F_FL + F_RL) - (F_FR + F_RR)      left  - right
e_pitch = (F_FL + F_FR) - (F_RL + F_RR)      front - rear
e_twist = (F_FL + F_RR) - (F_FR + F_RL)      diagonal
```

**Read `docs/FINDINGS.md` #1 before judging this run.**  A single raised foot
loads the FL/RR diagonal and unloads the other one *while leaving the body
level*.  Roll and pitch feedback structurally cannot see that mode.  With only
the memo's two terms the run looks like a success -- 0.09 deg of roll -- while
two feet carry 6.9 N and the other two carry 0.6 N.  The twist term
(`gains.k_force_twist`) is what fixes it, and `final_force_cv` is the check that
catches it.

The load errors are divided by the total load (`normalized_force_error: true`),
which makes the balance signal immune to per-FSR gain error -- the cheap sensors
in memo section 18 will not match each other.

## Pass criteria

| check | threshold |
|---|---|
| everything from RB-04 | |
| `level_roll` / `level_pitch` | <= 3 deg |
| `improved_roll` | final at least 2x better than the early peak |
| `load_not_extreme` | force CV <= 0.45 |
| `no_foot_lost` | least-loaded foot >= 0.3 N -- all four still down |

## Expected numbers (mock)

```text
final_abs_roll_deg    0.96     final_force_cv   0.17
final_abs_pitch_deg   0.75     e_twist steady   ~2.5 N
```

A steady residual `e_twist` is expected: the law is proportional, so it droops.
Do not add an integrator to chase it -- experiment 03 is what removes it, without
a model.

## Tuning, if Isaac disagrees

In `config/experiment.yaml` under `controller.gains`, one at a time:

1. `k_force_twist` (default 0.012) -- raise if `final_force_cv` stays high
2. `k_roll` / `k_pitch` (0.35) -- raise if the tilt is slow to settle
3. `posture_limit` (0.030 m) -- the per-leg cap; raise only if the correction is
   visibly saturating in `02_foot_forces.png`
4. `cmd_rate_limit` (1.5 rad/s) -- lower if the servos are chattering

If it oscillates, lower the force gains before the attitude gains: the force
loop closes through contact stiffness, which is the stiffest thing in the system.

## Record

Final roll/pitch, the four final foot forces, the CV, and every gain you
changed.  Gains are results.
