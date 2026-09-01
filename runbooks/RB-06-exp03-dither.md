# RB-06 - Experiment 03: active sensing

**Goal**: the robot levels itself on an unseen step **without any control law
for it** -- by probing and descending `J`.
**Time**: 1-2 h including tuning.  **Needs**: RB-05 passed.

memo.txt sections 27-31.  This is the centre of the project.  Everything else is
scaffolding for this experiment.

## What happens

For one joint at a time:

```
u + delta   ->  measure J, I        (settle 0.25 s, measure 0.20 s)
u - delta   ->  measure J, I
dJ/du ~= (J_plus - J_minus) / (2 delta)
u <- u - step(sign of the gradient)
```

No inverse kinematics, no Jacobian, no joint measurement.  The robot knows only
what it commanded and what came back.

## Run it

```bash
# one joint (FL knee), 60 s
.venv/bin/python -m reflex_quad 03_dither --backend mock --eval

# all eight joints in sequence, 150 s
.venv/bin/python -m reflex_quad 03b_dither_all --backend mock --eval

# then Isaac
python -m reflex_quad 03_dither --backend isaaclab --eval
```

## Expected (mock, one joint)

```text
t= 0-12 s   perched on the FL/RR diagonal, F = [7.4, 0, 0, 6.8] N, J = 2.57
t=18 s      other feet making contact
t=30 s      roll 0.07 deg, pitch -0.04 deg, F = [3.8, 3.5, 3.3, 3.5], J = 0.008
```

`J` falls by ~300x, from **one joint**, with the 20 mm step never disclosed.
Eight joints reach the same place in ~150 s (each joint gets an eighth of the
probes).

## Pass criteria

| check | threshold | note |
|---|---|---|
| `j_decreases` | Mann-Kendall p <= 0.05 | memo 31 "J decreases statistically" |
| `j_improvement` | last fifth 1.5x better than the first | |
| `gradient_consistent` | sign consistency >= 0.70 | over the first 60% of updates only |
| `level_roll` / `level_pitch` | <= 3 deg | memo 31 |
| everything from RB-04 | | |

## Tuning on Isaac -- expect to do this

The probe parameters were tuned against the mock backend and Isaac's contact
noise will be different.  **Read `docs/FINDINGS.md` #2 and #3 first**; they
explain why the memo's starting values do not work as written.

Retune in this order, in `config/experiment.yaml` under `dither`:

1. **`delta`** (default 0.020 rad).  memo section 27 suggests 0.0087 (0.5 deg);
   measured on the mock that moves `J` by 0.014 against a +-0.1 noise floor, so
   it estimates nothing.  Check `06_dither.png`: if `dJ` scatters around zero
   with no structure, raise `delta` until it separates.  Then bring it back down
   as far as it still works -- a big probe is a big disturbance.
2. **`settle_time`** (0.25 s).  If `05_objective.png` shows `J` still moving when
   the measurement window opens, raise it.  Too short is the classic way to
   measure noise.
3. **`repeats`** (2).  Probe pairs per estimate, in ABBA order (`+ - - +`) so
   drift that is linear over the cycle cancels.  Raise to 3-4 if the sign is
   still unreliable; each repeat costs 0.9 s per update.
4. **`rprop_step0` / `rprop_step_max`** (0.010 / 0.030 rad).  Convergence speed.
   Too large and it overshoots and hunts; too small and 150 s is not enough.
5. **`objective` weights**, only if the search converges somewhere you did not
   want.  `w_power` large enough makes "collapse and carry nothing" the optimum.

If you want the literal memo section 28 update instead of the sign-based one:
`dither.update_rule: gradient`, and then `alpha` matters and needs the scale of
`J` (see FINDINGS #3).

## Read these plots

* `06_dither.png` **left** -- `J(+d) - J(-d)` per update, coloured by joint.
  Early on it should be consistently one sign per joint; late it should sit near
  zero and alternate.  That transition *is* convergence.
* `06_dither.png` **right** -- current difference vs objective difference, the
  memo section 27 `D_I` plot.  Structure here means the current alone carries
  usable direction information, which matters for the real robot where `J` needs
  the FSRs and `D_I` does not.
* `05_objective.png` -- `J` and its three terms, log scale.
* `01_attitude.png` -- estimate against truth, all the way down.

## If J will not go down

1. Is the probe visible at all?  `06_dither.png` left, and FINDINGS #2.
2. Is the objective the one you meant?  If `J_power` dominates
   (`05_objective.png`), the robot is correctly minimising the wrong thing.
3. Is the search on a plateau?  With only two feet on the ground, `J_force` is
   **exactly** flat -- the two-point load split is fixed by statics, not by leg
   length -- and only `J_pose` provides a gradient.  This is real, not a bug, and
   it is why `w_pose` is 10.
4. Is the step size collapsing?  `dither_step` in `control.csv` at the floor
   with the gradient still one-signed means RPROP shrank on noise; raise
   `rprop_step_min` or `repeats`.

## Record

Every dither parameter, the J trace, the pass/fail table, and the plots.  If
this passes on Isaac, **memo.txt phase 1 is met** and the idea works.
