# RB-09 - Experiment 06: the first step

**Goal**: one leg unloads, lifts, moves **forward**, lands, and reloads, while
the other three keep the body up.
**Time**: 1 h.  **Needs**: RB-07 passed.

memo.txt section 39.  Not a gait: one step.

## Run it

```bash
.venv/bin/python -m reflex_quad 06_first_step --backend mock --eval
python -m reflex_quad 06_first_step --backend isaaclab --eval
```

The only difference from experiment 04 is `state_machine.forward: 0.030` -- the
foot is commanded 30 mm ahead while it is in the air.

## Pass criteria

Everything from RB-07, plus `moved_forward`: the foot ends at least 15 mm ahead
of where it started.

## What the mock cannot tell you

**The mock has no horizontal degree of freedom.**  The body's x, y and yaw are
pinned, so the robot does not translate no matter how well the step executes.  A
pass here means the *cycle* is right -- unload, lift, reach, land, reload, with
the body under control -- and nothing more.  Whether the robot moves is an Isaac
question, and it is the whole point of the experiment.  See
`docs/MOCK_BACKEND.md`.

## On Isaac, watch for

* **Does the body actually move forward?**  `truth.csv` -> `body_x`.  If the
  foot lands ahead but the body does not follow, the stance legs are not
  transferring the reach into translation -- which on a sagittal-only leg means
  the *other* feet must trail backwards as the body advances.  That is the next
  design step, not a failure of this one.
* **Slip on touchdown.**  `02_foot_forces.png` plus the foot IMU.  A foot that
  lands moving is a foot that slides; slow `LOWER` or search for contact more
  gently (`CONTACT_SEARCH` already moves at 30% speed).
* **Tipping.**  Six real degrees of freedom, one leg in the air, CoM near the
  support boundary.  If it tips, revisit the weight shift (FINDINGS #7); this is
  the mechanical design telling you something.

## Stride limit

At the nominal 180 mm stance the symmetric leg sits at -41 deg of hip, and the
limit is +-80 deg (`config/robot.yaml`, FINDINGS #13).  Before asking for a
longer step, check the commanded angles are not being clipped -- `u_0` in
`control.csv` sitting flat at the limit is the tell.

## Then: a crawl gait

memo section 40.  Once one leg steps reliably, `07_self_check` already runs the
cycle on all four legs in sequence (`state_machine.legs: [0, 1, 2, 3]`); change
that list to the static crawl order `FL, RR, FR, RL` -> `[0, 3, 1, 2]` and give
each leg a forward target.  Keep the transitions **event-driven** -- on contact,
on load, on IMU -- not on a timer.  That is memo section 40's actual
requirement and the reason the state machine is written the way it is.

## Record

Foot forward displacement, body x displacement (Isaac only), peak tilt, whether
the foot slipped on landing.
