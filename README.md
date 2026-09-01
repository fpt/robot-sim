# reflex_quad

A four-legged robot that is **not told what angle its joints are at**.

The simulator knows `q`, `qd` and `tau` -- it needs them for physics -- but the
control program never sees them.  All it gets is:

```
1 body IMU  +  4 foot IMUs  +  4 foot force sensors
            +  8 servo currents  +  supply voltage
            +  the commands it sent itself
```

From that it has to stand on ground it cannot see, level its body, share the
load across four feet, lift a leg and put it back, and notice when something
breaks.  The method is deliberate small movements: **probe, watch what comes
back, move toward the better state**.

Full requirements in [`docs/SPEC.md`](docs/SPEC.md); it is section-numbered and
everything in the code and the runbooks cites it.

## Why this repository exists

The target machine is a Linux box with an RTX 4070 running Isaac Sim 5.1 and
Isaac Lab.  Everything here is built so that the *only* thing needing that
machine is the physics:

```
                 controller, sensors, servo model, objective,
                 dither search, state machine, fault monitor,
                 logging, metrics, criteria, plots
                              |
              +---------------+---------------+
              |                               |
        mock backend                  Isaac Lab backend
      (numpy, 3 DOF, laptop)        (PhysX, 6 DOF, CUDA PC)
```

Both implement the same `SimBackend` interface, so an experiment is one flag
apart.  The mock suite runs in about 25 seconds and catches config and logic
errors before any GPU time is spent.  It is not a toy: fourteen substantive
findings in [`docs/FINDINGS.md`](docs/FINDINGS.md) came out of it, including
three that would have made a run *look* successful while measuring the wrong
thing.

## Quick start

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -e ".[dev]"

.venv/bin/python -m pytest -q                                  # 60 tests, ~25 s
.venv/bin/python -m reflex_quad 03_dither --backend mock --eval
```

That last command runs the central experiment -- levelling on an unseen 20 mm
step by probing a single joint -- and prints a pass/fail table.  Plots and a
report land in `logs/<run>/`.

On the CUDA machine, work through [`runbooks/`](runbooks/README.md) in order,
then:

```bash
python -m reflex_quad 03_dither --backend isaaclab --eval
```

## The experiments

| id | what it asks | spec |
|---|---|---|
| `01_stand` | does it stand on flat ground with no feedback at all? | 24 |
| `02_uneven_ground` | can a hand-written law level it on an unseen 20 mm step? | 25, 26 |
| `03_dither` | **can it level itself by probing, with no control law for the step?** | 27-31 |
| `03b_dither_all` | the same, sweeping all eight joints | 30 |
| `04_leg_unload` | unload a leg, lift it, prove it moved from its own IMU, put it back | 32-34 |
| `05_fault` | detect an injected fault from observations alone | 35-37 |
| `06_first_step` | one step forward | 39 |
| `07_self_check` | the insect-style self-inspection, all four legs | 38 |

`.venv/bin/python -m reflex_quad --list` prints them.

## Layout

```
config/       robot, servo, sensors, experiments, faults, fidelity, criteria
              -- all tuning lives here, none of it in code
reflex_quad/  robot.py servo_model.py sensors.py observer.py controller.py
              dither.py state_machine.py support.py objective.py faults.py
              logger.py runner.py backends/{mock,isaaclab}.py
experiments/  the numbered scripts from the spec (thin wrappers)
eval/         metrics -> criteria -> plots -> report, plus the isolation check
runbooks/     RB-00 .. RB-90, in order, for the CUDA machine
docs/         SPEC, FINDINGS, METRICS, MOCK_BACKEND, ISAAC_NOTES
scripts/      check_host.sh, isaac_preflight.py
logs/         one directory per run: control.csv, truth.csv, events.jsonl,
              meta.json, report.md, plots/
```

## The one rule

The controller may not read joint angle, joint velocity or joint torque.  This
is enforced three ways, because it is the entire point of the project and a
single accidental reference would quietly invalidate every result:

1. `Observation` is a frozen dataclass with no field for them.
2. `eval/isolation.py` parses the controller-side modules and fails on any
   mention of joint truth.  It is part of the phase 1 gate.
3. `runner._assert_isolated` refuses a controller holding a reference to the
   backend or the servo bank.

Joint truth is written to `truth.csv` and read **only** by the evaluator, after
the run, to answer "how close did the IMU-only estimate get?" (spec section 43).

## Evaluation

Pass/fail is decided by `config/criteria.yaml`, not by looking at a plot:

```bash
.venv/bin/python -m eval.cli --all --phase phase1 --phase phase2
```

```
=== 03_dither  [PASS]  03_dither_20260901_212700
   [PASS   ] j_decreases            J_trend_p_value = 1.75e-33  (need <= 0.05)
   [PASS   ] j_improvement          J_improvement_ratio = 61.2  (need >= 1.5)
   [PASS   ] gradient_consistent    gradient_sign_consistency = 0.833  (need >= 0.7)
   [PASS   ] level_roll             final_abs_roll_deg = 0.178  (need <= 3.0)
```

Thresholds are research decisions, so they are in version control where they can
be argued with, rather than buried in a script.  `docs/METRICS.md` defines every
metric.

## Where the interesting problems turned out to be

Short version of [`docs/FINDINGS.md`](docs/FINDINGS.md):

* Roll and pitch feedback **structurally cannot see** the load imbalance a single
  raised foot creates -- the body stays level while two feet carry everything.
  Four feet on a rigid body are statically indeterminate and the twist mode is
  the null space.
* A 0.5 degree dither is an order of magnitude below the noise floor.  Getting a
  usable gradient needed a bigger probe, ABBA-ordered repeats, and sign-based
  descent rather than `alpha * dJ/du`.
* A leg **cannot be lifted at all** with the CoM at the geometric centre: it sits
  exactly on the support-triangle boundary.  The fix available to a
  sagittal-only leg is to move the other foot at the same end outward first.
  This is a mechanical design constraint, discovered in simulation.
* "Did the foot move?" is the wrong question for an accelerometer -- a planted
  foot rattles harder than a lifted one.  The integrated gyro answers it cleanly.
* Two of the five fault cases are **not detectable** by passive monitoring at the
  default probe speed.  They are recorded as blind spots rather than tuned until
  noise lands on the right answer.
