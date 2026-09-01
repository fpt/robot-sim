# CLAUDE.md

Guidance for working in this repository.

## What this is

A quadruped simulation study whose defining constraint is that the controller
never sees joint angle, joint velocity or joint torque.  `docs/SPEC.md` is the
requirements document; every reference of the form "section 24" means a section
of that file.  Read it before changing behaviour.

## The rule that must not be broken

Nothing under `reflex_quad/controller.py`, `observer.py`, `objective.py`,
`dither.py`, `state_machine.py`, `support.py` or the detector half of
`faults.py` may read `q`, `qd`, `tau`, true poses or true contact forces.

If you need one of those to debug, log it to `truth.csv` and look at it in
`eval/`.  Do not pass it to a controller "temporarily" -- a run that does is
worthless, and it is not obvious from the output that it happened.

Enforcement: `Observation` has no field for them; `eval/isolation.py` scans the
source; `runner._assert_isolated` checks at run time;
`tests/test_observation_isolation.py` covers all three.

## Two backends, one interface

* `backends/mock.py` -- numpy, 3 DOF (heave, roll, pitch), runs anywhere.
* `backends/isaaclab.py` -- Isaac Sim 5.1 / Isaac Lab, needs the CUDA machine.

Everything else is shared.  **Develop and test against the mock**; it takes
seconds.  Read `docs/MOCK_BACKEND.md` for what it does not model (no horizontal
motion, no yaw, no slipping, no tipping sideways) before drawing a conclusion
from it.

The Isaac backend cannot be executed or tested on a machine without an NVIDIA
GPU.  Do not claim it works; the honest statement is that its API assumptions
are marked `# VERIFY` and checked by `scripts/isaac_preflight.py` on the target
machine.

## Working here

```bash
uv venv --python 3.11 .venv                       # the user prefers uv
uv pip install --python .venv/bin/python -e ".[dev]"
.venv/bin/python -m pytest -q                     # ~25 s, run it before and after
.venv/bin/python -m reflex_quad 03_dither --backend mock --eval
```

* Tuning values belong in `config/*.yaml`, never as literals in code.
* Pass/fail thresholds belong in `config/criteria.yaml`.  Changing one is a
  research decision: change it in its own commit, with the reason.
* A new metric goes in `eval/metrics.py` **and** `docs/METRICS.md`.  Add names,
  do not rename them -- `config/criteria.yaml` refers to them.
* New experiments are config entries, not new controllers.  A new *controller*
  goes in `CONTROLLERS` in `runner.py`.

## When something surprises you

This project's main output so far is `docs/FINDINGS.md`: fourteen numbered
results, several of which are cases where the specified approach *looks* like it
works while measuring the wrong thing.  If you find another one:

1. Measure it -- a number and how you got it, not an impression.
2. Add a numbered entry to `docs/FINDINGS.md` with the measurement.
3. Reference it from the code comment and from the relevant runbook.
4. If it changes a threshold or a default, say so in both places.

Comments in this repository explain *why*, especially where the code departs
from the spec.  Keep that style: a comment saying what the line does is noise, a
comment saying which approach was tried and what number ruled it out is the
reason the next person does not repeat the day.

## Known open items

* Fault cases B (friction) and C (delay) are not detectable by the passive
  monitor -- see FINDINGS #11.  Both want the fast, known-waveform probe of the
  section 38 self-check.  That routine is scaffolded (`07_self_check`) but does
  not yet record or compare signatures.
* `06_first_step` can only be judged on Isaac: the mock cannot translate.
* The crawl gait (section 40) is not implemented; `state_machine.legs` is the
  hook for it, and transitions must stay event-driven, not timed.

## Things not to add yet

Cameras, LiDAR, RTX sensors, reinforcement learning, parallel environments,
ROS 2.  Section 1 of the spec excludes them on purpose: each one changes what a
failure means, and phase 1 has not been demonstrated on real physics yet.
