# RB-08 - Experiment 05: fault detection

**Goal**: detect an injected fault from commands, currents, foot IMUs and foot
forces alone.
**Time**: 1 h.  **Needs**: RB-06 passed (this uses the dither controller).

memo.txt sections 35, 36, 37.

## The five cases

`config/faults.yaml`, injected at t=25 s into a 60 s dithering run.  The first
16 s train the residual baseline -- long enough for one full 8-joint sweep, so
that every joint has a baseline of its own.

| case | fault | expected class | detectable today |
|---|---|---|---|
| A | max torque x0.3 | `drive_failure` | yes, ~7 s |
| B | friction x10 | `mechanical_resistance` | **no** -- see below |
| C | +100 ms command delay | `sluggish_response` | **no** -- see below |
| D | foot FSR stuck | `force_sensor_stuck` | yes, ~1.4 s |
| E | foot IMU bias | `foot_imu_bias` | yes, ~0.9 s |

B and C are marked `expected_detectable: false` in the fault library so the
evaluator reports them as known gaps rather than pretending.  **They are the
next piece of research, not a bug.**  `docs/FINDINGS.md` #10 and #11 have the
measurements: friction is a *velocity* cost and the section 27 probe moves the
joint at only ~0.08 rad/s, and a pure delay needs a lag estimate whose scatter
(+-0.042 s) is comparable to the 0.100 s fault.  Both want the deliberate,
fast, known-waveform probe of the section 38 self-check.

Experiment `05_fault` runs at servo phase D, because "friction x10" does nothing
at phase B and "+100 ms" does nothing before phase C.  A fault case can only
show up if the servo models the thing it breaks.

## Run it

```bash
.venv/bin/python -m reflex_quad 05_fault --backend mock --eval
python -m reflex_quad 05_fault --backend isaaclab --eval
```

Change the case in `config/experiment.yaml`:

```yaml
    faults:
      - {case: D, joint: 1, t_start: 25.0}
```

## Pass criteria

| check | threshold |
|---|---|
| `detected` | something was flagged |
| `detected_in_time` | correct class within 15 s of injection |
| `no_false_alarm` | **zero** flags before the injection |
| `classified` | the class matches `expected_class` |

The latency gate encodes the physics rather than a wish: **detection is only
possible while the joint is being driven**, so the bound is the probe revisit
interval.  An 8-joint sweep at ~1.8 s per joint comes back to any one joint every
~14.4 s, hence 15 s.  Measured: 0.9 s for the foot-IMU bias and 1.4 s for the
stuck FSR (neither needs a probe), 7.2 s for the torque loss (which does).

If you want a faster number, do not lower the threshold -- probe the suspect
joint directly, `03_dither`-style, which brings it under 2 s.  That is exactly
what the section 38 self-check is for.

## The section 36 signatures

```
command sent + current high + foot motion low  ->  mechanical resistance
command sent + current low  + foot motion low  ->  motor / drive failure
foot force high + foot IMU vibration high      ->  slipping contact
foot force perfectly constant while others move ->  sensor stuck
|foot accel| away from its own baseline        ->  IMU bias
```

Two implementation notes that cost a day each to find:

* the monitor only learns and only judges in **quasi-static supported states**
  (`fault_monitor.active_states`).  A leg deliberately in the air is
  indistinguishable from a broken one; running the monitor through experiment 04
  unfiltered produced 260 false detections (FINDINGS #12).
* the tests are **ratios against the baseline mean**, not z-scores.  The
  baseline scatter of these features is the same order as their mean, so
  `z < -4` is unreachable no matter how broken the robot is (FINDINGS #10).

## If Isaac disagrees

`config/experiment.yaml` -> `fault_monitor`:

* `baseline_time` must cover at least one full sweep of whatever the controller
  is doing, or a joint has no baseline of its own
* `current_ratio_high` / `current_ratio_low` / `motion_ratio_max` -- the
  signature thresholds
* `dynamic_current_margin` -- moving-vs-holding current; set it above the noise
  floor you measure, which on the mock is 0.037 A
* `min_consecutive` (40 ticks = 0.4 s) -- a real fault persists, a transient at a
  sweep boundary does not

Measure the noise floor before setting a threshold.  Run the no-fault case
(`faults: []`) and look at the spread; anything you set below that spread buys
false alarms.

## Record

Per case: detected yes/no, class, joint, latency, false alarms before injection.
That table is the deliverable of memo phase 3.
