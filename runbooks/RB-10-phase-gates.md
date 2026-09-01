# RB-10 - The whole suite, the fidelity ladder, the phase gates

**Goal**: a single verdict for each of memo.txt's phases, at a stated level of
realism.
**Time**: 2-4 h.  **Needs**: RB-04 to RB-09 individually passing.

memo.txt sections 45-48, plus the four-stage ladder from the design discussion.

## The fidelity ladder

`config/fidelity.yaml`.  Do not skip rungs: if the maths does not converge with
a perfect servo and perfect sensors, nothing below it will, and you will spend
the day tuning noise instead of the control law.

| stage | servo | sensors | question |
|---|---|---|---|
| 1 | ideal PD (phase A) | noiseless, unbiased | does the maths converge at all? |
| 2 | + torque and speed ceilings (B) | noiseless | does a cheap servo still manage? |
| 3 | + deadband, delay, friction, backlash (D) | noisy and biased | is it realistic? |
| 4 | + parameter randomisation (E) | noisy and biased | how much sim-to-real margin is there? |

```bash
for s in 1 2 3 4; do
  .venv/bin/python -m reflex_quad 03_dither --backend mock --fidelity $s --tag f$s
done
.venv/bin/python -m eval.cli --all
```

The stage where a run stops passing is the finding.  Record it.

## Full suite

```bash
# mock, everything, ~8 minutes
for e in 01_stand 02_uneven_ground 03_dither 03b_dither_all \
         04_leg_unload 05_fault 06_first_step 07_self_check; do
  .venv/bin/python -m reflex_quad "$e" --backend mock --quiet
done

.venv/bin/python -m eval.cli --all --phase phase1 --phase phase2 --phase phase3 --phase phase4
```

On Isaac, same list with `--backend isaaclab` in the Isaac environment.  Run it
overnight if need be; it is the same command.

## The phase gates

From `config/criteria.yaml`:

* **phase 1** (memo 45) -- `01_stand`, `02_uneven_ground`, `03_dither` all pass,
  **and** `isolation_violations == 0`.  Levelling on unknown terrain without ever
  seeing a joint angle.  This is the milestone the whole project is built for.
* **phase 2** (memo 46) -- `04_leg_unload`.  One leg through the full cycle,
  verified by its own IMU.
* **phase 3** (memo 47) -- `05_fault`.  Faults found from inconsistency alone.
  Note the two documented blind spots (RB-08).
* **phase 4** (memo 48) -- `06_first_step`, on the way to a crawl gait.

## The isolation check is part of phase 1 and that is deliberate

```bash
.venv/bin/python -c "from eval.isolation import check_isolation; print(check_isolation() or 'clean')"
```

It scans the controller-side modules for any mention of joint truth.  A phase 1
pass with a violation in it would be a run that quietly used the thing the whole
experiment is about not using.  If it ever fires, the run is void.

## Repeats and seeds

One run is an anecdote.  Before claiming a phase:

```bash
.venv/bin/python -m reflex_quad 03_dither --backend mock --repeat 5 --seed 100
.venv/bin/python -m eval.cli logs/03_dither_*s*
```

Different seeds mean different sensor noise, biases and FSR gain mismatch.  A
criterion that passes on 5/5 is a result; 3/5 is a tuning job, and worth saying
so plainly in the note.

## What to write down

For each phase: pass/fail, the runs behind it, the fidelity stage, the backend,
the Isaac Lab commit, and every config value that differs from what is committed.
`report.md` in each run directory has the tables; the note is where they get
tied together into a claim.
