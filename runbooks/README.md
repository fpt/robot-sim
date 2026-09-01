# Runbooks

Ordered.  Each one states what it is for, what must already be true, the exact
commands, what "worked" looks like, and what to do when it does not.  Every
runbook ends by telling you what to write in the experiment note
(`runbooks/templates/experiment-note.md`).

| # | file | takes | what it gets you |
|---|---|---|---|
| 00 | [RB-00-host-check.md](RB-00-host-check.md) | 10 min | the machine is what you think it is, recorded |
| 01 | [RB-01-python-and-isaac-sim.md](RB-01-python-and-isaac-sim.md) | 1-2 h | Python 3.11 + Isaac Sim 5.1 + CUDA PyTorch, via uv (uv is a documented Isaac Lab option, marked experimental) |
| 02 | [RB-02-isaac-lab.md](RB-02-isaac-lab.md) | 15-60 min | Isaac Lab installed (pip package or source) and stepping |
| 03 | [RB-03-project-bringup.md](RB-03-project-bringup.md) | 30 min | this repo running, mock suite green, Isaac API verified |
| 04 | [RB-04-exp01-stand.md](RB-04-exp01-stand.md) | 20 min | it stands up (memo 24) |
| 05 | [RB-05-exp02-uneven.md](RB-05-exp02-uneven.md) | 30 min | it levels itself on an unseen step (memo 25, 26) |
| 06 | [RB-06-exp03-dither.md](RB-06-exp03-dither.md) | 1-2 h | **active sensing works** (memo 27-31) |
| 07 | [RB-07-exp04-leg-unload.md](RB-07-exp04-leg-unload.md) | 1 h | one leg up, verified by its own IMU (memo 32-34) |
| 08 | [RB-08-exp05-fault.md](RB-08-exp05-fault.md) | 1 h | faults found from observations only (memo 35-37) |
| 09 | [RB-09-exp06-first-step.md](RB-09-exp06-first-step.md) | 1 h | one step (memo 39) |
| 10 | [RB-10-phase-gates.md](RB-10-phase-gates.md) | 2-4 h | the full suite, the fidelity ladder, the phase verdicts |
| 90 | [RB-90-troubleshooting.md](RB-90-troubleshooting.md) | -- | when something breaks |

## How to use these

1. Do them in order.  Each assumes the previous one passed.
2. Run everything on the **mock backend first** (`--backend mock`).  It takes
   seconds and catches config and logic errors before GPU time is spent.
3. Then the same command with `--backend isaaclab`.
4. Let the evaluator decide pass/fail (`--eval`), not your eye on a plot.  The
   thresholds live in `config/criteria.yaml`; if you disagree with one, change it
   there **in a commit with a reason**, so the bar is visible.
5. Copy `templates/experiment-note.md` per session and fill it in.  A run you
   cannot reproduce did not happen.

## The one rule

The controller never sees joint angle, joint velocity or joint torque
(memo.txt section 13).  It is enforced three ways: `Observation` has no field
for them, `eval/isolation.py` scans the controller-side source, and the runner
refuses a controller that holds a reference to the backend.  If you ever find
yourself wanting to pass `q` "just for debugging", write it to `truth.csv` and
look at it in the evaluator instead.
