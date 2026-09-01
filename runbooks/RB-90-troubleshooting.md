# RB-90 - Troubleshooting

## Install

| symptom | cause | fix |
|---|---|---|
| `No module named pip` during `./isaaclab.sh --install` | venv made without `--seed` | `uv venv --python 3.11 --seed` and reinstall (RB-01 step 3) |
| uv cannot resolve `isaacsim` | uv's default first-index strategy | add `--index-strategy unsafe-best-match` (RB-01 step 4) |
| `torch.cuda.is_available()` is `False` | CPU wheel installed | check `torch.version.cuda` is not `None`; reinstall with `--index-url https://download.pytorch.org/whl/cu128` |
| Isaac Sim first launch hangs on a prompt | EULA | `export OMNI_KIT_ACCEPT_EULA=YES` |
| extension build errors | headers missing | `sudo apt install python3.11-dev` |
| disk fills mid-install | extension cache | 60 GB free before starting |

## Isaac runtime

| symptom | cause | fix |
|---|---|---|
| `AttributeError` on an Isaac Lab object | API moved between versions | `python scripts/isaac_preflight.py` names the check that failed; fix the `# VERIFY` line in `reflex_quad/backends/isaaclab.py` and record the version |
| out of VRAM | too much scene | stay headless, one environment, no cameras (`docs/ISAAC_NOTES.md`) |
| articulation missing joints | URDF conversion merged the fixed foot joints | `merge_fixed_joints: false` -- the foot links carry the sensors |
| `RuntimeError: articulation is missing joints [...]` | joint names differ | the URDF names them `FL_hip` etc.; regenerate with `python -m reflex_quad.asset_builder` |
| contact forces always zero | sensor on the wrong prim, or feet merged away | preflight check 8 prints them; check the prim path matches a real body name |
| runs but very slow | GUI, or physics at 1 kHz | `isaac.headless: true`, `physics_dt: 0.002` |

## Physics and control

| symptom | first thing to look at |
|---|---|
| robot sinks into the floor | contact stiffness / solver iterations; `tau_max` too low to hold the weight |
| bounces or chatters | contact damping; `physics_dt: 0.001`; `friction.stick_velocity` if using phase D |
| tips over immediately | joint axis sign -- compare `assets/reflex_quad.urdf` `<axis>` against `reflex_quad/robot.py` |
| one foot never loads | it is not touching; check terrain block placement and `stance.height` |
| currents pinned at the limit | `tau_max` too low, or the stance is too extended |
| posture oscillates | lower the **force** gains first: that loop closes through contact stiffness |
| a leg will not unload | FINDINGS #5, #6, #7 -- posture mask, reachable `F*`, weight shift |

## Dither

| symptom | look at |
|---|---|
| `dJ` scatters around zero | `delta` too small (FINDINGS #2); check `06_dither.png` |
| gradient sign flips constantly | `settle_time` too short, or raise `repeats` |
| converges to a crouch | `w_power` too high -- "carry nothing" is cheapest |
| step size collapses | RPROP shrinking on noise; raise `rprop_step_min` or `repeats` |
| no progress with two feet down | not a bug: `J_force` is exactly flat in two-point support (FINDINGS, RB-06) |

## Fault monitor

| symptom | fix |
|---|---|
| flags everywhere | is it running through a leg lift? `fault_monitor.active_states` |
| nothing detected | is the fault modelled at this servo phase?  B needs D, C needs C |
| detected on the wrong joint | detection needs the joint to be *driven*; single-joint probe the suspect |
| high false-alarm rate | run with `faults: []`, measure the spread, set thresholds above it |

## Evaluation

| symptom | fix |
|---|---|
| `MISSING` in the criteria table | the metric is not produced for that controller -- check `docs/METRICS.md` and the criteria entry |
| everything `NO-CRITERIA` | the experiment id has no block in `config/criteria.yaml` |
| plots empty | run has no `J_total`/`dither_*` columns; that controller does not produce them |

## When you are properly stuck

1. Reproduce it on the **mock** backend.  If it reproduces, it is not Isaac.
2. `.venv/bin/python -m pytest -q` -- 60 tests, under a minute.
3. Bisect the fidelity ladder: does stage 1 do it too?
4. Compare `control.csv` against `truth.csv`.  The gap between what the robot
   believed and what was true is usually the whole story.
