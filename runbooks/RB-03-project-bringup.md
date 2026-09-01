# RB-03 - Project bring-up

**Goal**: this repository runs; the mock suite is green; every Isaac API
assumption is verified.
**Time**: 30 min.  **Needs**: RB-02 passed.

memo.txt section 9.  Isaac Lab is **not** modified -- this is a separate project
that imports it.

## 1. Get the code

```bash
cd ~/robotics
git clone <this repo> reflex_quad     # or copy the directory across
cd reflex_quad
```

## 2. Two environments, on purpose

| environment | for | contains |
|---|---|---|
| `~/robotics/env_isaaclab` | Isaac runs | Isaac Sim, Isaac Lab, CUDA torch, + this project |
| `.venv` in the repo | mock runs, tests, evaluation | numpy, matplotlib, pyyaml, pytest |

The second one takes ten seconds to build and lets you run the whole test suite
and the evaluator without touching Isaac.  Use it for everything except an
actual Isaac experiment.

```bash
# development / mock environment
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -e ".[dev]"

# and make this project importable inside the Isaac environment
source ~/robotics/env_isaaclab/bin/activate
uv pip install -e .
```

## 3. Mock smoke test

```bash
.venv/bin/python -m pytest -q
```

Expect all tests to pass in under a minute.  They cover the observation
isolation rule, kinematics, the servo model, the sensors, the observer, the
config loader, the evaluator, and short end-to-end runs of five experiments.

**If any test fails, stop here.**  A failing mock suite means the CUDA machine
will only tell you the same thing more slowly.

Then one real run:

```bash
.venv/bin/python -m reflex_quad 01_stand --backend mock --eval
```

Expect `=== 01_stand  [PASS]` and a run directory under `logs/`.

## 4. Verify the Isaac API before using it

```bash
source ~/robotics/env_isaaclab/bin/activate
python scripts/isaac_preflight.py
```

This checks, one at a time and printing PASS/FAIL for each:

1. torch sees CUDA and names the GPU
2. `import isaacsim`
3. `AppLauncher` starts the app
4. the `isaaclab` modules this project imports
5. the URDF generator writes `assets/reflex_quad.urdf`
6. `UrdfConverter` turns it into USD
7. the scene builds; it prints the articulation's joint and body names
8. 100 physics steps run; it prints joint angles, contact forces and body IMU

Expect `8/8 checks passed`.

**Read check 7's output.**  It prints Isaac's own joint order.  The backend
remaps it to this project's fixed order, and a missing joint raises there -- but
seeing the list once tells you the URDF converted the way you meant.

If a check fails with `AttributeError`, an Isaac Lab API name has moved.  Fix it
in `reflex_quad/backends/isaaclab.py` (the assumptions are marked `# VERIFY`),
note the Isaac Lab version in the experiment note, and rerun the preflight.
Do not work around it in the controller.

## 5. First Isaac run

```bash
python -m reflex_quad 01_stand --backend isaaclab --duration 5 --eval
```

Slower than mock, and that is expected.  If it passes, the whole pipeline works
on the real simulator and you can go on to RB-04.

## Done when

- `pytest` is green in `.venv`
- `01_stand` passes on the mock backend
- `isaac_preflight.py` reports 8/8
- `01_stand` runs on the Isaac backend (pass or fail -- RB-04 judges it)
