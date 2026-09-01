# RB-02 - Isaac Lab

**Goal**: `import isaaclab` works against the Isaac Sim from RB-01, and one
simulation actually steps.
**Time**: 15-60 min depending on route.  **Needs**: RB-01 passed, venv active.

docs/SPEC.md sections 7, 8.

## Two routes

| | route A - pip package | route B - from source |
|---|---|---|
| get | `uv pip install isaaclab` | `git clone` + `./isaaclab.sh --install` |
| gives you | the library | the library, the tutorials, the RL scripts |
| time | ~10 min | ~40 min |
| this project needs | **this one** | only if you want the samples |

`reflex_quad` imports `isaaclab` and nothing else -- no `isaaclab.sh`, no
`scripts/`, no task registry.  **Route A is the one to take.**  SPEC sections 7
and 8 describe route B because that was the documented path when the plan was
written; the pip package is now the documented default and it is less to go
wrong.

Take route B as well if you want NVIDIA's tutorials to compare against, which is
a reasonable thing to want the first time.  They coexist: route B installs in
editable mode and shadows the wheel.

## Route A - pip package

```bash
source ~/robotics/env_isaaclab/bin/activate
uv pip install "isaaclab[all]" \
    --extra-index-url https://pypi.nvidia.com \
    --index-strategy unsafe-best-match
```

Pin it once you have a version that works:

```bash
uv pip install "isaaclab[all]==2.3.2.post1" --extra-index-url https://pypi.nvidia.com
```

NVIDIA also publishes a combined extra, `isaaclab[isaacsim,all]`, which pulls
Isaac Sim too.  Do not use it here -- RB-01 already installed Isaac Sim
explicitly, and letting two commands both own that dependency is how you end up
with a mystery version.

### Verify

```bash
python -c "import isaaclab, isaacsim; print(isaaclab.__version__, isaacsim.__version__)"
```

Then check it can actually step physics, not merely import:

```bash
python scripts/isaac_preflight.py
```

That is RB-03's script and it does the real work -- app launch, URDF conversion,
articulation, sensors, 100 steps.  Running it here is the honest end of RB-02.

## Route B - from source

```bash
cd ~/robotics
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git log -1 --format='%H %cd'      # record this hash
```

Isaac Lab's `main` moves.  The hash is the difference between "it worked
yesterday" and a debuggable report.

```bash
source ~/robotics/env_isaaclab/bin/activate    # must be active
./isaaclab.sh --install
```

This shells out to `python -m pip`, which is why RB-01 created the venv with
`--seed`.  If you see `No module named pip`, that is the cause.

Do **not** modify Isaac Lab to suit this project.  Everything specific to this
work lives in this repository; that separation is SPEC section 9 and it is what
makes an Isaac Lab upgrade a small event.

Run a sample:

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

If that path has moved in your checkout:

```bash
find scripts -iname "*.py" | head -30
```

Add `--headless` if the window is a nuisance.

## Record

```bash
python -c "import isaaclab, isaacsim; print(isaaclab.__version__, isaacsim.__version__)"
cd ~/robotics/IsaacLab 2>/dev/null && git log -1 --format='%H'   # route B only
```

Both versions, the route you took, and the commit hash if any, into the
experiment note.  SPEC section 8 asks for this and it costs ten seconds.

## Done when

- `import isaaclab` succeeds and prints a version
- something stepped physics: `isaac_preflight.py`, or a tutorial on route B
- the versions are written down

## If it fails

| symptom | cause | fix |
|---|---|---|
| `No module named pip` | venv made without `--seed` | recreate it (RB-01 step 3) and reinstall |
| uv cannot resolve `isaaclab` | default first-index strategy | add `--index-strategy unsafe-best-match` |
| `import isaacsim` fails after installing isaaclab | two commands fought over the Isaac Sim version | uninstall both, redo RB-01 step 4, then route A without the `isaacsim` extra |
| a build error in an extension | headers missing | `python3.11-dev` (RB-01 step 1) |
| `module 'omni.usd' has no attribute 'UsdContext'` | stale Kit extension cache | delete `<venv>/lib/python3.11/site-packages/isaacsim/extscache` and relaunch |
| the sample opens a window then dies | display or driver | try `--headless`; if that works it is not Isaac Lab, and this project runs headless anyway |
| `uv run` cannot find isaaclab | it picked this repo's `.venv` | `uv run --active`, or call `python` directly (RB-01, "the `uv run` trap") |

## Sources

- [Isaac Lab: installation using Isaac Lab pip packages](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/isaaclab_pip_installation.html)
- [Isaac Lab: installation using the Isaac Sim pip package](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)
