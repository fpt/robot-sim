# RB-02 - Isaac Lab

**Goal**: Isaac Lab installed against the Isaac Sim from RB-01, one sample
running.
**Time**: 30-60 min.  **Needs**: RB-01 passed, venv active.

memo.txt sections 7, 8.

## 1. Clone

```bash
cd ~/robotics
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git log -1 --format='%H %cd'      # record this hash in the experiment note
```

Isaac Lab's `main` moves.  The hash is the difference between "it worked
yesterday" and a debuggable report.

## 2. Install

```bash
source ~/robotics/env_isaaclab/bin/activate    # must be active
./isaaclab.sh --install
```

This runs `python -m pip install -e ...` over the source packages -- hence the
seeded venv from RB-01 step 3.

Do **not** change Isaac Lab's own configuration to suit this project.  Keep the
stock install; everything specific to this work lives in this repository.

## 3. Run a sample

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

If the path has moved in your checkout:

```bash
find scripts -iname "*.py" | head -30
```

and run the smallest `00_sim` example you find.  Add `--headless` if the window
is a nuisance.

Expect: the app starts, prints simulation steps, exits cleanly.

## 4. Record the state that works

```bash
cd ~/robotics/IsaacLab && git log -1 --format='%H'
python -c "import isaaclab, isaacsim; print(isaaclab.__version__, isaacsim.__version__)"
```

Put both in the experiment note.  memo.txt section 8 asks for this and it is
worth the ten seconds.

## Done when

- `./isaaclab.sh --install` finished without errors
- a tutorial script ran and exited 0
- the Isaac Lab commit hash and both versions are written down

## If it fails

- `No module named pip` -> the venv was made without `--seed`.  Recreate it
  (RB-01 step 3) and reinstall.  This is the most common failure.
- A build error in an extension -> `python3.11-dev` missing (RB-01 step 1).
- The sample opens a window then dies -> try `--headless`; if that works, it is
  a display/driver issue, not Isaac Lab, and this project does not need the GUI.
