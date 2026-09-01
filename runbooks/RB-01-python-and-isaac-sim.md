# RB-01 - Python 3.11, Isaac Sim 5.1, CUDA PyTorch

**Goal**: `import isaacsim` and `torch.cuda.is_available() == True`.
**Time**: 1-2 hours, mostly download.  **Needs**: RB-00 passed, ~60 GB free.

docs/SPEC.md sections 3-6.  This runbook uses **uv**.  That is not a
workaround: Isaac Lab's own installation page lists uv alongside conda and venv,
marked *experimental*, with exactly the command below.  The package set is
identical either way -- uv creates a standard venv and installs the same wheels.

## 1. System packages

```bash
sudo apt update
sudo apt install -y git cmake build-essential curl \
    python3.11 python3.11-venv python3.11-dev
```

`python3.11-dev` is not in the memo's list but Isaac Lab builds extensions
against the headers.

## 2. uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
exec $SHELL -l
uv --version
```

## 3. Workspace and virtual environment

```bash
mkdir -p ~/robotics && cd ~/robotics
uv venv --python 3.11 --seed env_isaaclab
source env_isaaclab/bin/activate
python --version        # expect Python 3.11.x
```

**`--seed` matters**, and NVIDIA's docs say so too: a venv made by `uv venv`
has no `pip`, and parts of the Isaac install shell out to `python -m pip`.
Without it you get a bare "No module named pip".  uv can create the environment;
it cannot make someone else's installer stop using pip.

**Python 3.11 is not a preference.**  The venv's Python must match Isaac Sim's:
3.11 for Isaac Sim 5.X, 3.12 for 6.X.  This project targets 5.1 (SPEC section 1)
so 3.11 it is.  See "Isaac Sim 6" at the bottom if you would rather move up.

If uv does not have a 3.11 interpreter it will fetch one -- that is fine and is
independent of the system `python3.11`.

### The `uv run` trap

This repository has its own `.venv` (RB-03) for mock runs and tests.  `uv run`
inside the repo prefers *that* environment, not the activated Isaac one, so an
Isaac experiment launched with a bare `uv run` will fail on `import isaacsim`.

Either call the interpreter directly (what every runbook here does):

```bash
python -m reflex_quad 01_stand --backend isaaclab
```

or force uv to respect the activated environment:

```bash
uv run --active python -m reflex_quad 01_stand --backend isaaclab
```

## 4. Isaac Sim 5.1

```bash
export OMNI_KIT_ACCEPT_EULA=YES
uv pip install "isaacsim[all,extscache]==5.1.0" \
    --extra-index-url https://pypi.nvidia.com \
    --index-strategy unsafe-best-match
```

Two flags need explaining:

* `OMNI_KIT_ACCEPT_EULA=YES` -- otherwise the first launch stops on a prompt.
* `--index-strategy unsafe-best-match` -- uv's default only looks at the first
  index that has a package name at all.  Isaac's requirements are spread across
  PyPI and `pypi.nvidia.com`, so the default strategy misses versions.  The flag
  name is alarming and it means "consider all indexes"; only use it with index
  URLs you trust, which these are.

This downloads tens of GB.  Let it finish.

**If a large wheel fails to download** (`nvidia-cublas` is the usual victim),
clear just that entry rather than the whole cache and retry:

```bash
uv cache clean nvidia-cublas
```

A full `uv cache clean` throws away everything else you just downloaded.

## 5. CUDA PyTorch

```bash
uv pip install -U torch==2.7.0 torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128
```

`--index-url` (not `--extra-`) on purpose: it *replaces* the default index so
you get the CUDA 12.8 wheels rather than the CPU ones from PyPI.

## 6. Verify

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

Expected:

```text
2.7.0+cu128
True
NVIDIA GeForce RTX 4070
```

**If this prints `False`, stop.**  Do not continue to Isaac Lab.  In order:

1. `nvidia-smi` still works? If not, the driver broke -- back to RB-00.
2. `python -c "import torch; print(torch.version.cuda)"` -- if it prints `None`
   you installed a CPU wheel; redo step 5 and watch the index URL.
3. Driver too old for CUDA 12.8 -> update the driver, or drop to a cu121 wheel
   set and record that deviation in the experiment note.

## 7. Isaac Sim starts

```bash
isaacsim
```

First launch takes several minutes: it compiles shaders and fills the extension
cache.  A window appears; close it.

On this GPU, prefer headless from here on -- the runbooks all pass
`isaac.headless: true`.  A GUI launch is for looking at the robot, not for
running experiments.

## Done when

- `torch.cuda.is_available()` is `True` and names the 4070
- `isaacsim` opened and closed cleanly once
- both recorded in `logs/host_*.md` (rerun `bash scripts/check_host.sh`)

---

## Alternative: uv project mode

The steps above use `uv pip` into an activated venv, which mirrors NVIDIA's
documented flow one command at a time.  uv's project mode gives you a lockfile
instead, which is worth it if you expect to rebuild this environment:

```bash
uv init --python 3.11 --package isaac_env && cd isaac_env
uv add "isaaclab[isaacsim,all]" omniverse-kit
```

with the indexes declared in `pyproject.toml`:

```toml
[[tool.uv.index]]
name = "nvidia"
url = "https://pypi.nvidia.com"
explicit = true

[[tool.uv.index]]
name = "pytorch-cu128"
url = "https://download.pytorch.org/whl/cu128"
explicit = true

[tool.uv.sources]
torch = { index = "pytorch-cu128" }
torchvision = { index = "pytorch-cu128" }
isaacsim = { index = "nvidia" }
isaaclab = { index = "nvidia" }
```

`explicit = true` on both is the part that matters: without it uv is free to
resolve `torch` from PyPI and you get a CPU wheel, which fails silently until
`torch.cuda.is_available()` says `False`.

Same result, different bookkeeping.  Pick one and record which in the
experiment note.

## Alternative: Isaac Sim 6.x

Isaac Sim 6.0 exists and wants **Python 3.12** and **torch 2.11.0+cu128**:

```bash
uv venv --python 3.12 --seed env_isaaclab
uv pip install "isaacsim[all,extscache]==6.0.0.1" \
    --extra-index-url https://pypi.nvidia.com \
    --index-strategy unsafe-best-match --prerelease=allow
uv pip install -U torch==2.11.0 torchvision==0.26.0 \
    --index-url https://download.pytorch.org/whl/cu128
```

This project does not require 5.1 -- nothing in `reflex_quad/` is version
specific, and `pyproject.toml` asks only for `>=3.11`.  SPEC section 1 names 5.1
because that is what was current when the plan was written.

Staying on 5.1 means following a documented, widely-used path.  Moving to 6.x
means fewer people have hit your bugs before you.  Either way run
`scripts/isaac_preflight.py` (RB-03) first -- it exists precisely so an API
change announces itself in 60 seconds -- and write the version in the note.

## X11 packages, even headless

```bash
sudo apt install -y libx11-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev
```

Kit links against these.  Without them the install succeeds and the viewer fails
to start later, which is a confusing way to find out.

## Sources

- [Isaac Lab: installation using Isaac Lab pip packages](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/isaaclab_pip_installation.html)
- [Isaac Lab: installation using the Isaac Sim pip package](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)
