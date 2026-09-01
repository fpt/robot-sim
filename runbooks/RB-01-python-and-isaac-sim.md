# RB-01 - Python 3.11, Isaac Sim 5.1, CUDA PyTorch

**Goal**: `import isaacsim` and `torch.cuda.is_available() == True`.
**Time**: 1-2 hours, mostly download.  **Needs**: RB-00 passed, ~60 GB free.

memo.txt sections 3, 4, 5, 6.  This runbook uses **uv** rather than
`python3.11 -m venv` + `pip`; the package set is identical.

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

**`--seed` matters.**  It installs `pip` into the venv.  Isaac Lab's
`./isaaclab.sh --install` (RB-02) shells out to `python -m pip`, and without a
seeded venv it fails with a bare "No module named pip".  uv can create the
environment; it cannot make Isaac Lab's installer stop using pip.

If uv does not have a 3.11 interpreter it will fetch one -- that is fine and is
independent of the system `python3.11`.

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

This downloads several GB.  Let it finish.

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
