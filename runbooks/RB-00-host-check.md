# RB-00 - Host check

**Goal**: know exactly what machine this is, and write it down.
**Time**: 10 minutes.  **Needs**: nothing.

memo.txt section 2.

## 1. GPU

```bash
nvidia-smi
```

Expect a table naming `NVIDIA GeForce RTX 4070` and a driver version.

- No such command -> the proprietary driver is not installed.  Install it
  (`sudo ubuntu-drivers install` or the NVIDIA `.run` for your kernel), reboot,
  and try again.  Nouveau will not do.
- The GPU appears but `nvidia-smi` reports `ERR!` for memory -> a driver/kernel
  mismatch after an update; reinstall the driver for the running kernel.

**Note the 12 GB.**  Isaac Sim 5.1's documented minimum is an RTX 4080 / 16 GB.
This project stays inside a 4070 by staying headless, camera-free and
single-environment.  See `docs/ISAAC_NOTES.md`.

## 2. OS and libc

```bash
lsb_release -a
uname -r
ldd --version
```

Ubuntu 22.04 or 24.04 is the safe ground for Isaac Sim 5.1.

## 3. Disk and memory

```bash
df -h ~
free -h
```

The pip install of Isaac Sim plus its extension cache runs to **tens of GB**.
Have 60 GB free before starting RB-01.  128 GB of RAM is plenty.

## 4. Record it

```bash
bash scripts/check_host.sh
```

Writes `logs/host_<timestamp>.md` with OS, kernel, driver, GPU, VRAM, RAM,
CUDA, Python, torch and the git hash.  Commit it, or paste it into the
experiment note.  Every later "it used to work" question starts here.

## Done when

`logs/host_*.md` exists and shows the 4070 and a driver version.
