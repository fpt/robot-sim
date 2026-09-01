#!/usr/bin/env bash
# Record the machine this experiment ran on.  memo.txt section 2.
# Writes logs/host_<date>.md and prints it.
set -uo pipefail
out="logs/host_$(date +%Y%m%d_%H%M%S).md"
mkdir -p logs
{
  echo "# Host record - $(date +%Y-%m-%dT%H:%M:%S)"
  echo
  echo '```text'
  printf 'Ubuntu:  '; (lsb_release -ds 2>/dev/null || cat /etc/os-release 2>/dev/null | head -1 || sw_vers -productVersion)
  printf 'Kernel:  '; uname -r
  printf 'glibc:   '; (ldd --version 2>/dev/null | head -1 || echo 'n/a')
  printf 'CPU:     '; (lscpu 2>/dev/null | awk -F: '/Model name/{print $2; exit}' | xargs || sysctl -n machdep.cpu.brand_string 2>/dev/null)
  printf 'RAM:     '; (free -h 2>/dev/null | awk '/Mem:/{print $2}' || echo 'n/a')
  printf 'GPU:     '; (nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'no nvidia-smi')
  printf 'VRAM:    '; (nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null || echo 'n/a')
  printf 'Driver:  '; (nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null || echo 'n/a')
  printf 'CUDA:    '; (nvcc --version 2>/dev/null | tail -1 || echo 'nvcc not on PATH')
  printf 'Python:  '; (python --version 2>&1 || python3 --version 2>&1)
  printf 'uv:      '; (uv --version 2>/dev/null || echo 'not installed')
  printf 'torch:   '; (python || echo python3) >/dev/null 2>&1; ${PYTHON:-python3} -c "import torch;print(torch.__version__, 'cuda', torch.cuda.is_available())" 2>/dev/null || echo 'torch not importable'
  printf 'isaacsim:'; ${PYTHON:-python3} -c "import isaacsim;print(getattr(isaacsim,'__version__','?'))" 2>/dev/null || echo ' not importable'
  printf 'git:     '; git rev-parse --short HEAD 2>/dev/null || echo 'not a repo'
  echo '```'
} | tee "$out"
echo
echo "saved to $out"
