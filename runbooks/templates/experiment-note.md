# Experiment note

**Date**:
**Operator**:
**Session goal**:

## Machine

Paste the relevant `logs/host_*.md`, or:

```text
Ubuntu:
Kernel:
NVIDIA driver:
GPU:
VRAM:
RAM:
```

## Software

```text
Isaac Sim:
Isaac Lab commit:
torch:
reflex_quad commit:
backend:            mock | isaaclab
fidelity stage:     1 | 2 | 3 | 4
```

## Runs

| experiment | run directory | verdict | note |
|---|---|---|---|
| | | | |

## Config changes from what is committed

Every value you changed, and **why**.  Gains are results.

| file | key | committed | used | reason |
|---|---|---|---|---|
| | | | | |

## Numbers

```text
final roll / pitch:
four foot forces:
force CV:
J first / last:
attitude estimate RMSE vs truth:
```

## Phase verdict

- [ ] phase 1 - levelling on unknown terrain, no joint feedback (memo 45)
- [ ] phase 2 - one leg unload / lift / verify / lower / reload (memo 46)
- [ ] phase 3 - fault detection from observations only (memo 47)
- [ ] phase 4 - static crawl gait (memo 48)

## What surprised me

The most valuable section.  Anything that did not match expectation, whether or
not it was fixed.  If it is reproducible and general, move it into
`docs/FINDINGS.md`.

## Next

1.
2.
