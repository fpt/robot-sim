"""Target load distribution F* from the support geometry.

memo_full.txt asks for F* to be ramped so the other legs take the load, but a
uniform target is the wrong thing to ask for.  With one leg off the ground the
remaining three must satisfy three equilibrium equations,

    sum F_i = W        sum F_i y_i = 0        sum F_i x_i = 0

which for a symmetric quadruped forces the diagonal partner of the lifted leg to
carry about half the weight.  Commanding W/3 on each of the other three is
physically unreachable, and a load-balance loop that chases it saturates and
destroys the posture instead (this is exactly what experiment 04 did before
this module existed -- see docs/FINDINGS.md).

The positions used here are the *commanded* foot positions, which the controller
computed itself from its own IK.  No joint measurement is involved.
"""
from __future__ import annotations

import numpy as np

from . import N_LEGS


def target_forces(
    foot_x: np.ndarray,
    foot_y: np.ndarray,
    weight: float,
    lifted: int | None = None,
    com_xy: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    """Even-as-possible load split that still balances the body.

    Least-norm solution of  A F = b  around the uniform split, then clamped to
    non-negative (a foot can push but not pull).
    """
    x = np.asarray(foot_x, dtype=float) - com_xy[0]
    y = np.asarray(foot_y, dtype=float) - com_xy[1]
    active = np.ones(N_LEGS, dtype=bool)
    if lifted is not None:
        active[int(lifted)] = False
    n = int(active.sum())
    if n == 0:
        return np.zeros(N_LEGS)

    a = np.vstack([np.ones(n), y[active], x[active]])          # (3, n)
    b = np.array([weight, 0.0, 0.0])
    f0 = np.full(n, weight / n)
    f = f0 + np.linalg.pinv(a) @ (b - a @ f0)

    if np.any(f < 0):                 # a foot cannot pull: drop it and re-solve
        keep = f >= 0
        if keep.sum() >= 3:
            idx = np.where(active)[0][keep]
            sub = target_forces(x, y, weight, None) * 0.0
            a2 = np.vstack([np.ones(keep.sum()), y[idx], x[idx]])
            f2 = np.full(keep.sum(), weight / keep.sum())
            f2 = f2 + np.linalg.pinv(a2) @ (b - a2 @ f2)
            sub[idx] = np.maximum(f2, 0.0)
            return sub
        f = np.maximum(f, 0.0)

    out = np.zeros(N_LEGS)
    out[active] = f
    return out
