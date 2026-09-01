#!/usr/bin/env python
"""Check every Isaac assumption reflex_quad makes, one at a time.

Run this on the CUDA machine BEFORE the first experiment (runbooks/RB-03).
Each check prints PASS/FAIL with the real exception, so when an Isaac Lab API
name moves you find out in 60 seconds instead of twenty minutes into a run.

    python scripts/isaac_preflight.py            # headless
    python scripts/isaac_preflight.py --gui
"""
from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

RESULTS: list[tuple[str, bool, str]] = []


def check(name):
    def deco(fn):
        def wrapped(*a, **k):
            try:
                detail = fn(*a, **k) or ""
                RESULTS.append((name, True, str(detail)))
                print(f"[PASS] {name}: {detail}")
                return True
            except Exception as exc:  # noqa: BLE001 - this is the point
                RESULTS.append((name, False, f"{type(exc).__name__}: {exc}"))
                print(f"[FAIL] {name}: {type(exc).__name__}: {exc}")
                if "-v" in sys.argv:
                    traceback.print_exc()
                return False
        return wrapped
    return deco


@check("torch + CUDA")
def check_torch():
    import torch

    assert torch.cuda.is_available(), "torch.cuda.is_available() is False"
    return f"torch {torch.__version__}, {torch.cuda.get_device_name(0)}"


@check("isaacsim import")
def check_isaacsim():
    import isaacsim

    return getattr(isaacsim, "__version__", "version attribute absent")


@check("AppLauncher")
def check_app(headless: bool):
    from reflex_quad.isaac_boot import ensure_app

    ensure_app(headless=headless)
    return "app launched"


@check("isaaclab modules")
def check_modules():
    import isaaclab
    import isaaclab.sim as sim_utils  # noqa: F401
    from isaaclab.actuators import ImplicitActuatorCfg  # noqa: F401
    from isaaclab.assets import Articulation, ArticulationCfg  # noqa: F401
    from isaaclab.sensors import ContactSensor, ContactSensorCfg, Imu, ImuCfg  # noqa: F401
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: F401

    return f"isaaclab {getattr(isaaclab, '__version__', '?')}"


@check("URDF generation")
def check_urdf():
    from reflex_quad.asset_builder import write_urdf

    path = write_urdf()
    return f"{path} ({path.stat().st_size} bytes)"


@check("URDF -> USD conversion")
def check_convert():
    from reflex_quad.backends.isaaclab import IsaacLabBackend
    from reflex_quad.config import load_experiment

    cfg = load_experiment("01_stand")
    # the converter does not need a live backend, only the config
    shim = type("ConfigOnly", (), {"cfg": cfg})()
    return IsaacLabBackend._ensure_usd(shim, cfg.get("isaac", {}))


@check("scene + articulation")
def check_scene():
    import numpy as np

    from reflex_quad.backends.isaaclab import IsaacLabBackend
    from reflex_quad.config import load_experiment

    cfg = load_experiment("02_uneven_ground")
    be = IsaacLabBackend(cfg, np.random.default_rng(0))
    joints = list(be.robot.joint_names)
    bodies = list(be.robot.body_names)
    globals()["_BACKEND"] = be
    return f"{len(joints)} joints {joints}; {len(bodies)} bodies"


@check("sensors + 100 steps")
def check_step():
    import numpy as np

    be = globals()["_BACKEND"]
    st = be.reset()
    for _ in range(100):
        st = be.step(np.zeros(8))
    return (f"q[0]={st.q[0]:+.3f} body_z={st.body_pos[2]:.3f} "
            f"contact_z={np.round(st.contact_force[:, 2], 2)} "
            f"body_acc={np.round(st.body_accel_body, 2)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="run with the Isaac window visible")
    args, _ = ap.parse_known_args()

    check_torch()
    check_isaacsim()
    if not check_app(headless=not args.gui):
        print("\nAppLauncher failed; nothing else can run.")
        return 1
    check_modules()
    check_urdf()
    check_convert()
    if check_scene():
        check_step()

    failed = [n for n, ok, _ in RESULTS if not ok]
    print("\n" + "=" * 60)
    print(f"{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    for n, ok, d in RESULTS:
        if not ok:
            print(f"  FAILED {n}: {d}")
    if failed:
        print("\nFix these before running an experiment; see runbooks/RB-90-troubleshooting.md")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
