"""Start the Omniverse app before anything from `isaaclab` is imported.

Isaac Lab requires the SimulationApp to exist before its modules are imported;
importing them first fails with an unhelpful error.  Everything that touches
Isaac therefore goes through here.
"""
from __future__ import annotations

_APP = None


def ensure_app(headless: bool = True, enable_cameras: bool = False):
    """Launch (once) and return the SimulationApp."""
    global _APP
    if _APP is not None:
        return _APP
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=headless, enable_cameras=enable_cameras)
    _APP = launcher.app
    return _APP


def close_app() -> None:
    global _APP
    if _APP is not None:
        _APP.close()
        _APP = None
