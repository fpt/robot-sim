"""Shared entry point for the numbered experiment scripts.

The scripts in this directory exist because memo.txt section 9 names them.  They
are deliberately thin: all the logic is in reflex_quad/, so that an experiment
is a *configuration*, not a fork of the control code.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from reflex_quad.cli import main as _main  # noqa: E402


def main(experiment_id: str) -> int:
    argv = sys.argv[1:]
    if not any(a == experiment_id for a in argv):
        argv = [experiment_id, *argv]
    return _main(argv)
