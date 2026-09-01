#!/usr/bin/env python
"""Insect-style self-inspection: run the lift cycle on all four legs.

memo.txt section 38.  Records the per-leg command -> current / foot IMU / foot
force signature that later runs are compared against.
"""
from _common import main

if __name__ == "__main__":
    raise SystemExit(main("07_self_check"))
