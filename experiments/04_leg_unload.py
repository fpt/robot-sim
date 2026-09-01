#!/usr/bin/env python
"""Unload, lift, verify by IMU, lower, reload. memo.txt sections 32-34.

    python experiments/04_leg_unload.py --backend mock --eval
    python experiments/04_leg_unload.py --backend isaaclab --fidelity 2 --eval
"""
from _common import main

if __name__ == "__main__":
    raise SystemExit(main("04_leg_unload"))
