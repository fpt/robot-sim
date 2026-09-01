#!/usr/bin/env python
"""Flat ground, no feedback. memo.txt section 24.

    python experiments/01_stand.py --backend mock --eval
    python experiments/01_stand.py --backend isaaclab --fidelity 2 --eval
"""
from _common import main

if __name__ == "__main__":
    raise SystemExit(main("01_stand"))
