#!/usr/bin/env python
"""Inject a fault and detect it from observations only. memo.txt sections 35-37.

    python experiments/05_fault.py --backend mock --eval
    python experiments/05_fault.py --backend isaaclab --fidelity 2 --eval
"""
from _common import main

if __name__ == "__main__":
    raise SystemExit(main("05_fault"))
