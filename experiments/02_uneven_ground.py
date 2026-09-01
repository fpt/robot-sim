#!/usr/bin/env python
"""One foot on an unseen 20 mm block. memo.txt sections 25, 26.

    python experiments/02_uneven_ground.py --backend mock --eval
    python experiments/02_uneven_ground.py --backend isaaclab --fidelity 2 --eval
"""
from _common import main

if __name__ == "__main__":
    raise SystemExit(main("02_uneven_ground"))
