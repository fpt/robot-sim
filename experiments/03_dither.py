#!/usr/bin/env python
"""Active sensing on one joint. memo.txt sections 27-31.

    python experiments/03_dither.py --backend mock --eval
    python experiments/03_dither.py --backend isaaclab --fidelity 2 --eval
"""
from _common import main

if __name__ == "__main__":
    raise SystemExit(main("03_dither"))
