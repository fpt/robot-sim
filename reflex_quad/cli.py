"""Command line for running experiments.

    python -m reflex_quad 01_stand
    python -m reflex_quad 03_dither --backend isaaclab --fidelity 2 --eval
    python -m reflex_quad --list
"""
from __future__ import annotations

import argparse
from pathlib import Path

from .config import list_experiments
from .runner import run_experiment


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser("reflex_quad", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiment", nargs="?", help="experiment id from config/experiment.yaml")
    ap.add_argument("--list", action="store_true", help="list experiment ids and exit")
    ap.add_argument("--backend", choices=["mock", "isaaclab"], default=None)
    ap.add_argument("--fidelity", type=int, choices=[1, 2, 3, 4], default=None,
                    help="verification ladder rung, see config/fidelity.yaml")
    ap.add_argument("--duration", type=float, default=None, help="override seconds")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--tag", default="", help="suffix for the run directory name")
    ap.add_argument("--log-root", type=Path, default=Path("logs"))
    ap.add_argument("--repeat", type=int, default=1, help="run N times with seed+i")
    ap.add_argument("--eval", action="store_true", help="evaluate immediately afterwards")
    ap.add_argument("--quiet", action="store_true")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list or not args.experiment:
        for e in list_experiments():
            print(e)
        return 0 if args.list else 2

    status = 0
    for i in range(args.repeat):
        seed = None if args.seed is None else args.seed + i
        tag = args.tag if args.repeat == 1 else f"{args.tag}s{i}".strip()
        print(f"running {args.experiment} "
              f"(backend={args.backend or 'config'}, fidelity={args.fidelity or 'config'})")
        run_dir = run_experiment(
            args.experiment, log_root=args.log_root, seed=seed, duration=args.duration,
            backend_name=args.backend, fidelity=args.fidelity, tag=tag,
            progress=not args.quiet,
        )
        if args.eval:
            from eval.cli import main as eval_main

            status |= eval_main([str(run_dir)])
    return status


if __name__ == "__main__":
    raise SystemExit(main())
