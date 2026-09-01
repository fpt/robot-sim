"""Evaluate runs.

    python -m eval.cli logs/01_stand_20260901_120000
    python -m eval.cli --latest 01_stand
    python -m eval.cli --all
    python -m eval.cli --all --phase phase1
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .criteria import evaluate_phase, evaluate_run, load_criteria
from .isolation import check_isolation
from .metrics import compute_metrics, load_run
from .plots import make_plots
from .report import write_report


def _latest(log_root: Path, exp_id: str) -> Path | None:
    runs = sorted(p for p in log_root.glob(f"{exp_id}_*") if (p / "control.csv").exists())
    return runs[-1] if runs else None


def _all_latest(log_root: Path) -> dict[str, Path]:
    out: dict[str, Path] = {}
    for p in sorted(log_root.iterdir()):
        if not (p / "meta.json").exists():
            continue
        exp = p.name.rsplit("_", 2)[0]
        out[exp] = p            # sorted order means the last one wins
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", type=Path)
    ap.add_argument("--log-root", type=Path, default=Path("logs"))
    ap.add_argument("--latest", metavar="EXPERIMENT_ID")
    ap.add_argument("--all", action="store_true", help="latest run of every experiment")
    ap.add_argument("--phase", action="append", default=[], help="also check a phase gate")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    runs = list(args.runs)
    if args.latest:
        p = _latest(args.log_root, args.latest)
        if p is None:
            print(f"no run found for {args.latest}", file=sys.stderr)
            return 2
        runs.append(p)
    if args.all:
        runs.extend(_all_latest(args.log_root).values())
    if not runs:
        ap.error("give a run directory, --latest ID, or --all")

    spec = load_criteria()
    verdicts = {}
    failed = 0
    for path in runs:
        run = load_run(path)
        metrics = compute_metrics(run)
        verdict = evaluate_run(run, metrics, spec)
        plots = [] if args.no_plots else make_plots(run)
        write_report(run, metrics, verdict, plots)
        verdicts[verdict.experiment_id] = verdict

        if not verdict.checks:
            status = "NO-CRITERIA"      # an empty gate is not a pass
        else:
            status = "PASS" if verdict.passed else "FAIL"
        print(f"\n=== {verdict.experiment_id}  [{status}]  {path.name}")
        if not args.quiet:
            for c in verdict.checks:
                print("   " + c.line())
        if verdict.checks and not verdict.passed:
            failed += 1

    violations = check_isolation()
    extra = {"isolation_violations": len(violations)}
    if violations:
        print("\nOBSERVATION ISOLATION VIOLATIONS (memo.txt section 13):")
        for v in violations:
            print("   " + v)

    for phase in args.phase:
        ok, notes = evaluate_phase(phase, verdicts, extra, spec)
        print(f"\n=== {phase}: {spec['phases'][phase]['title']}  [{'PASS' if ok else 'FAIL'}]")
        for n in notes:
            print("   - " + n)
        if not ok:
            failed += 1

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
