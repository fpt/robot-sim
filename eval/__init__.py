"""Offline evaluation of a run directory produced by reflex_quad.runner.

    metrics.py   control.csv + truth.csv + events.jsonl  ->  one flat dict
    criteria.py  that dict + config/criteria.yaml        ->  pass/fail per check
    plots.py     the six graphs of memo.txt section 44
    report.py    report.md + report.json next to the logs

Nothing here runs during an experiment.  This is the only place that is allowed
to open truth.csv (memo.txt section 43).
"""
from .criteria import evaluate_phase, evaluate_run  # noqa: F401
from .metrics import compute_metrics, load_run  # noqa: F401
