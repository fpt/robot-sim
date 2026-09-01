"""Apply config/criteria.yaml to a metrics dict.

A check is `{metric, op, value, why}`.  The point of keeping the thresholds in
YAML rather than in code is that a pass/fail line is a research decision: it
should be visible, versioned, and arguable without touching the evaluator.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

CRITERIA_PATH = Path(__file__).resolve().parent.parent / "config" / "criteria.yaml"

OPS = {
    "lt": lambda a, b: a < b,
    "le": lambda a, b: a <= b,
    "gt": lambda a, b: a > b,
    "ge": lambda a, b: a >= b,
    "eq": lambda a, b: a == b,
    "ne": lambda a, b: a != b,
    "is_true": lambda a, b: bool(a) is True,
    "is_false": lambda a, b: bool(a) is False,
}
SYMBOL = {"lt": "<", "le": "<=", "gt": ">", "ge": ">=", "eq": "==", "ne": "!=",
          "is_true": "is true", "is_false": "is false"}


@dataclass
class CheckResult:
    name: str
    metric: str
    passed: bool
    actual: object
    expected: str
    why: str
    missing: bool = False

    def line(self) -> str:
        mark = "MISSING" if self.missing else ("PASS" if self.passed else "FAIL")
        actual = self.actual
        if isinstance(actual, float):
            actual = f"{actual:.4g}"
        return f"[{mark:7s}] {self.name:24s} {self.metric} = {actual}  (need {self.expected})"


@dataclass
class RunVerdict:
    experiment_id: str
    run_dir: str
    checks: list[CheckResult]

    @property
    def passed(self) -> bool:
        return all(c.passed and not c.missing for c in self.checks)

    @property
    def failures(self) -> list[CheckResult]:
        return [c for c in self.checks if not c.passed or c.missing]


def load_criteria(path: Path | None = None) -> dict:
    with open(path or CRITERIA_PATH) as fh:
        return yaml.safe_load(fh)


def _resolve(exp_id: str, spec: dict, seen: set[str] | None = None) -> dict:
    """Expand `inherit:` (either `common` or another experiment id)."""
    seen = seen or set()
    if exp_id in seen:
        raise ValueError(f"circular inherit at {exp_id!r}")
    seen.add(exp_id)
    raw = spec["experiments"][exp_id]
    block = dict(raw) if isinstance(raw, dict) else {}
    parent = block.pop("inherit", None)
    checks: dict = {}
    if parent == "common":
        checks.update(spec.get("common", {}))
    elif parent:
        checks.update(_resolve(parent, spec, seen))
    checks.update(block)
    return checks


def evaluate_metrics(exp_id: str, metrics: dict, spec: dict | None = None) -> list[CheckResult]:
    spec = spec or load_criteria()
    if exp_id not in spec.get("experiments", {}):
        return []
    results = []
    for name, check in _resolve(exp_id, spec).items():
        metric = check["metric"]
        op = check["op"]
        want = check.get("value")
        if metric not in metrics:
            results.append(CheckResult(name, metric, False, None,
                                       f"{SYMBOL[op]} {want}".strip(), check.get("why", ""),
                                       missing=True))
            continue
        actual = metrics[metric]
        try:
            ok = bool(OPS[op](actual, want))
        except TypeError:
            ok = False
        if isinstance(actual, float) and not math.isfinite(actual) and op in ("lt", "le"):
            ok = False
        expected = f"{SYMBOL[op]} {want}".strip() if want is not None else SYMBOL[op]
        results.append(
            CheckResult(name, metric, ok, actual, expected, check.get("why", ""))
        )
    return results


def evaluate_run(run, metrics: dict, spec: dict | None = None) -> RunVerdict:
    exp_id = metrics.get("experiment_id") or run.meta.get("experiment_id")
    return RunVerdict(exp_id, str(run.path), evaluate_metrics(exp_id, metrics, spec))


def evaluate_phase(
    phase: str,
    verdicts: dict[str, RunVerdict],
    extra_metrics: dict | None = None,
    spec: dict | None = None,
) -> tuple[bool, list[str]]:
    """A phase gate passes when every run it names passes, plus its own checks."""
    spec = spec or load_criteria()
    block = spec["phases"][phase]
    notes: list[str] = []
    ok = True
    for exp_id in block.get("requires_runs", []):
        v = verdicts.get(exp_id)
        if v is None:
            ok = False
            notes.append(f"missing run: {exp_id}")
        elif not v.passed:
            ok = False
            notes.append(f"{exp_id}: " + ", ".join(c.name for c in v.failures))
        else:
            notes.append(f"{exp_id}: pass")
    for name, check in (block.get("extra") or {}).items():
        metrics = extra_metrics or {}
        if check["metric"] not in metrics:
            ok = False
            notes.append(f"{name}: metric {check['metric']} not supplied")
            continue
        if not OPS[check["op"]](metrics[check["metric"]], check.get("value")):
            ok = False
            notes.append(f"{name}: failed ({metrics[check['metric']]})")
        else:
            notes.append(f"{name}: pass")
    return ok, notes
