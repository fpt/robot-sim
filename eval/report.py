"""Write report.md and report.json into a run directory."""
from __future__ import annotations

import json
from pathlib import Path

from .criteria import RunVerdict


def write_report(run, metrics: dict, verdict: RunVerdict, plots: list[Path]) -> Path:
    out = run.path / "report.md"
    meta = run.meta
    if not verdict.checks:
        status = "NO CRITERIA"          # an empty gate is not a pass
    else:
        status = "PASS" if verdict.passed else "FAIL"

    lines = [
        f"# {meta.get('experiment_id')} - {status}",
        "",
        f"- run: `{run.path.name}`",
        f"- backend: `{meta.get('backend')}`  fidelity: `{meta.get('fidelity', {}).get('stage')}`",
        f"- git: `{meta.get('git')}`  numpy {meta.get('numpy')}  python {meta.get('python')}",
        f"- duration: {metrics.get('duration_s', 0):.1f} s,"
        f" {metrics.get('samples')} control samples",
        "",
        "## Criteria",
        "",
    ]
    if verdict.checks:
        lines += ["| check | metric | actual | required | verdict |",
                  "|---|---|---|---|---|"]
        for c in verdict.checks:
            actual = f"{c.actual:.4g}" if isinstance(c.actual, float) else c.actual
            mark = "MISSING" if c.missing else ("pass" if c.passed else "**FAIL**")
            lines.append(f"| {c.name} | `{c.metric}` | {actual} | {c.expected} | {mark} |")
        lines.append("")
        for c in verdict.failures:
            if c.why:
                lines.append(f"- **{c.name}**: {c.why}")
    else:
        lines.append("_no criteria defined for this experiment id_")

    lines += ["", "## Metrics", "", "| metric | value |", "|---|---|"]
    for k in sorted(metrics):
        v = metrics[k]
        if isinstance(v, float):
            v = f"{v:.5g}"
        lines.append(f"| `{k}` | {v} |")

    if meta.get("fault_plan"):
        lines += ["", "## Faults injected", ""]
        for f in meta["fault_plan"]:
            lines.append(
                f"- t={f['t']} s, joint {f['joint']},"
                f" expected class `{f['expected_class']}`"
            )
        det = [e for e in run.events if e["kind"] == "fault_detected"]
        lines.append("")
        lines.append(f"detections: {len(det)}")
        for e in det[:12]:
            lines.append(f"- t={e['t']:.2f} `{e['detected_class']}` joint {e['joint']}")

    if plots:
        lines += ["", "## Plots", ""]
        lines += [f"![{p.stem}]({p.relative_to(run.path)})" for p in plots]

    out.write_text("\n".join(lines) + "\n")
    (run.path / "report.json").write_text(json.dumps({
        "experiment_id": meta.get("experiment_id"),
        "run": run.path.name,
        "passed": verdict.passed,
        "metrics": metrics,
        "checks": [
            {"name": c.name, "metric": c.metric, "passed": c.passed, "actual": c.actual,
             "expected": c.expected, "missing": c.missing, "why": c.why}
            for c in verdict.checks
        ],
    }, indent=2, default=str))
    return out
