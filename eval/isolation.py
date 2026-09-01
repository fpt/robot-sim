"""Static check of the one rule (memo.txt section 13).

The controller side of the code may not name joint truth.  This is a source
scan, so it also catches a violation that never executes.  It is used by the
test suite and reported as `isolation_violations` in the phase 1 gate.
"""
from __future__ import annotations

import ast
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent / "reflex_quad"

# modules that run on the robot: they see Observation and nothing else
CONTROLLER_SIDE = (
    "controller.py", "observer.py", "objective.py", "dither.py",
    "state_machine.py", "support.py", "faults.py",
)
# names that only exist inside the simulator
FORBIDDEN = (
    "q_true", "qd_true", "tau_true", "joint_angle", "joint_velocity",
    "joint_torque", "body_pose_true", "contact_force_true", "ground_truth",
    "sim_state", "SimState", "backend", "GroundTruth",
)
# faults.py legitimately injects into the servo/sensor models; only its detector
# half is controller-side, so the injector class is exempt by name
EXEMPT_CLASSES = {"FaultInjector"}


def scan_file(path: Path) -> list[str]:
    tree = ast.parse(path.read_text())
    exempt_lines: set[int] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name in EXEMPT_CLASSES:
            exempt_lines.update(range(node.lineno, (node.end_lineno or node.lineno) + 1))

    hits = []
    for node in ast.walk(tree):
        if getattr(node, "lineno", None) in exempt_lines:
            continue
        name = None
        if isinstance(node, ast.Name):
            name = node.id
        elif isinstance(node, ast.Attribute):
            name = node.attr
        elif isinstance(node, (ast.arg,)):
            name = node.arg
        if name and name in FORBIDDEN:
            hits.append(f"{path.name}:{node.lineno}: reads {name!r}")
    return hits


def check_isolation(pkg: Path | None = None) -> list[str]:
    pkg = pkg or PKG
    violations: list[str] = []
    for name in CONTROLLER_SIDE:
        path = pkg / name
        if path.exists():
            violations.extend(scan_file(path))
    return violations
