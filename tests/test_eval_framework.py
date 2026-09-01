import math

import numpy as np
import pytest

from eval.criteria import CheckResult, RunVerdict, evaluate_metrics, evaluate_phase, load_criteria
from eval.metrics import _mann_kendall, _settling_time


def test_criteria_inherit_pulls_in_the_common_checks():
    checks = {c.name for c in evaluate_metrics("01_stand", {"nan_count": 0}, load_criteria())}
    assert "no_nan" in checks and "quiet_roll" in checks


def test_criteria_inherit_can_chain_experiments():
    checks = {c.name for c in evaluate_metrics("06_first_step", {}, load_criteria())}
    assert {"no_nan", "unload_reached", "moved_forward"} <= checks


def test_a_missing_metric_fails_loudly_rather_than_passing():
    checks = evaluate_metrics("01_stand", {}, load_criteria())
    assert all(c.missing for c in checks)
    assert not any(c.passed for c in checks)


def test_infinite_latency_does_not_satisfy_a_less_than_check():
    spec = load_criteria()
    metrics = {"fault_detection_latency_s": math.inf}
    check = [c for c in evaluate_metrics("05_fault", metrics, spec)
             if c.metric == "fault_detection_latency_s"][0]
    assert not check.passed


def test_mann_kendall_sees_a_downward_trend():
    tau, p = _mann_kendall(np.linspace(10, 1, 500) + np.random.default_rng(0).normal(0, .1, 500))
    assert tau < -0.9 and p < 0.01


def test_mann_kendall_ignores_pure_noise():
    _, p = _mann_kendall(np.random.default_rng(3).normal(0, 1, 500))
    assert p > 0.05


def test_settling_time_finds_the_last_excursion():
    t = np.linspace(0, 10, 1000)
    sig = np.where(t < 3.0, 5.0, 0.0)
    assert _settling_time(sig, t, band=0.5) == pytest.approx(3.0, abs=0.05)


def test_phase_gate_fails_when_a_run_is_missing():
    ok, notes = evaluate_phase("phase1", {}, {"isolation_violations": 0}, load_criteria())
    assert not ok
    assert any("missing run" in n for n in notes)


def test_phase_gate_fails_on_isolation_violations():
    passing = {e: RunVerdict(e, "x", [CheckResult("c", "m", True, 1, "", "")])
               for e in ("01_stand", "02_uneven_ground", "03_dither")}
    ok, _ = evaluate_phase("phase1", passing, {"isolation_violations": 0}, load_criteria())
    assert ok
    bad, notes = evaluate_phase("phase1", passing, {"isolation_violations": 3}, load_criteria())
    assert not bad
