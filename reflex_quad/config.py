"""YAML config loading with `defaults` + per-experiment override merging."""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"


def _deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_yaml(name: str, config_dir: Path | None = None) -> dict:
    path = (config_dir or CONFIG_DIR) / name
    with open(path) as fh:
        return yaml.safe_load(fh)


class Config(dict):
    """A dict with dotted lookup: cfg.get_path('servo.pd.kp')."""

    def get_path(self, dotted: str, default: Any = None) -> Any:
        node: Any = self
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node


def load_experiment(exp_id: str, config_dir: Path | None = None) -> Config:
    """Build the fully merged config for one experiment id."""
    cdir = config_dir or CONFIG_DIR
    exp_file = load_yaml("experiment.yaml", cdir)
    if exp_id not in exp_file["experiments"]:
        raise KeyError(
            f"unknown experiment {exp_id!r}; known: {sorted(exp_file['experiments'])}"
        )
    merged = _deep_merge(exp_file["defaults"], exp_file["experiments"][exp_id])
    servo = load_yaml("servo.yaml", cdir)
    if "servo_phase" in merged:
        # A fault case can only show up if the servo models the thing it breaks:
        # "friction x10" does nothing at phase B, "delay +100 ms" does nothing
        # before phase C.  Experiments that inject those must ask for the phase.
        servo["phase"] = merged.pop("servo_phase")
    cfg = Config(
        experiment_id=exp_id,
        robot=load_yaml("robot.yaml", cdir),
        servo=servo,
        sensors=load_yaml("sensors.yaml", cdir),
        fault_library=load_yaml("faults.yaml", cdir),
        **merged,
    )
    return cfg


def apply_fidelity(cfg: Config, stage: int, rng=None) -> Config:
    """Apply one rung of the four-stage ladder (config/fidelity.yaml) in place."""
    ladder = load_yaml("fidelity.yaml")
    key = int(stage)
    if key not in ladder:
        raise KeyError(f"fidelity stage {stage} not in {sorted(ladder)}")
    prof = ladder[key]
    cfg["fidelity"] = {"stage": key, **prof}
    cfg["servo"]["phase"] = prof["servo_phase"]

    ns, bs = float(prof["sensor_noise_scale"]), float(prof["sensor_bias_scale"])
    for group in cfg["sensors"].values():
        if not isinstance(group, dict):
            continue
        for k, v in list(group.items()):
            if not isinstance(v, (int, float)):
                continue
            if k.endswith("noise_std"):
                group[k] = v * ns
            elif k.endswith("bias_std") or k.endswith("drift_std") or k == "gain_spread":
                group[k] = v * bs

    r = float(prof.get("randomize", 0.0))
    if r > 0 and rng is not None:
        pd, lim, fr = cfg["servo"]["pd"], cfg["servo"]["limits"], cfg["servo"]["friction"]
        for d, keys in ((pd, ["kp", "kd"]), (lim, ["tau_max", "qd_max"]),
                        (fr, ["coulomb", "viscous"])):
            for k in keys:
                d[k] = float(d[k]) * float(1.0 + rng.uniform(-r, r))
    return cfg


def list_experiments(config_dir: Path | None = None) -> list[str]:
    return list(load_yaml("experiment.yaml", config_dir or CONFIG_DIR)["experiments"])
