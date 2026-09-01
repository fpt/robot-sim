"""Simulator backends.  `mock` needs nothing but numpy; `isaaclab` needs the
CUDA machine.  Both expose the same `SimBackend` interface so an experiment
script does not change between them.
"""
from __future__ import annotations


def make_backend(name: str, cfg: dict, rng):
    if name == "mock":
        from .mock import MockBackend

        return MockBackend(cfg, rng)
    if name in ("isaaclab", "isaac", "isaacsim"):
        from .isaaclab import IsaacLabBackend

        return IsaacLabBackend(cfg, rng)
    raise ValueError(f"unknown backend {name!r} (expected 'mock' or 'isaaclab')")
