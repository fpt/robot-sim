import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from reflex_quad.config import load_experiment  # noqa: E402
from reflex_quad.robot import LegGeometry  # noqa: E402


@pytest.fixture
def cfg():
    return load_experiment("01_stand")


@pytest.fixture
def geom(cfg):
    return LegGeometry(cfg["robot"])


@pytest.fixture
def rng():
    return np.random.default_rng(1234)
