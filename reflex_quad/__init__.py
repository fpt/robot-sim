"""reflex_quad - proprioception-free quadruped experiments on Isaac Sim / Isaac Lab.

The one rule of this package (memo.txt section 13):

    Nothing under `controller.py`, `observer.py`, `objective.py` or
    `state_machine.py` may read joint angle, joint velocity or joint torque.
    Those live in `GroundTruth`, which is handed only to the logger.

`tests/test_observation_isolation.py` enforces this mechanically.
"""

__version__ = "0.1.0"

LEG_NAMES = ("FL", "FR", "RL", "RR")
JOINT_NAMES = tuple(f"{leg}_{j}" for leg in LEG_NAMES for j in ("hip", "knee"))
N_LEGS = 4
N_JOINTS = 8
