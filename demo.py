from __future__ import annotations

from pprint import pprint

import numpy as np

from ur5_style_arm import UR5StyleArm


def run_demo() -> dict[str, np.ndarray | dict[str, float | int]]:
    arm = UR5StyleArm()
    q_seed = np.array([0.2, -1.1, 1.0, -0.7, 0.3, 0.2], dtype=float)
    qdot = np.array([0.1, -0.05, 0.08, 0.02, -0.04, 0.03], dtype=float)

    fk_matrix = arm.forward_kinematics(q_seed, output="matrix")
    ik_solution, ik_info = arm.inverse_kinematics(fk_matrix, q0=arm.neutral_q, max_iters=300, tol=1e-6)
    forward_twist = arm.forward_velocity(q_seed, qdot)
    inverse_qdot = arm.inverse_velocity(q_seed, forward_twist, damping=1e-6)

    return {
        "fk_matrix": fk_matrix,
        "ik_solution": ik_solution,
        "ik_info": ik_info,
        "forward_twist": forward_twist,
        "inverse_qdot": inverse_qdot,
    }


if __name__ == "__main__":
    pprint(run_demo())
