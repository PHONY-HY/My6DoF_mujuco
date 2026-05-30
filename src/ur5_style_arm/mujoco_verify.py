from __future__ import annotations

import mujoco
import numpy as np


def compare_end_effector_positions(arm, model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray) -> dict[str, np.ndarray | float]:
    q = np.asarray(q, dtype=float)
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)

    pinocchio_pose = arm.forward_kinematics(q, output="matrix")
    pinocchio_position = pinocchio_pose[:3, 3]
    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    mujoco_position = data.site_xpos[ee_site_id].copy()
    error = mujoco_position - pinocchio_position

    return {
        "pinocchio_position": pinocchio_position,
        "mujoco_position": mujoco_position,
        "position_error": error,
        "position_error_norm": float(np.linalg.norm(error)),
    }
