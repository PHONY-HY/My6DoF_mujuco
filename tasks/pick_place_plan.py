from __future__ import annotations

import numpy as np


def build_pick_place_plan(
    pick_position: np.ndarray,
    cube_position: np.ndarray,
    place_position: np.ndarray,
) -> list[dict[str, np.ndarray | float | str]]:
    pick_position = np.asarray(pick_position, dtype=float)
    cube_position = np.asarray(cube_position, dtype=float)
    place_position = np.asarray(place_position, dtype=float)

    hover_position = pick_position + np.array([0.0, 0.0, 0.08], dtype=float)
    grasp_position = cube_position + np.array([0.0, 0.0, 0.05], dtype=float)
    lift_position = cube_position + np.array([0.0, 0.0, 0.15], dtype=float)
    place_hover = place_position + np.array([0.0, 0.0, 0.08], dtype=float)

    return [
        {"name": "hover_pick", "target_position": hover_position, "gripper_opening": 0.04},
        {"name": "descend_pick", "target_position": grasp_position, "gripper_opening": 0.04},
        {"name": "close_gripper", "target_position": grasp_position, "gripper_opening": 0.0},
        {"name": "lift", "target_position": lift_position, "gripper_opening": 0.0},
        {"name": "hover_place", "target_position": place_hover, "gripper_opening": 0.0},
        {"name": "descend_place", "target_position": place_position, "gripper_opening": 0.0},
        {"name": "open_gripper", "target_position": place_position, "gripper_opening": 0.04},
        {"name": "retreat", "target_position": place_hover, "gripper_opening": 0.04},
    ]