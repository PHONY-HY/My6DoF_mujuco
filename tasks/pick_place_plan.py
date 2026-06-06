from __future__ import annotations

import numpy as np


GRIPPER_OPENING_OPEN = 0.025
GRIPPER_OPENING_CLOSE = 0.0145


def build_pick_place_plan(
    pick_position: np.ndarray,
    cube_position: np.ndarray,
    place_position: np.ndarray,
) -> list[dict[str, np.ndarray | float | str]]:
    pick_position = np.asarray(pick_position, dtype=float)
    cube_position = np.asarray(cube_position, dtype=float)
    place_position = np.asarray(place_position, dtype=float)

    hover_position = cube_position + np.array([0.0, 0.0, 0.06], dtype=float)
    grasp_position = cube_position.copy()
    transport_height = max(float(cube_position[2]), float(place_position[2])) + 0.10
    lift_clearance = np.array([cube_position[0], cube_position[1], transport_height], dtype=float)
    outboard_x = max(float(cube_position[0]), float(place_position[0])) + 0.02
    translate_outboard = np.array([outboard_x, cube_position[1], transport_height], dtype=float)
    translate_lateral_place = np.array([outboard_x, place_position[1], transport_height], dtype=float)
    translate_inboard_place = np.array([place_position[0], place_position[1], transport_height], dtype=float)
    place_hover = place_position + np.array([0.0, 0.0, 0.06], dtype=float)
    place_contact = place_position + np.array([0.0, 0.0, -0.05], dtype=float)

    return [
        {"name": "hover_pick", "target_position": hover_position, "gripper_opening": GRIPPER_OPENING_OPEN, "controller_steps": 260, "position_tolerance": 0.05, "control_mode": "pose"},
        {"name": "descend_pick", "target_position": grasp_position, "gripper_opening": GRIPPER_OPENING_OPEN, "controller_steps": 1200, "position_tolerance": 0.015, "control_mode": "pose"},
        {"name": "close_gripper", "target_position": grasp_position, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 600, "position_tolerance": 0.015, "control_mode": "pose"},
        {"name": "lift_clearance", "target_position": lift_clearance, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 500, "position_tolerance": 0.10, "control_mode": "pose"},
        {"name": "translate_outboard", "target_position": translate_outboard, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 380, "position_tolerance": 0.18, "control_mode": "pose"},
        {"name": "translate_lateral_place", "target_position": translate_lateral_place, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 800, "position_tolerance": 0.16, "control_mode": "pose"},
        {"name": "translate_inboard_place", "target_position": translate_inboard_place, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 600, "position_tolerance": 0.16, "control_mode": "pose"},
        {"name": "place_hover", "target_position": place_hover, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 800, "position_tolerance": 0.20, "control_mode": "pose"},
        {"name": "descend_place_contact", "target_position": place_contact, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 600, "position_tolerance": 0.02, "control_mode": "pose"},
        {"name": "settle_place", "target_position": place_contact, "gripper_opening": GRIPPER_OPENING_CLOSE, "controller_steps": 20, "position_tolerance": 0.02, "control_mode": "pose"},
        {"name": "release_place", "target_position": place_contact, "gripper_opening": GRIPPER_OPENING_OPEN, "controller_steps": 40, "position_tolerance": 0.02, "control_mode": "pose"},
        {"name": "retreat", "target_position": place_hover, "gripper_opening": GRIPPER_OPENING_OPEN, "controller_steps": 120, "position_tolerance": 0.03, "control_mode": "pose"},
    ]
