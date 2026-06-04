from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from tasks.recording import TrajectoryFrame


@dataclass
class PickPlaceEpisodeResult:
    frames: list[TrajectoryFrame]
    grasp_success: bool
    episode_success: bool
    failure_reason: str | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "frames": [frame.to_dict() for frame in self.frames],
            "grasp_success": self.grasp_success,
            "episode_success": self.episode_success,
            "failure_reason": self.failure_reason,
        }


def _distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def is_gripper_closed_for_grasp(gripper_state: dict[str, float]) -> bool:
    left = float(gripper_state["left"])
    right = float(gripper_state["right"])
    return 0.014 <= left <= 0.0185 and 0.014 <= right <= 0.0185


def pick_place_stage_failure_reason(
    step_name: str,
    grasp_likely_secured: bool,
    cube_height: float,
    initial_cube_height: float,
    table_contact_detected: bool = True,
    place_alignment_reached: bool = True,
) -> str | None:
    if step_name == "close_gripper" and not grasp_likely_secured:
        return "grasp_not_secured"
    if step_name == "lift_clearance" and float(cube_height) <= float(initial_cube_height) + 0.02:
        return "cube_not_lifted"
    if step_name == "descend_place_contact" and not table_contact_detected:
        return "place_contact_not_reached"
    if step_name == "descend_place_contact" and not place_alignment_reached:
        return "place_alignment_not_reached"
    return None


def is_blocking_position_stage(step_name: str) -> bool:
    return step_name in {
        "hover_pick",
        "descend_pick",
        "lift_clearance",
        "translate_outboard",
        "translate_lateral_place",
        "translate_inboard_place",
        "place_hover",
    }


def evaluate_pick_place_episode(
    frames: list[TrajectoryFrame],
    initial_cube_height: float,
    place_target: np.ndarray,
) -> PickPlaceEpisodeResult:
    if not frames:
        return PickPlaceEpisodeResult(
            frames=[],
            grasp_success=False,
            episode_success=False,
            failure_reason="no_frames",
        )

    place_target = np.asarray(place_target, dtype=float)
    cube_positions = [
        np.asarray(frame.object_pose["position"], dtype=float)
        for frame in frames
        if frame.object_pose is not None
    ]

    lifted_frames = [
        position
        for position in cube_positions
        if float(position[2]) > float(initial_cube_height) + 0.02
    ]
    if not lifted_frames:
        return PickPlaceEpisodeResult(
            frames=frames,
            grasp_success=False,
            episode_success=False,
            failure_reason="cube_not_lifted",
        )

    transported_frames = [
        position
        for position in cube_positions
        if _distance(position[:2], place_target[:2]) <= 0.06
    ]
    if not transported_frames:
        return PickPlaceEpisodeResult(
            frames=frames,
            grasp_success=False,
            episode_success=False,
            failure_reason="cube_not_transported",
        )

    final_position = cube_positions[-1]
    final_gripper_state = frames[-1].gripper_state
    placed = (
        _distance(final_position[:2], place_target[:2]) <= 0.04
        and abs(float(final_position[2]) - float(place_target[2])) <= 0.04
        and float(final_gripper_state["left"]) >= 0.02
        and float(final_gripper_state["right"]) >= 0.02
    )

    if not placed:
        return PickPlaceEpisodeResult(
            frames=frames,
            grasp_success=True,
            episode_success=False,
            failure_reason="cube_not_placed",
        )

    return PickPlaceEpisodeResult(
        frames=frames,
        grasp_success=True,
        episode_success=True,
        failure_reason=None,
    )
