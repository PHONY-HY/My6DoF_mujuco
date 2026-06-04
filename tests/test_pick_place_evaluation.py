import numpy as np
import pytest

from tasks.pick_place_plan import build_pick_place_plan
from tasks.recording import TrajectoryFrame


def test_pick_place_plan_exposes_close_gripper_dwell_steps():
    pick_position = np.array([0.35, 0.0, 0.32], dtype=float)
    cube_position = np.array([0.35, 0.0, 0.25], dtype=float)
    place_position = np.array([0.55, 0.1, 0.28], dtype=float)

    plan = build_pick_place_plan(pick_position, cube_position, place_position)
    close_step = next(step for step in plan if step["name"] == "close_gripper")

    assert close_step["controller_steps"] >= 20


def test_pick_place_plan_descends_to_cube_center_for_contact_grasp():
    pick_position = np.array([0.35, 0.0, 0.32], dtype=float)
    cube_position = np.array([0.35, 0.0, 0.25], dtype=float)
    place_position = np.array([0.55, 0.1, 0.28], dtype=float)

    plan = build_pick_place_plan(pick_position, cube_position, place_position)
    descend_step = next(step for step in plan if step["name"] == "descend_pick")

    assert abs(float(descend_step["target_position"][2]) - float(cube_position[2])) <= 0.015


def test_pick_place_plan_uses_realistic_gripper_openings_for_cube_size():
    pick_position = np.array([0.35, 0.0, 0.32], dtype=float)
    cube_position = np.array([0.35, 0.0, 0.25], dtype=float)
    place_position = np.array([0.55, 0.1, 0.28], dtype=float)

    plan = build_pick_place_plan(pick_position, cube_position, place_position)
    close_step = next(step for step in plan if step["name"] == "close_gripper")
    release_step = next(step for step in plan if step["name"] == "release_place")

    assert 0.014 <= float(close_step["gripper_opening"]) <= 0.0185
    assert float(release_step["gripper_opening"]) == 0.025


def test_pick_place_episode_result_is_reported_explicitly():
    from tasks.pick_place_evaluation import PickPlaceEpisodeResult

    result = PickPlaceEpisodeResult(
        frames=[],
        grasp_success=False,
        episode_success=False,
        failure_reason="grasp_not_secured",
    )

    payload = result.to_dict()

    assert payload["grasp_success"] is False
    assert payload["episode_success"] is False
    assert payload["failure_reason"] == "grasp_not_secured"


def test_grasp_success_requires_lift_transport_and_place():
    from tasks.pick_place_evaluation import evaluate_pick_place_episode

    initial_cube_height = 0.25
    place_target = np.array([0.55, 0.1, 0.28], dtype=float)

    successful_frames = [
        TrajectoryFrame(
            timestamp=0.1,
            joint_positions=[0.0] * 6,
            gripper_state={"left": 0.0, "right": 0.0},
            target_pose={"position": place_target.tolist(), "quaternion": [1.0, 0.0, 0.0, 0.0]},
            object_pose={"position": [0.36, 0.0, 0.29], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        ),
        TrajectoryFrame(
            timestamp=0.2,
            joint_positions=[0.0] * 6,
            gripper_state={"left": 0.0, "right": 0.0},
            target_pose={"position": place_target.tolist(), "quaternion": [1.0, 0.0, 0.0, 0.0]},
            object_pose={"position": [0.54, 0.1, 0.29], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        ),
        TrajectoryFrame(
            timestamp=0.3,
            joint_positions=[0.0] * 6,
            gripper_state={"left": 0.025, "right": 0.025},
            target_pose={"position": place_target.tolist(), "quaternion": [1.0, 0.0, 0.0, 0.0]},
            object_pose={"position": [0.55, 0.1, 0.28], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        ),
    ]

    result = evaluate_pick_place_episode(successful_frames, initial_cube_height, place_target)

    assert result.grasp_success is True
    assert result.episode_success is True
    assert result.failure_reason is None


def test_failed_grasp_reports_reason():
    from tasks.pick_place_evaluation import evaluate_pick_place_episode

    initial_cube_height = 0.25
    place_target = np.array([0.55, 0.1, 0.28], dtype=float)

    failed_frames = [
        TrajectoryFrame(
            timestamp=0.1,
            joint_positions=[0.0] * 6,
            gripper_state={"left": 0.0, "right": 0.0},
            target_pose={"position": place_target.tolist(), "quaternion": [1.0, 0.0, 0.0, 0.0]},
            object_pose={"position": [0.35, 0.0, 0.25], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        ),
        TrajectoryFrame(
            timestamp=0.2,
            joint_positions=[0.0] * 6,
            gripper_state={"left": 0.0, "right": 0.0},
            target_pose={"position": place_target.tolist(), "quaternion": [1.0, 0.0, 0.0, 0.0]},
            object_pose={"position": [0.36, 0.0, 0.251], "quaternion": [1.0, 0.0, 0.0, 0.0]},
        ),
    ]

    result = evaluate_pick_place_episode(failed_frames, initial_cube_height, place_target)

    assert result.grasp_success is False
    assert result.episode_success is False
    assert result.failure_reason == "cube_not_lifted"


def test_gripper_opening_near_cube_width_is_treated_as_valid_grasp_closure():
    from tasks.pick_place_evaluation import is_gripper_closed_for_grasp

    assert is_gripper_closed_for_grasp({"left": 0.0145, "right": 0.0145}) is True
    assert is_gripper_closed_for_grasp({"left": 0.016, "right": 0.016}) is True
    assert is_gripper_closed_for_grasp({"left": 0.0175, "right": 0.0175}) is True
    assert is_gripper_closed_for_grasp({"left": 0.025, "right": 0.025}) is False


def test_stage_failure_reason_reports_grasp_failure_at_close_step():
    from tasks.pick_place_evaluation import pick_place_stage_failure_reason

    reason = pick_place_stage_failure_reason(
        step_name="close_gripper",
        grasp_likely_secured=False,
        cube_height=0.25,
        initial_cube_height=0.25,
    )

    assert reason == "grasp_not_secured"


def test_stage_failure_reason_reports_lift_failure():
    from tasks.pick_place_evaluation import pick_place_stage_failure_reason

    reason = pick_place_stage_failure_reason(
        step_name="lift_clearance",
        grasp_likely_secured=True,
        cube_height=0.25,
        initial_cube_height=0.25,
    )

    assert reason == "cube_not_lifted"


def test_stage_failure_reason_reports_place_contact_failure():
    from tasks.pick_place_evaluation import pick_place_stage_failure_reason

    reason = pick_place_stage_failure_reason(
        step_name="descend_place_contact",
        grasp_likely_secured=True,
        cube_height=0.31,
        initial_cube_height=0.25,
        table_contact_detected=False,
    )

    assert reason == "place_contact_not_reached"


def test_stage_failure_reason_reports_place_alignment_failure():
    from tasks.pick_place_evaluation import pick_place_stage_failure_reason

    reason = pick_place_stage_failure_reason(
        step_name="descend_place_contact",
        grasp_likely_secured=True,
        cube_height=0.31,
        initial_cube_height=0.25,
        table_contact_detected=True,
        place_alignment_reached=False,
    )

    assert reason == "place_alignment_not_reached"


def test_only_pre_grasp_position_stages_are_blocking():
    from tasks.pick_place_evaluation import is_blocking_position_stage

    assert is_blocking_position_stage("hover_pick") is True
    assert is_blocking_position_stage("descend_pick") is True
    assert is_blocking_position_stage("lift_clearance") is True
    assert is_blocking_position_stage("translate_outboard") is True
    assert is_blocking_position_stage("translate_lateral_place") is True
    assert is_blocking_position_stage("translate_inboard_place") is True
    assert is_blocking_position_stage("place_hover") is True
    assert is_blocking_position_stage("descend_place_contact") is False
