import numpy as np

from tasks.pick_place_plan import build_pick_place_plan


def test_pick_place_plan_contains_expected_stage_order():
    pick_position = np.array([0.35, 0.0, 0.32], dtype=float)
    cube_position = np.array([0.35, 0.0, 0.25], dtype=float)
    place_position = np.array([0.55, 0.1, 0.28], dtype=float)

    plan = build_pick_place_plan(pick_position, cube_position, place_position)

    assert [step["name"] for step in plan] == [
        "hover_pick",
        "descend_pick",
        "close_gripper",
<<<<<<< HEAD
        "lift",
        "hover_place",
        "descend_place",
        "open_gripper",
        "retreat",
    ]
    assert plan[0]["gripper_opening"] == 0.04
    assert plan[2]["gripper_opening"] == 0.0
    assert plan[6]["gripper_opening"] == 0.04
=======
        "lift_clearance",
        "translate_outboard",
        "translate_lateral_place",
        "translate_inboard_place",
        "place_hover",
        "descend_place_contact",
        "settle_place",
        "release_place",
        "retreat",
    ]
    assert plan[0]["gripper_opening"] == 0.025
    assert plan[2]["gripper_opening"] == 0.0145
    assert plan[10]["gripper_opening"] == 0.025


def test_pick_place_plan_uses_pose_control_through_grasp_and_transport():
    plan = build_pick_place_plan(
        np.array([0.35, 0.0, 0.32], dtype=float),
        np.array([0.35, 0.0, 0.25], dtype=float),
        np.array([0.55, 0.1, 0.28], dtype=float),
    )

    assert plan[0]["control_mode"] == "pose"
    assert plan[1]["control_mode"] == "pose"
    assert plan[2]["control_mode"] == "pose"
    assert plan[3]["control_mode"] == "pose"
    assert plan[4]["control_mode"] == "pose"
    assert plan[5]["control_mode"] == "pose"
    assert plan[6]["control_mode"] == "pose"


def test_pick_place_plan_uses_pose_mode_for_contact_aware_place_approach():
    plan = build_pick_place_plan(
        np.array([0.35, 0.0, 0.32], dtype=float),
        np.array([0.35, 0.0, 0.25], dtype=float),
        np.array([0.55, 0.1, 0.28], dtype=float),
    )

    assert plan[6]["control_mode"] == "pose"
    assert plan[7]["control_mode"] == "pose"
    assert plan[8]["control_mode"] == "pose"


def test_pick_place_plan_keeps_gripper_closed_until_release_phase():
    plan = build_pick_place_plan(
        np.array([0.35, 0.0, 0.32], dtype=float),
        np.array([0.35, 0.0, 0.25], dtype=float),
        np.array([0.55, 0.1, 0.28], dtype=float),
    )

    settle_step = next(step for step in plan if step["name"] == "settle_place")
    release_step = next(step for step in plan if step["name"] == "release_place")

    assert float(settle_step["gripper_opening"]) == 0.0145
    assert float(release_step["gripper_opening"]) == 0.025
    assert int(settle_step["controller_steps"]) >= 10
    assert int(release_step["controller_steps"]) >= 20


def test_pick_place_plan_uses_transport_height_above_pick_and_place():
    pick_position = np.array([0.35, 0.0, 0.23], dtype=float)
    cube_position = np.array([0.35, 0.0, 0.16], dtype=float)
    place_position = np.array([0.55, 0.1, 0.19], dtype=float)

    plan = build_pick_place_plan(pick_position, cube_position, place_position)
    lift_clearance_step = next(step for step in plan if step["name"] == "lift_clearance")
    translate_step = next(step for step in plan if step["name"] == "translate_inboard_place")

    assert float(lift_clearance_step["target_position"][2]) > float(cube_position[2]) + 0.08
    assert float(lift_clearance_step["target_position"][2]) > float(place_position[2]) + 0.08
    assert float(translate_step["target_position"][2]) == float(lift_clearance_step["target_position"][2])


def test_pick_place_plan_lifts_before_translation_and_descends_only_above_place():
    pick_position = np.array([0.35, 0.0, 0.23], dtype=float)
    cube_position = np.array([0.35, 0.0, 0.16], dtype=float)
    place_position = np.array([0.55, 0.1, 0.19], dtype=float)

    plan = build_pick_place_plan(pick_position, cube_position, place_position)
    lift_clearance_step = next(step for step in plan if step["name"] == "lift_clearance")
    translate_outboard_step = next(step for step in plan if step["name"] == "translate_outboard")
    lateral_step = next(step for step in plan if step["name"] == "translate_lateral_place")
    inboard_step = next(step for step in plan if step["name"] == "translate_inboard_place")
    place_hover_step = next(step for step in plan if step["name"] == "place_hover")
    descend_step = next(step for step in plan if step["name"] == "descend_place_contact")

    assert np.allclose(lift_clearance_step["target_position"][:2], cube_position[:2])
    assert float(translate_outboard_step["target_position"][0]) > max(float(cube_position[0]), float(place_position[0]))
    assert float(translate_outboard_step["target_position"][1]) == float(cube_position[1])
    assert float(translate_outboard_step["target_position"][2]) == float(lift_clearance_step["target_position"][2])
    assert float(lateral_step["target_position"][0]) == float(translate_outboard_step["target_position"][0])
    assert float(lateral_step["target_position"][1]) == float(place_position[1])
    assert float(lateral_step["target_position"][2]) == float(lift_clearance_step["target_position"][2])
    assert float(inboard_step["target_position"][0]) == float(place_position[0])
    assert float(inboard_step["target_position"][1]) == float(place_position[1])
    assert float(inboard_step["target_position"][2]) == float(lift_clearance_step["target_position"][2])
    assert np.allclose(place_hover_step["target_position"][:2], place_position[:2])
    assert np.allclose(descend_step["target_position"][:2], place_position[:2])
    assert float(descend_step["target_position"][2]) < float(place_hover_step["target_position"][2])
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
