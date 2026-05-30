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
        "lift",
        "hover_place",
        "descend_place",
        "open_gripper",
        "retreat",
    ]
    assert plan[0]["gripper_opening"] == 0.04
    assert plan[2]["gripper_opening"] == 0.0
    assert plan[6]["gripper_opening"] == 0.04