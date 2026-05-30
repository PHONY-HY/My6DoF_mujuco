from pathlib import Path

import pytest


mujoco = pytest.importorskip("mujoco")


ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = ROOT / "mujoco_assets" / "my6dof"


def test_my6dof_base_model_loads_with_expected_names():
    model = mujoco.MjModel.from_xml_path(str(ASSET_ROOT / "my6dof_base.xml"))

    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(model.njnt)
    ]
    actuator_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
    ]

    assert joint_names[:6] == [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]
    assert "ee_site" in [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        for i in range(model.nsite)
    ]
    assert actuator_names == [
        "joint_1_ctrl",
        "joint_2_ctrl",
        "joint_3_ctrl",
        "joint_4_ctrl",
        "joint_5_ctrl",
        "joint_6_ctrl",
        "left_finger_ctrl",
        "right_finger_ctrl",
    ]


def test_follow_and_pick_place_scenes_load():
    follow_model = mujoco.MjModel.from_xml_path(str(ASSET_ROOT / "scene_my6dof_follow_target.xml"))
    pick_model = mujoco.MjModel.from_xml_path(str(ASSET_ROOT / "scene_my6dof_pick_place.xml"))

    assert follow_model.nq >= 8
    assert pick_model.nq >= 15
    assert mujoco.mj_name2id(follow_model, mujoco.mjtObj.mjOBJ_BODY, "follow_target") >= 0
    assert mujoco.mj_name2id(pick_model, mujoco.mjtObj.mjOBJ_BODY, "grasp_cube") >= 0
