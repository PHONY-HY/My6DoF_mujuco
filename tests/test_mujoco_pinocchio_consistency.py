from pathlib import Path

import numpy as np
import pytest

from ur5_style_arm import UR5StyleArm


mujoco = pytest.importorskip("mujoco")

from ur5_style_arm.mujoco_verify import compare_end_effector_positions


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_follow_target.xml"
PICK_SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_pick_place.xml"


def test_pinocchio_and_mujoco_end_effector_positions_are_close_at_neutral_pose():
    arm = UR5StyleArm()
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    report = compare_end_effector_positions(arm, model, data, arm.neutral_q)

    assert report["pinocchio_position"].shape == (3,)
    assert report["mujoco_position"].shape == (3,)
    assert report["position_error_norm"] < 0.35


def test_pinocchio_and_mujoco_end_effector_positions_are_tightly_aligned_at_neutral_pose():
    arm = UR5StyleArm()
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    report = compare_end_effector_positions(arm, model, data, arm.neutral_q)

    assert report["position_error_norm"] < 0.08


def test_hover_pick_target_configuration_is_kinematically_reachable_without_self_collision():
    from tasks.pick_place_plan import build_pick_place_plan
    from tasks.my6dof_pick_place import PickPlaceTask

    task = PickPlaceTask(interactive=False, record_path=None)
    model = task.model
    data = task.data

    cube_position = data.xpos[task.cube_body_id].copy()
    pick_position = model.body_pos[task.pick_target_body_id].copy()
    place_position = data.mocap_pos[task.place_target_mocap_id].copy()
    hover_target = build_pick_place_plan(pick_position, cube_position, place_position)[0]["target_position"]

    q_target = task._plan_joint_target(hover_target)
    assert q_target is not None
    data.qpos[:6] = q_target
    mujoco.mj_forward(model, data)

    contacts = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        contacts.add(tuple(sorted((geom1, geom2))))

    assert tuple(sorted(("link3_geom", "link5_geom"))) not in contacts
    assert float(np.linalg.norm(data.site_xpos[task.grasp_site_id].copy() - hover_target)) < 0.02


def test_pick_place_joint_target_places_grasp_site_close_to_hover_target():
    from tasks.my6dof_pick_place import PickPlaceTask
    from tasks.pick_place_plan import build_pick_place_plan

    task = PickPlaceTask(interactive=False, record_path=None)
    pick_position = task.model.body_pos[task.pick_target_body_id].copy()
    cube_position = task.data.xpos[task.cube_body_id].copy()
    place_position = task.data.mocap_pos[task.place_target_mocap_id].copy()
    hover_target = build_pick_place_plan(pick_position, cube_position, place_position)[0]["target_position"]

    q_target = task._plan_joint_target(hover_target)
    assert q_target is not None

    task.data.qpos[task.controller.arm_qpos_adr] = q_target
    mujoco.mj_forward(task.model, task.data)
    distance = task._distance_to_target(hover_target)

    assert distance < 0.01
