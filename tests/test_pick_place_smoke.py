import numpy as np
import pytest


mujoco = pytest.importorskip("mujoco")

from tasks.my6dof_pick_place import PickPlaceTask
<<<<<<< HEAD
=======
from tasks.pick_place_evaluation import PickPlaceEpisodeResult
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
from ur5_style_arm.robot_model import NEUTRAL_Q


def test_pick_place_headless_episode_returns_record_with_object_pose():
    task = PickPlaceTask(interactive=False, record_path=None)
    record = task.run_scripted_step()

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert "position" in record.object_pose
    assert set(record.gripper_state.keys()) == {"left", "right"}


def test_pick_place_initializes_robot_at_neutral_pose():
    task = PickPlaceTask(interactive=False, record_path=None)

<<<<<<< HEAD
    assert np.allclose(task.controller.current_joint_positions(), NEUTRAL_Q, atol=1e-6)
=======
    assert np.allclose(task.controller.current_joint_positions(), NEUTRAL_Q, atol=1e-6)


def test_pick_place_task_no_longer_uses_attachment_state():
    task = PickPlaceTask(interactive=False, record_path=None)

    assert not hasattr(task, "cube_attached")
    assert not hasattr(task, "attach_offset")
    assert not hasattr(task, "_try_attach_cube")


def test_pick_place_episode_reports_explicit_result():
    task = PickPlaceTask(interactive=False, record_path=None)
    result = task.run_episode()

    assert isinstance(result, PickPlaceEpisodeResult)
    assert isinstance(result.grasp_success, bool)
    assert isinstance(result.episode_success, bool)


def test_pick_place_episode_failure_reason_is_explicit_when_unsuccessful():
    task = PickPlaceTask(interactive=False, record_path=None)
    result = task.run_episode()

    if not result.episode_success:
        assert result.failure_reason is not None


def test_pick_place_task_builds_explicit_transport_path_after_grasp():
    task = PickPlaceTask(interactive=False, record_path=None)
    plan = task._build_episode_plan()

    assert [step["name"] for step in plan[3:9]] == [
        "lift_clearance",
        "translate_outboard",
        "translate_lateral_place",
        "translate_inboard_place",
        "place_hover",
        "descend_place_contact",
    ]


def test_pick_place_neutral_pose_places_grasp_site_in_front_of_base():
    task = PickPlaceTask(interactive=False, record_path=None)
    grasp_position = task.data.site_xpos[task.grasp_site_id].copy()

    assert 0.18 <= float(grasp_position[0]) <= 0.24
    assert 0.19 <= float(grasp_position[2]) <= 0.24


def test_pick_place_neutral_pose_starts_near_pick_work_area():
    task = PickPlaceTask(interactive=False, record_path=None)
    grasp_position = task.data.site_xpos[task.grasp_site_id].copy()
    cube_position = task.data.xpos[task.cube_body_id].copy()

    assert abs(float(grasp_position[0]) - float(cube_position[0])) < 0.10


def test_pick_place_scene_workspace_positions_remain_calibrated():
    task = PickPlaceTask(interactive=False, record_path=None)
    table_body_id = mujoco.mj_name2id(task.model, mujoco.mjtObj.mjOBJ_BODY, "table")

    assert float(task.model.body_pos[table_body_id][0]) == pytest.approx(0.46)
    assert float(task.model.body_pos[task.pick_target_body_id][0]) == pytest.approx(0.24)
    assert float(task.model.body_pos[task.cube_body_id][0]) == pytest.approx(0.24)
    assert float(task.data.mocap_pos[task.place_target_mocap_id][0]) == pytest.approx(0.36)


def test_pick_place_scene_does_not_start_with_robot_table_overlap():
    task = PickPlaceTask(interactive=False, record_path=None)
    forbidden_pairs = {
        tuple(sorted(("base_geom", "table_geom"))),
        tuple(sorted(("link1_geom", "table_geom"))),
        tuple(sorted(("link2_geom", "table_geom"))),
    }
    contacts = set()
    for index in range(task.data.ncon):
        contact = task.data.contact[index]
        geom1 = mujoco.mj_id2name(task.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2 = mujoco.mj_id2name(task.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        contacts.add(tuple(sorted((geom1, geom2))))

    assert forbidden_pairs.isdisjoint(contacts)

def test_close_gripper_stage_keeps_grasp_site_close_to_cube():
    task = PickPlaceTask(interactive=False, record_path=None)
    plan = task._build_episode_plan()

    for step in plan[:3]:
        task._execute_stage(step)

    cube_position = task.data.xpos[task.cube_body_id].copy()
    grasp_position = task.data.site_xpos[task.grasp_site_id].copy()

    assert float(np.linalg.norm(grasp_position - cube_position)) < 0.04
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
