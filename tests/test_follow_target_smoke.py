import numpy as np
import pytest


pytest.importorskip("mujoco")

<<<<<<< HEAD
from tasks.my6dof_follow_target import FollowTargetTask
from ur5_style_arm.robot_model import NEUTRAL_Q
=======
from tasks.my6dof_follow_target import FOLLOW_HOME_Q, FollowTargetTask
>>>>>>> 1f097dd (Update .gitignore and restage clean files)


def test_follow_target_headless_step_returns_record():
    task = FollowTargetTask(interactive=False, record_path=None)
<<<<<<< HEAD
    target = np.array([0.42, 0.05, 0.42], dtype=float)
=======
    start_target = np.array(task._target_pose()["position"], dtype=float)
    target = start_target + np.array([0.01, 0.0, -0.02], dtype=float)
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
    initial_error = np.linalg.norm(task.controller.current_end_effector_position() - target)
    record = task.step_to_target(target)
    final_error = np.linalg.norm(task.controller.current_end_effector_position() - target)

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert record.object_pose is None
<<<<<<< HEAD
    assert abs(record.target_pose["position"][0] - 0.42) < 1e-9
    assert final_error < initial_error


def test_follow_target_initializes_robot_at_neutral_pose():
    task = FollowTargetTask(interactive=False, record_path=None)

    assert np.allclose(task.controller.current_joint_positions(), NEUTRAL_Q, atol=1e-6)
=======
    assert abs(record.target_pose["position"][0] - float(target[0])) < 1e-9
    assert final_error < initial_error


def test_follow_target_initializes_robot_at_follow_home_pose():
    task = FollowTargetTask(interactive=False, record_path=None)

    assert np.allclose(task.controller.current_joint_positions(), FOLLOW_HOME_Q, atol=1e-6)


def test_follow_target_home_pose_starts_in_top_right_workspace():
    task = FollowTargetTask(interactive=False, record_path=None)
    ee_position = task.controller.current_end_effector_position()

    assert 0.40 <= float(ee_position[0]) <= 0.60
    assert 0.60 <= float(ee_position[2]) <= 0.70


def test_follow_target_initial_target_starts_below_end_effector():
    task = FollowTargetTask(interactive=False, record_path=None)
    ee_position = task.controller.current_end_effector_position()
    target_position = np.array(task._target_pose()["position"], dtype=float)

    assert abs(float(target_position[0]) - float(ee_position[0])) < 1e-6
    assert abs(float(target_position[1]) - float(ee_position[1])) < 1e-6
    assert float(target_position[2]) < float(ee_position[2])
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
