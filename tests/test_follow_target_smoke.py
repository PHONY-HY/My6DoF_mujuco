import numpy as np
import pytest


pytest.importorskip("mujoco")

from tasks.my6dof_follow_target import FollowTargetTask
from ur5_style_arm.robot_model import NEUTRAL_Q


def test_follow_target_headless_step_returns_record():
    task = FollowTargetTask(interactive=False, record_path=None)
    target = np.array([0.42, 0.05, 0.42], dtype=float)
    initial_error = np.linalg.norm(task.controller.current_end_effector_position() - target)
    record = task.step_to_target(target)
    final_error = np.linalg.norm(task.controller.current_end_effector_position() - target)

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert record.object_pose is None
    assert abs(record.target_pose["position"][0] - 0.42) < 1e-9
    assert final_error < initial_error


def test_follow_target_initializes_robot_at_neutral_pose():
    task = FollowTargetTask(interactive=False, record_path=None)

    assert np.allclose(task.controller.current_joint_positions(), NEUTRAL_Q, atol=1e-6)