import numpy as np
import pytest


pytest.importorskip("mujoco")

from tasks.my6dof_follow_target import FollowTargetTask


def test_follow_target_headless_step_returns_record():
    task = FollowTargetTask(interactive=False, record_path=None)
    record = task.step_to_target(np.array([0.42, 0.05, 0.42], dtype=float))

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert record.object_pose is None
    assert abs(record.target_pose["position"][0] - 0.42) < 1e-9
