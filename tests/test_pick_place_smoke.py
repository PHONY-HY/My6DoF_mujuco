import pytest


pytest.importorskip("mujoco")

from tasks.my6dof_pick_place import PickPlaceTask


def test_pick_place_headless_episode_returns_record_with_object_pose():
    task = PickPlaceTask(interactive=False, record_path=None)
    record = task.run_scripted_step()

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert "position" in record.object_pose
    assert set(record.gripper_state.keys()) == {"left", "right"}
