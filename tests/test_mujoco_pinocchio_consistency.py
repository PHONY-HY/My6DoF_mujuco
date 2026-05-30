from pathlib import Path

import numpy as np
import pytest

from ur5_style_arm import UR5StyleArm


mujoco = pytest.importorskip("mujoco")

from ur5_style_arm.mujoco_verify import compare_end_effector_positions


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_follow_target.xml"


def test_pinocchio_and_mujoco_end_effector_positions_are_close_at_neutral_pose():
    arm = UR5StyleArm()
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    report = compare_end_effector_positions(arm, model, data, arm.neutral_q)

    assert report["pinocchio_position"].shape == (3,)
    assert report["mujoco_position"].shape == (3,)
    assert report["position_error_norm"] < 0.35
