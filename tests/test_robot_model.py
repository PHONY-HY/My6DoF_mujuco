import numpy as np

from ur5_style_arm.robot_model import build_ur5_style_model


def test_robot_model_has_six_dof_and_tool_frame():
    model, data, ee_frame_id, lower, upper, neutral = build_ur5_style_model()

    assert model.nq == 6
    assert model.nv == 6
    assert model.frames[ee_frame_id].name == "tool0"
    assert lower.shape == (6,)
    assert upper.shape == (6,)
    assert neutral.shape == (6,)
    assert np.all(lower <= neutral)
    assert np.all(neutral <= upper)
