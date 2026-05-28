import numpy as np
import pytest

from ur5_style_arm import UR5StyleArm
from ur5_style_arm.pose_utils import pose_input_to_se3, se3_to_matrix


def test_forward_kinematics_returns_matrix_and_quaternion_outputs():
    arm = UR5StyleArm()
    q = np.array([0.0, -1.2, 1.2, -0.8, 0.2, 0.1], dtype=float)

    matrix = arm.forward_kinematics(q, output="matrix")
    quat_pose = arm.forward_kinematics(q, output="quat")

    assert matrix.shape == (4, 4)
    assert quat_pose["position"].shape == (3,)
    assert quat_pose["quaternion"].shape == (4,)

    reconstructed = se3_to_matrix(pose_input_to_se3(quat_pose))
    assert np.allclose(matrix, reconstructed, atol=1e-8)


def test_forward_kinematics_validates_joint_shape():
    arm = UR5StyleArm()

    with pytest.raises(ValueError, match="shape"):
        arm.forward_kinematics(np.zeros(5), output="matrix")
