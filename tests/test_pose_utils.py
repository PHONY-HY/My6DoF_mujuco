import numpy as np
import pinocchio as pin
import pytest

from ur5_style_arm.pose_utils import pose_input_to_se3, se3_to_matrix, se3_to_pose_dict


def test_pose_dict_round_trip_preserves_position_and_normalizes_quaternion():
    pose = {
        "position": np.array([0.3, -0.1, 0.5], dtype=float),
        "quaternion": np.array([0.0, 0.0, 0.70710678, 0.70710678], dtype=float),
    }

    transform = pose_input_to_se3(pose)
    result = se3_to_pose_dict(transform)

    assert isinstance(transform, pin.SE3)
    assert np.allclose(result["position"], pose["position"])
    assert np.isclose(np.linalg.norm(result["quaternion"]), 1.0)


def test_matrix_input_round_trip_matches_original_translation():
    matrix = np.eye(4)
    matrix[:3, 3] = np.array([0.2, 0.4, 0.6], dtype=float)

    transform = pose_input_to_se3(matrix)
    result_matrix = se3_to_matrix(transform)

    assert np.allclose(result_matrix[:3, 3], matrix[:3, 3])
    assert result_matrix.shape == (4, 4)


def test_invalid_quaternion_raises_value_error():
    pose = {
        "position": np.array([0.0, 0.0, 0.0], dtype=float),
        "quaternion": np.array([0.0, 0.0, 0.0, 0.0], dtype=float),
    }

    with pytest.raises(ValueError, match="quaternion"):
        pose_input_to_se3(pose)
