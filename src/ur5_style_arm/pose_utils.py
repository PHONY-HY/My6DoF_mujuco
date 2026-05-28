from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np
import pinocchio as pin


def _as_array(values: Sequence[float], *, expected_shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != expected_shape:
        raise ValueError(f"{name} must have shape {expected_shape}, got {array.shape}")
    return array


def _normalized_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    quat = _as_array(quaternion, expected_shape=(4,), name="quaternion")
    norm = np.linalg.norm(quat)
    if norm <= 0.0:
        raise ValueError("quaternion must have non-zero norm")
    return quat / norm


def pose_input_to_se3(target_pose: Mapping[str, Sequence[float]] | np.ndarray) -> pin.SE3:
    if isinstance(target_pose, Mapping):
        if "position" not in target_pose or "quaternion" not in target_pose:
            raise ValueError("pose dictionary must contain 'position' and 'quaternion'")

        position = _as_array(target_pose["position"], expected_shape=(3,), name="position")
        quaternion = _normalized_quaternion(target_pose["quaternion"])
        rotation = pin.Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2]).matrix()
        return pin.SE3(rotation, position)

    matrix = np.asarray(target_pose, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"pose matrix must have shape (4, 4), got {matrix.shape}")

    return pin.SE3(matrix[:3, :3], matrix[:3, 3])


def se3_to_matrix(transform: pin.SE3) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = transform.rotation
    matrix[:3, 3] = transform.translation
    return matrix


def se3_to_pose_dict(transform: pin.SE3) -> dict[str, np.ndarray]:
    quaternion = pin.Quaternion(transform.rotation)
    quat_xyzw = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=float,
    )

    return {
        "position": np.array(transform.translation, dtype=float),
        "quaternion": quat_xyzw / np.linalg.norm(quat_xyzw),
    }
