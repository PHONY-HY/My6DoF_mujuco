from __future__ import annotations

import numpy as np
import pinocchio as pin

from .pose_utils import se3_to_matrix, se3_to_pose_dict
from .robot_model import build_ur5_style_model


class UR5StyleArm:
    """UR5-style 6DOF manipulator wrapper."""

    def __init__(self) -> None:
        (
            self.model,
            self.data,
            self.ee_frame_id,
            self.lower_limits,
            self.upper_limits,
            self.neutral_q,
        ) = build_ur5_style_model()

    @staticmethod
    def _validate_vector(values: np.ndarray, *, name: str) -> np.ndarray:
        array = np.asarray(values, dtype=float)
        if array.shape != (6,):
            raise ValueError(f"{name} must have shape (6,), got {array.shape}")
        return array

    def _frame_pose(self, q: np.ndarray) -> pin.SE3:
        q = self._validate_vector(q, name="q")
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.ee_frame_id]

    def forward_kinematics(self, q: np.ndarray, output: str = "matrix") -> np.ndarray | dict[str, np.ndarray]:
        transform = self._frame_pose(q)

        if output == "matrix":
            return se3_to_matrix(transform)
        if output == "quat":
            return se3_to_pose_dict(transform)

        raise ValueError("output must be either 'matrix' or 'quat'")
