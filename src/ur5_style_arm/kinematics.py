from __future__ import annotations

import numpy as np
import pinocchio as pin

from .exceptions import IKConvergenceError
from .pose_utils import pose_input_to_se3, se3_to_matrix, se3_to_pose_dict
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

    def _base_frame_jacobian(self, q: np.ndarray) -> np.ndarray:
        q = self._validate_vector(q, name="q")
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

    @staticmethod
    def _left_gram(matrix: np.ndarray) -> np.ndarray:
        return np.einsum("ik,jk->ij", matrix, matrix)

    @staticmethod
    def _matvec(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
        return np.einsum("ij,j->i", matrix, vector)

    @staticmethod
    def _solve_linear_system(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
        augmented = np.hstack(
            (
                np.asarray(matrix, dtype=float).copy(),
                np.asarray(vector, dtype=float).reshape(-1, 1).copy(),
            )
        )
        n = augmented.shape[0]

        for pivot_idx in range(n):
            max_row = pivot_idx + int(np.argmax(np.abs(augmented[pivot_idx:, pivot_idx])))
            pivot_value = augmented[max_row, pivot_idx]
            if abs(pivot_value) <= 1e-12:
                raise ValueError("linear system is singular")

            if max_row != pivot_idx:
                augmented[[pivot_idx, max_row]] = augmented[[max_row, pivot_idx]]

            augmented[pivot_idx] = augmented[pivot_idx] / augmented[pivot_idx, pivot_idx]

            for row_idx in range(n):
                if row_idx == pivot_idx:
                    continue
                factor = augmented[row_idx, pivot_idx]
                augmented[row_idx] = augmented[row_idx] - factor * augmented[pivot_idx]

        return augmented[:, -1]

    def forward_kinematics(self, q: np.ndarray, output: str = "matrix") -> np.ndarray | dict[str, np.ndarray]:
        transform = self._frame_pose(q)

        if output == "matrix":
            return se3_to_matrix(transform)
        if output == "quat":
            return se3_to_pose_dict(transform)

        raise ValueError("output must be either 'matrix' or 'quat'")

    def inverse_kinematics(
        self,
        target_pose: dict[str, np.ndarray] | np.ndarray,
        q0: np.ndarray | None = None,
        max_iters: int = 200,
        tol: float = 1e-6,
    ) -> tuple[np.ndarray, dict[str, float | int]]:
        target = pose_input_to_se3(target_pose)
        q = self.neutral_q.copy() if q0 is None else self._validate_vector(q0, name="q0").copy()
        damping = 1e-6

        for iteration in range(1, max_iters + 1):
            current = self._frame_pose(q)
            error_transform = current.actInv(target)
            residual = pin.log6(error_transform).vector
            position_error_norm = float(np.linalg.norm(residual[:3]))
            orientation_error_norm = float(np.linalg.norm(residual[3:]))
            final_error_norm = float(np.linalg.norm(residual))

            if final_error_norm <= tol:
                info = {
                    "iterations": iteration,
                    "final_error_norm": final_error_norm,
                    "position_error_norm": position_error_norm,
                    "orientation_error_norm": orientation_error_norm,
                }
                return q.copy(), info

            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_frame_id,
                pin.ReferenceFrame.LOCAL,
            )
            lhs = self._left_gram(jacobian) + damping * np.eye(6)
            delta_q = self._matvec(jacobian.T, self._solve_linear_system(lhs, residual))
            q = np.clip(q + delta_q, self.lower_limits, self.upper_limits)

        raise IKConvergenceError(
            "inverse kinematics did not converge",
            final_q=q,
            iterations=max_iters,
            residual_twist=residual,
            position_error_norm=position_error_norm,
            orientation_error_norm=orientation_error_norm,
        )

    def forward_velocity(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        q = self._validate_vector(q, name="q")
        qdot = self._validate_vector(qdot, name="qdot")
        jacobian = self._base_frame_jacobian(q)
        return self._matvec(jacobian, qdot)

    def inverse_velocity(self, q: np.ndarray, ee_twist: np.ndarray, damping: float = 1e-6) -> np.ndarray:
        q = self._validate_vector(q, name="q")
        ee_twist = self._validate_vector(ee_twist, name="ee_twist")
        jacobian = self._base_frame_jacobian(q)
        lhs = self._left_gram(jacobian) + damping * np.eye(6)
        return self._matvec(jacobian.T, self._solve_linear_system(lhs, ee_twist))
