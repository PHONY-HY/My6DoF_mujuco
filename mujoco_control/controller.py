from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class ControllerConfig:
    damping: float = 1e-4
    step_scale: float = 0.2
    position_gain: float = 1.0


class DifferentialIKController:
    """Lightweight position-only differential IK controller for the my6dof arm."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        ee_site_name: str = "ee_site",
        config: ControllerConfig | None = None,
    ) -> None:
        self.model = model
        self.data = data
        self.ee_site_name = ee_site_name
        self.ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)
        self.config = config or ControllerConfig()

        self.arm_joint_names = [f"joint_{index}" for index in range(1, 7)]
        self.arm_joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.arm_joint_names
        ]
        self.arm_actuator_names = [f"joint_{index}_ctrl" for index in range(1, 7)]
        self.arm_actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.arm_actuator_names
        ]
        self.arm_qpos_adr = np.array([self.model.jnt_qposadr[joint_id] for joint_id in self.arm_joint_ids], dtype=int)
        self.arm_qvel_adr = np.array([self.model.jnt_dofadr[joint_id] for joint_id in self.arm_joint_ids], dtype=int)
        self.arm_joint_ranges = self.model.jnt_range[self.arm_joint_ids].copy()

        self.left_gripper_actuator_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "left_finger_ctrl",
        )
        self.right_gripper_actuator_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "right_finger_ctrl",
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
                raise ValueError("controller linear system is singular")

            if max_row != pivot_idx:
                augmented[[pivot_idx, max_row]] = augmented[[max_row, pivot_idx]]

            augmented[pivot_idx] = augmented[pivot_idx] / augmented[pivot_idx, pivot_idx]

            for row_idx in range(n):
                if row_idx == pivot_idx:
                    continue
                factor = augmented[row_idx, pivot_idx]
                augmented[row_idx] = augmented[row_idx] - factor * augmented[pivot_idx]

        return augmented[:, -1]

    def current_joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.arm_qpos_adr].copy()

    def current_end_effector_position(self) -> np.ndarray:
        return self.data.site_xpos[self.ee_site_id].copy()

    def set_gripper_opening(self, opening: float) -> dict[str, float]:
        opening = float(np.clip(opening, 0.0, 0.04))
        self.data.ctrl[self.left_gripper_actuator_id] = opening
        self.data.ctrl[self.right_gripper_actuator_id] = opening
        return {"left": opening, "right": opening}

    def step_to_position(self, target_position: np.ndarray) -> np.ndarray:
        target_position = np.asarray(target_position, dtype=float)

        jacp = np.zeros((3, self.model.nv), dtype=float)
        jacr = np.zeros((3, self.model.nv), dtype=float)
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.ee_site_id)

        error = target_position - self.current_end_effector_position()
        task_jacobian = jacp[:, self.arm_qvel_adr]
        lhs = self._left_gram(task_jacobian) + self.config.damping * np.eye(3)
        delta_q = self._matvec(
            task_jacobian.T,
            self._solve_linear_system(lhs, self.config.position_gain * error),
        )

        current_q = self.current_joint_positions()
        next_q = np.clip(
            current_q + self.config.step_scale * delta_q,
            self.arm_joint_ranges[:, 0],
            self.arm_joint_ranges[:, 1],
        )

        for actuator_id, value in zip(self.arm_actuator_ids, next_q):
            self.data.ctrl[actuator_id] = value

        return next_q
