from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco
import numpy as np

from mujoco_control import DifferentialIKController
<<<<<<< HEAD
from tasks.pick_place_plan import build_pick_place_plan
from tasks.recording import TrajectoryFrame, TrajectoryRecorder
=======
from tasks.pick_place_evaluation import (
    PickPlaceEpisodeResult,
    evaluate_pick_place_episode,
    is_blocking_position_stage,
    is_gripper_closed_for_grasp,
    pick_place_stage_failure_reason,
)
from tasks.pick_place_plan import (
    GRIPPER_OPENING_CLOSE,
    GRIPPER_OPENING_OPEN,
    build_pick_place_plan,
)
from tasks.recording import TrajectoryFrame, TrajectoryRecorder
from ur5_style_arm import IKConvergenceError, UR5StyleArm
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
from ur5_style_arm.robot_model import NEUTRAL_Q


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_pick_place.xml"


class PickPlaceTask:
    PLACE_ALIGNMENT_TOLERANCE = 0.04
    PLACE_CONTACT_XY_GAIN = 0.35
    PLACE_CONTACT_XY_MAX_CORRECTION = 0.03

    def __init__(self, interactive: bool = True, record_path: str | None = None) -> None:
        self.interactive = interactive
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.controller = DifferentialIKController(self.model, self.data, "grasp_site")
        self.kinematic_arm = UR5StyleArm()

        self.place_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "place_target")
        self.place_target_mocap_id = int(self.model.body_mocapid[self.place_target_body_id])
        self.pick_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pick_target")
        self.cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "grasp_cube")
        self.cube_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "grasp_cube_freejoint")
        self.cube_qpos_adr = self.model.jnt_qposadr[self.cube_joint_id]
        self.cube_qvel_adr = self.model.jnt_dofadr[self.cube_joint_id]
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self.grasp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "grasp_site")
        self.recorder = TrajectoryRecorder(record_path) if record_path is not None else None

<<<<<<< HEAD
        self.cube_attached = False
        self.attach_offset = np.array([0.0, 0.0, -0.05], dtype=float)
        self.phase_index = 0
        self.phase_tick = 0
        self.phase_duration = 35
        self._initialize_home_pose()

    def _initialize_home_pose(self) -> None:
        self.controller.set_arm_joint_positions(NEUTRAL_Q)
        self.controller.set_gripper_opening(0.04)
        self.data.mocap_quat[self.place_target_mocap_id] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        mujoco.mj_forward(self.model, self.data)
=======
        self.phase_index = 0
        self.phase_tick = 0
        self.phase_duration = 35
        self.initial_cube_height = 0.0
        self.grasp_target_rotation = np.eye(3)
        self.transport_target_rotation = np.eye(3)
        self.grasp_site_offset_local = np.array([0.0, 0.0, -0.13], dtype=float)
        self.grasp_site_rotation_local = np.eye(3)
        self.failure_reason: str | None = None
        self.current_step_name = "idle"
        self.current_target_position = np.zeros(3, dtype=float)
        self.interactive_plan: list[dict[str, np.ndarray | float | str]] | None = None
        self.active_stage_joint_target: np.ndarray | None = None
        self.active_stage_start_position = np.zeros(3, dtype=float)
        self.active_contact_descent_hold_z: float | None = None
        self._initialize_home_pose()
>>>>>>> 1f097dd (Update .gitignore and restage clean files)

    def _initialize_home_pose(self) -> None:
        self.controller.set_arm_joint_positions(NEUTRAL_Q)
        self.controller.set_gripper_opening(0.04)
        self.data.mocap_quat[self.place_target_mocap_id] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        mujoco.mj_forward(self.model, self.data)
        self.initial_cube_height = float(self.data.xpos[self.cube_body_id][2])
        ee_position = self.data.site_xpos[self.ee_site_id].copy()
        ee_rotation = self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy()
        grasp_position = self.data.site_xpos[self.grasp_site_id].copy()
        self.grasp_target_rotation = self.data.site_xmat[self.grasp_site_id].reshape(3, 3).copy()
        self.grasp_site_offset_local = np.einsum("ij,j->i", ee_rotation.T, grasp_position - ee_position)
        self.grasp_site_rotation_local = np.einsum("ij,jk->ik", ee_rotation.T, self.grasp_target_rotation)
        self.transport_target_rotation = self.grasp_target_rotation.copy()

    @staticmethod
    def _downward_transport_rotation() -> np.ndarray:
        return np.eye(3, dtype=float)

    def _cube_pose(self) -> dict[str, list[float]]:
        return {
            "position": self.data.xpos[self.cube_body_id].copy().tolist(),
            "quaternion": self.data.xquat[self.cube_body_id].copy().tolist(),
        }

    def _target_pose(self) -> dict[str, list[float]]:
        return {
            "position": self.current_target_position.copy().tolist(),
            "quaternion": [1.0, 0.0, 0.0, 0.0],
        }

    def _current_gripper_state(self) -> dict[str, float]:
        left_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_finger_joint")
        right_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_finger_joint")
        return {
            "left": float(self.data.qpos[self.model.jnt_qposadr[left_joint_id]]),
            "right": float(self.data.qpos[self.model.jnt_qposadr[right_joint_id]]),
        }

    def _build_frame(self) -> TrajectoryFrame:
        return TrajectoryFrame(
            timestamp=float(self.data.time),
            joint_positions=self.controller.current_joint_positions().tolist(),
            gripper_state=self._current_gripper_state(),
            target_pose=self._target_pose(),
            object_pose=self._cube_pose(),
        )

    def _set_gripper(self, opening: float) -> dict[str, float]:
        return self.controller.set_gripper_opening(opening)

    def _distance_to_target(self, target_position: np.ndarray) -> float:
        grasp_position = self.data.site_xpos[self.grasp_site_id].copy()
        return float(np.linalg.norm(grasp_position - np.asarray(target_position, dtype=float)))

    def _build_target_pose_matrix(self, target_position: np.ndarray, target_rotation: np.ndarray | None = None) -> np.ndarray:
        target_pose = np.eye(4)
        if target_rotation is None:
            target_rotation = self.grasp_target_rotation
        target_rotation = np.asarray(target_rotation, dtype=float).reshape(3, 3)
        tool_rotation = np.einsum("ij,jk->ik", target_rotation, self.grasp_site_rotation_local.T)
        target_pose[:3, :3] = tool_rotation
        target_pose[:3, 3] = np.asarray(target_position, dtype=float) - (
            np.einsum("ij,j->i", tool_rotation, self.grasp_site_offset_local)
        )
        return target_pose

    def _plan_joint_target(self, target_position: np.ndarray, target_rotation: np.ndarray | None = None) -> np.ndarray | None:
        q_seed = self.controller.current_joint_positions()
        target_pose = self._build_target_pose_matrix(target_position, target_rotation=target_rotation)
        try:
            q_target, _ = self.kinematic_arm.inverse_kinematics(
                target_pose,
                q0=q_seed,
                max_iters=400,
                tol=1e-6,
            )
            return q_target
        except IKConvergenceError:
            return None

    def _build_episode_plan(self) -> list[dict[str, np.ndarray | float | str]]:
        pick_position = self.model.body_pos[self.pick_target_body_id].copy()
        cube_position = self.data.xpos[self.cube_body_id].copy()
        place_position = self.data.mocap_pos[self.place_target_mocap_id].copy()
        return build_pick_place_plan(pick_position, cube_position, place_position)

    def _step_with_target(self, target_position: np.ndarray, gripper_opening: float, controller_steps: int = 10) -> TrajectoryFrame:
        return self._step_with_target_mode(target_position, gripper_opening, controller_steps, control_mode="pose")

    def _step_with_target_mode(
        self,
        target_position: np.ndarray,
        gripper_opening: float,
        controller_steps: int = 10,
        control_mode: str = "pose",
        joint_target: np.ndarray | None = None,
        target_rotation: np.ndarray | None = None,
    ) -> TrajectoryFrame:
        target_position = np.asarray(target_position, dtype=float)
        self.current_target_position = target_position.copy()
        self._set_gripper(gripper_opening)
        mujoco.mj_forward(self.model, self.data)
<<<<<<< HEAD
=======
        q_target = joint_target
        should_use_joint_target = control_mode == "position" and float(gripper_opening) > 0.02
        if should_use_joint_target and q_target is None:
            current_rotation = self.data.site_xmat[self.grasp_site_id].reshape(3, 3).copy()
            q_target = self._plan_joint_target(target_position, target_rotation=current_rotation)
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
        for _ in range(controller_steps):
            if control_mode == "position":
                if q_target is not None and should_use_joint_target:
                    self.controller.command_arm_joint_positions(q_target)
                else:
                    self.controller.step_to_position(target_position)
            else:
                desired_rotation = self.grasp_target_rotation if target_rotation is None else np.asarray(target_rotation, dtype=float)
                self.controller.step_to_pose(target_position, desired_rotation)
            mujoco.mj_step(self.model, self.data)

        frame = self._build_frame()
        if self.recorder is not None:
            self.recorder.write(frame)
        return frame

    def _cube_height(self) -> float:
        return float(self.data.xpos[self.cube_body_id][2])

    def _cube_distance_to_grasp_site(self) -> float:
        cube_position = self.data.xpos[self.cube_body_id].copy()
        grasp_position = self.data.site_xpos[self.grasp_site_id].copy()
        return float(np.linalg.norm(cube_position - grasp_position))

    def _grasp_likely_secured(self) -> bool:
        gripper_state = self._current_gripper_state()
        return (
            self._cube_distance_to_grasp_site() < 0.03
            and is_gripper_closed_for_grasp(gripper_state)
        )

    def _cube_xy_distance_to_target(self, target_position: np.ndarray) -> float:
        cube_position = self.data.xpos[self.cube_body_id].copy()
        target_position = np.asarray(target_position, dtype=float)
        return float(np.linalg.norm(cube_position[:2] - target_position[:2]))

    def _cube_touching_table(self) -> bool:
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            if {geom1, geom2} == {"grasp_cube_geom", "table_geom"}:
                return True
        return False

    def _interpolate_release_opening(self, step_index: int, total_steps: int) -> float:
        if total_steps <= 1:
            return float(GRIPPER_OPENING_OPEN)
        alpha = float(step_index) / float(total_steps - 1)
        return float(GRIPPER_OPENING_CLOSE + alpha * (GRIPPER_OPENING_OPEN - GRIPPER_OPENING_CLOSE))

    def _interpolate_stage_target(
        self,
        start_position: np.ndarray,
        end_position: np.ndarray,
        step_index: int,
        total_steps: int,
    ) -> np.ndarray:
        if total_steps <= 1:
            return np.asarray(end_position, dtype=float).copy()
        alpha = float(step_index + 1) / float(total_steps)
        start_position = np.asarray(start_position, dtype=float)
        end_position = np.asarray(end_position, dtype=float)
        return start_position + alpha * (end_position - start_position)

    def _compute_contact_descent_target(
        self,
        start_position: np.ndarray,
        end_position: np.ndarray,
        *,
        step_index: int,
        total_steps: int,
        cube_position: np.ndarray,
        freeze_z: bool,
        frozen_z: float | None,
    ) -> np.ndarray:
        target = self._interpolate_stage_target(start_position, end_position, step_index, total_steps)
        cube_position = np.asarray(cube_position, dtype=float)
        xy_error = np.asarray(end_position, dtype=float)[:2] - cube_position[:2]
        xy_correction = np.clip(
            self.PLACE_CONTACT_XY_GAIN * xy_error,
            -self.PLACE_CONTACT_XY_MAX_CORRECTION,
            self.PLACE_CONTACT_XY_MAX_CORRECTION,
        )
        target[:2] += xy_correction
        if freeze_z and frozen_z is not None:
            target[2] = float(frozen_z)
        return target

    def _run_pose_transport_stage(self, step: dict[str, np.ndarray | float | str]) -> tuple[TrajectoryFrame, float]:
        frame: TrajectoryFrame | None = None
        controller_steps = int(step["controller_steps"])
        start_position = self.data.site_xpos[self.grasp_site_id].copy()
        end_position = np.asarray(step["target_position"], dtype=float)
        for step_index in range(controller_steps):
            interpolated_target = self._interpolate_stage_target(
                start_position,
                end_position,
                step_index,
                controller_steps,
            )
            frame = self._step_with_target_mode(
                interpolated_target,
                gripper_opening=float(step["gripper_opening"]),
                controller_steps=1,
                control_mode=str(step["control_mode"]),
                target_rotation=self.transport_target_rotation,
            )
        assert frame is not None
        distance = self._distance_to_target(step["target_position"])
        return frame, distance

    def _run_pose_joint_target_stage(
        self,
        step: dict[str, np.ndarray | float | str],
        target_rotation: np.ndarray,
        instant_set: bool = False,
        freeze_arm: bool = False,
    ) -> tuple[TrajectoryFrame, float]:
        q_target = self._plan_joint_target(
            np.asarray(step["target_position"], dtype=float),
            target_rotation=target_rotation,
        )
        if q_target is not None and instant_set:
            self.current_target_position = np.asarray(step["target_position"], dtype=float).copy()
            self._set_gripper(float(step["gripper_opening"]))
            self.controller.set_arm_joint_positions(q_target)
            mujoco.mj_forward(self.model, self.data)
            frame = self._build_frame()
            if self.recorder is not None:
                self.recorder.write(frame)
        else:
            if q_target is not None and freeze_arm:
                self.current_target_position = np.asarray(step["target_position"], dtype=float).copy()
                frame: TrajectoryFrame | None = None
                for _ in range(int(step["controller_steps"])):
                    self._set_gripper(float(step["gripper_opening"]))
                    self.controller.set_arm_joint_positions(q_target)
                    mujoco.mj_step(self.model, self.data)
                    frame = self._build_frame()
                    if self.recorder is not None:
                        self.recorder.write(frame)
                assert frame is not None
            else:
                frame = self._step_with_target_mode(
                    step["target_position"],
                    gripper_opening=float(step["gripper_opening"]),
                    controller_steps=int(step["controller_steps"]),
                    control_mode=str(step["control_mode"]),
                    joint_target=q_target,
                    target_rotation=target_rotation,
                )
        distance = self._distance_to_target(step["target_position"])
        return frame, distance

    def _run_contact_descent_stage(
        self,
        step: dict[str, np.ndarray | float | str],
    ) -> tuple[TrajectoryFrame, float, bool, bool]:
        frame: TrajectoryFrame | None = None
        contact_detected = False
        alignment_reached = False
        contact_hold_z: float | None = None
        controller_steps = int(step["controller_steps"])
        start_position = self.data.site_xpos[self.grasp_site_id].copy()
        end_position = np.asarray(step["target_position"], dtype=float)
        for step_index in range(controller_steps):
            interpolated_target = self._compute_contact_descent_target(
                start_position,
                end_position,
                step_index=step_index,
                total_steps=controller_steps,
                cube_position=self.data.xpos[self.cube_body_id].copy(),
                freeze_z=contact_detected,
                frozen_z=contact_hold_z,
            )
            frame = self._step_with_target_mode(
                interpolated_target,
                gripper_opening=float(step["gripper_opening"]),
                controller_steps=1,
                control_mode=str(step["control_mode"]),
                target_rotation=self.transport_target_rotation,
            )
            if self._cube_touching_table():
                contact_detected = True
                if contact_hold_z is None:
                    contact_hold_z = float(self.data.site_xpos[self.grasp_site_id][2])
                alignment_reached = self._cube_xy_distance_to_target(step["target_position"]) <= self.PLACE_ALIGNMENT_TOLERANCE
                if alignment_reached:
                    break
        assert frame is not None
        distance = self._distance_to_target(step["target_position"])
        return frame, distance, contact_detected, alignment_reached

    def _run_release_stage(self, step: dict[str, np.ndarray | float | str]) -> tuple[TrajectoryFrame, float]:
        frame: TrajectoryFrame | None = None
        controller_steps = int(step["controller_steps"])
        for step_index in range(controller_steps):
            opening = self._interpolate_release_opening(step_index, controller_steps)
            frame = self._step_with_target_mode(
                step["target_position"],
                gripper_opening=opening,
                controller_steps=1,
                control_mode=str(step["control_mode"]),
                target_rotation=self.transport_target_rotation,
            )
        assert frame is not None
        distance = self._distance_to_target(step["target_position"])
        return frame, distance

    def _execute_stage(self, step: dict[str, np.ndarray | float | str]) -> tuple[TrajectoryFrame, float, bool]:
        self.current_step_name = str(step["name"])
        if self.current_step_name == "descend_place_contact":
            return self._run_contact_descent_stage(step)
        if self.current_step_name == "release_place":
            frame, distance = self._run_release_stage(step)
            return frame, distance, True, True
        if self.current_step_name in {
            "lift_clearance",
            "translate_outboard",
            "translate_lateral_place",
            "translate_inboard_place",
            "place_hover",
            "retreat",
        }:
            frame, distance = self._run_pose_transport_stage(step)
            return frame, distance, True, True
        frame = self._step_with_target_mode(
            step["target_position"],
            gripper_opening=float(step["gripper_opening"]),
            controller_steps=int(step["controller_steps"]),
            control_mode=str(step["control_mode"]),
        )
        distance = self._distance_to_target(step["target_position"])
        return frame, distance, True, True

    def _prepare_interactive_stage(self) -> None:
        if self.interactive_plan is None:
            return
        step = self.interactive_plan[self.phase_index]
        self.current_step_name = str(step["name"])
        self.current_target_position = np.asarray(step["target_position"], dtype=float).copy()
        self.active_stage_start_position = self.data.site_xpos[self.grasp_site_id].copy()
        self.active_stage_joint_target = None
        self.active_contact_descent_hold_z = None
        if str(step["control_mode"]) == "position" and float(step["gripper_opening"]) > 0.02:
            current_rotation = self.data.site_xmat[self.grasp_site_id].reshape(3, 3).copy()
            self.active_stage_joint_target = self._plan_joint_target(
                step["target_position"],
                target_rotation=current_rotation,
            )

    def _reset_interactive_episode(self) -> None:
        self.phase_index = 0
        self.phase_tick = 0
        self.failure_reason = None
        self.transport_target_rotation = self.grasp_target_rotation.copy()
        self.interactive_plan = self._build_episode_plan()
        self._prepare_interactive_stage()

    def _interactive_tick(self) -> None:
        if self.interactive_plan is None:
            self._reset_interactive_episode()
        if self.interactive_plan is None or self.failure_reason is not None:
            return

        step = self.interactive_plan[self.phase_index]
        if str(step["name"]) == "release_place":
            opening = self._interpolate_release_opening(self.phase_tick, int(step["controller_steps"]))
            self._step_with_target_mode(
                step["target_position"],
                gripper_opening=opening,
                controller_steps=1,
                control_mode=str(step["control_mode"]),
                joint_target=self.active_stage_joint_target,
                target_rotation=self.transport_target_rotation,
            )
        elif str(step["name"]) in {
            "lift_clearance",
            "translate_outboard",
            "translate_lateral_place",
            "translate_inboard_place",
            "place_hover",
            "descend_place_contact",
            "retreat",
        }:
            if str(step["name"]) == "descend_place_contact":
                interpolated_target = self._compute_contact_descent_target(
                    self.active_stage_start_position,
                    np.asarray(step["target_position"], dtype=float),
                    step_index=self.phase_tick,
                    total_steps=int(step["controller_steps"]),
                    cube_position=self.data.xpos[self.cube_body_id].copy(),
                    freeze_z=self.active_contact_descent_hold_z is not None,
                    frozen_z=self.active_contact_descent_hold_z,
                )
            else:
                interpolated_target = self._interpolate_stage_target(
                    self.active_stage_start_position,
                    np.asarray(step["target_position"], dtype=float),
                    self.phase_tick,
                    int(step["controller_steps"]),
                )
            self._step_with_target_mode(
                interpolated_target,
                gripper_opening=float(step["gripper_opening"]),
                controller_steps=1,
                control_mode=str(step["control_mode"]),
                joint_target=self.active_stage_joint_target,
                target_rotation=self.transport_target_rotation,
            )
        else:
            self._step_with_target_mode(
                step["target_position"],
                gripper_opening=float(step["gripper_opening"]),
                controller_steps=1,
                control_mode=str(step["control_mode"]),
                joint_target=self.active_stage_joint_target,
            )

        if str(step["name"]) == "descend_place_contact":
            if self._cube_touching_table():
                if self.active_contact_descent_hold_z is None:
                    self.active_contact_descent_hold_z = float(self.data.site_xpos[self.grasp_site_id][2])
                if self._cube_xy_distance_to_target(step["target_position"]) <= self.PLACE_ALIGNMENT_TOLERANCE:
                    self.phase_tick = 0
                    if self.phase_index < len(self.interactive_plan) - 1:
                        self.phase_index += 1
                        self._prepare_interactive_stage()
                    return
            self.phase_tick += 1
            if self.phase_tick >= int(step["controller_steps"]):
                if self._cube_touching_table():
                    self.failure_reason = "place_alignment_not_reached"
                else:
                    self.failure_reason = "place_contact_not_reached"
            return

        self.phase_tick += 1
        if self.phase_tick < max(self.phase_duration, int(step["controller_steps"])):
            return

        self.phase_tick = 0
        distance = self._distance_to_target(step["target_position"])
        tolerance = float(step["position_tolerance"])
        if is_blocking_position_stage(str(step["name"])) and distance > tolerance:
            self.failure_reason = f"{step['name']}_target_not_reached"
            return

        failure_reason = pick_place_stage_failure_reason(
            step_name=step["name"],
            grasp_likely_secured=self._grasp_likely_secured(),
            cube_height=self._cube_height(),
            initial_cube_height=self.initial_cube_height,
            table_contact_detected=self._cube_touching_table(),
        )
        if failure_reason is not None:
            self.failure_reason = failure_reason
            return

        if self.phase_index < len(self.interactive_plan) - 1:
            self.phase_index += 1
            self._prepare_interactive_stage()

    def run_scripted_step(self) -> TrajectoryFrame:
        pick_position = self.model.body_pos[self.pick_target_body_id].copy()
        return self._step_with_target(pick_position, gripper_opening=0.02)

    def run_episode(self) -> PickPlaceEpisodeResult:
        frames: list[TrajectoryFrame] = []
<<<<<<< HEAD

        pick_position = self.model.body_pos[self.pick_target_body_id].copy()
        cube_position = self.data.xpos[self.cube_body_id].copy()
        place_position = self.data.mocap_pos[self.place_target_mocap_id].copy()

        for step in build_pick_place_plan(pick_position, cube_position, place_position):
            frames.append(
                self._step_with_target(
                    step["target_position"],
                    gripper_opening=step["gripper_opening"],
                )
            )
            if step["name"] == "close_gripper":
                self._try_attach_cube()
            if step["name"] == "open_gripper":
                self.cube_attached = False
=======
        place_position = self.data.mocap_pos[self.place_target_mocap_id].copy()

        for step in self._build_episode_plan():
            frame, distance, table_contact_detected, place_alignment_reached = self._execute_stage(step)
            frames.append(frame)
            tolerance = float(step["position_tolerance"])
            if is_blocking_position_stage(str(step["name"])) and distance > tolerance:
                return PickPlaceEpisodeResult(
                    frames=frames,
                    grasp_success=False,
                    episode_success=False,
                    failure_reason=f"{step['name']}_target_not_reached",
                )
            failure_reason = pick_place_stage_failure_reason(
                step_name=step["name"],
                grasp_likely_secured=self._grasp_likely_secured(),
                cube_height=self._cube_height(),
                initial_cube_height=self.initial_cube_height,
                table_contact_detected=table_contact_detected,
                place_alignment_reached=place_alignment_reached,
            )
            if failure_reason is not None:
                return PickPlaceEpisodeResult(
                    frames=frames,
                    grasp_success=False,
                    episode_success=False,
                    failure_reason=failure_reason,
                )
>>>>>>> 1f097dd (Update .gitignore and restage clean files)

        return evaluate_pick_place_episode(frames, self.initial_cube_height, place_position)

    def run_interactive(self, max_steps: int | None = None) -> None:
        import mujoco.viewer

        self._reset_interactive_episode()
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            step_count = 0
            reported_failure: str | None = None
            while viewer.is_running():
<<<<<<< HEAD
                plan = build_pick_place_plan(
                    self.model.body_pos[self.pick_target_body_id].copy(),
                    self.data.xpos[self.cube_body_id].copy(),
                    self.data.mocap_pos[self.place_target_mocap_id].copy(),
                )
                step = plan[self.phase_index]
                self._step_with_target(step["target_position"], gripper_opening=step["gripper_opening"], controller_steps=1)

                if step["name"] == "close_gripper" and self.phase_tick == self.phase_duration // 2:
                    self._try_attach_cube()
                if step["name"] == "open_gripper" and self.phase_tick == 0:
                    self.cube_attached = False

                self.phase_tick += 1
                if self.phase_tick >= self.phase_duration:
                    self.phase_tick = 0
                    if self.phase_index < len(plan) - 1:
                        self.phase_index += 1
=======
                if self.failure_reason is None:
                    self._interactive_tick()
                elif self.failure_reason != reported_failure:
                    print(f"Pick-place episode stopped: {self.failure_reason}")
                    reported_failure = self.failure_reason
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
                viewer.sync()
                time.sleep(self.model.opt.timestep)
                step_count += 1
                if max_steps is not None and step_count >= max_steps:
                    break


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the my6dof pick-and-place task.")
    parser.add_argument("--headless", action="store_true", help="Run the scripted episode headlessly.")
    parser.add_argument("--record-path", default=None, help="Optional JSONL output path.")
    parser.add_argument("--max-steps", type=int, default=None, help="Optional limit for interactive viewer steps.")
    args = parser.parse_args()

    task = PickPlaceTask(interactive=not args.headless, record_path=args.record_path)
    if args.headless:
        result = task.run_episode()
        print(result.to_dict())
        if task.recorder is not None:
            task.recorder.close()
        return

    task.run_interactive(max_steps=args.max_steps)
    if task.recorder is not None:
        task.recorder.close()


if __name__ == "__main__":
    main()