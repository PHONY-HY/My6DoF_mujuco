from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco
import numpy as np

from mujoco_control import DifferentialIKController
from tasks.recording import TrajectoryFrame, TrajectoryRecorder
from ur5_style_arm.robot_model import NEUTRAL_Q


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_pick_place.xml"


class PickPlaceTask:
    def __init__(self, interactive: bool = True, record_path: str | None = None) -> None:
        self.interactive = interactive
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.controller = DifferentialIKController(self.model, self.data, "ee_site")

        self.place_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "place_target")
        self.place_target_mocap_id = int(self.model.body_mocapid[self.place_target_body_id])
        self.pick_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pick_target")
        self.cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "grasp_cube")
        self.cube_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "grasp_cube_freejoint")
        self.cube_qpos_adr = self.model.jnt_qposadr[self.cube_joint_id]
        self.cube_qvel_adr = self.model.jnt_dofadr[self.cube_joint_id]
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self.recorder = TrajectoryRecorder(record_path) if record_path is not None else None

        self.cube_attached = False
        self.attach_offset = np.array([0.0, 0.0, -0.05], dtype=float)
        self._initialize_home_pose()

    def _initialize_home_pose(self) -> None:
        self.controller.set_arm_joint_positions(NEUTRAL_Q)
        self.controller.set_gripper_opening(0.04)
        self.data.mocap_quat[self.place_target_mocap_id] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        mujoco.mj_forward(self.model, self.data)

    def _set_cube_pose(self, position: np.ndarray, quaternion: np.ndarray | None = None) -> None:
        quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float) if quaternion is None else np.asarray(quaternion, dtype=float)
        self.data.qpos[self.cube_qpos_adr : self.cube_qpos_adr + 3] = np.asarray(position, dtype=float)
        self.data.qpos[self.cube_qpos_adr + 3 : self.cube_qpos_adr + 7] = quaternion
        self.data.qvel[self.cube_qvel_adr : self.cube_qvel_adr + 6] = 0.0

    def _cube_pose(self) -> dict[str, list[float]]:
        return {
            "position": self.data.xpos[self.cube_body_id].copy().tolist(),
            "quaternion": self.data.xquat[self.cube_body_id].copy().tolist(),
        }

    def _target_pose(self) -> dict[str, list[float]]:
        return {
            "position": self.data.mocap_pos[self.place_target_mocap_id].copy().tolist(),
            "quaternion": self.data.mocap_quat[self.place_target_mocap_id].copy().tolist(),
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

    def _step_with_target(self, target_position: np.ndarray, gripper_opening: float, controller_steps: int = 10) -> TrajectoryFrame:
        target_position = np.asarray(target_position, dtype=float)
        self._set_gripper(gripper_opening)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(controller_steps):
            self.controller.step_to_position(target_position)
            if self.cube_attached:
                ee_position = self.data.site_xpos[self.ee_site_id].copy()
                self._set_cube_pose(ee_position + self.attach_offset)
            mujoco.mj_step(self.model, self.data)

        frame = self._build_frame()
        if self.recorder is not None:
            self.recorder.write(frame)
        return frame

    def _try_attach_cube(self) -> None:
        ee_position = self.data.site_xpos[self.ee_site_id].copy()
        cube_position = self.data.xpos[self.cube_body_id].copy()
        if np.linalg.norm(ee_position - cube_position) < 0.08:
            self.cube_attached = True
            self.attach_offset = cube_position - ee_position

    def run_scripted_step(self) -> TrajectoryFrame:
        pick_position = self.model.body_pos[self.pick_target_body_id].copy()
        return self._step_with_target(pick_position, gripper_opening=0.02)

    def run_episode(self) -> list[TrajectoryFrame]:
        frames: list[TrajectoryFrame] = []

        pick_position = self.model.body_pos[self.pick_target_body_id].copy()
        cube_position = self.data.xpos[self.cube_body_id].copy()
        hover_position = pick_position + np.array([0.0, 0.0, 0.08], dtype=float)
        grasp_position = cube_position + np.array([0.0, 0.0, 0.05], dtype=float)
        lift_position = cube_position + np.array([0.0, 0.0, 0.15], dtype=float)
        place_position = self.data.mocap_pos[self.place_target_mocap_id].copy()
        place_hover = place_position + np.array([0.0, 0.0, 0.08], dtype=float)

        frames.append(self._step_with_target(hover_position, gripper_opening=0.04))
        frames.append(self._step_with_target(grasp_position, gripper_opening=0.04))
        frames.append(self._step_with_target(grasp_position, gripper_opening=0.0))
        self._try_attach_cube()
        frames.append(self._step_with_target(lift_position, gripper_opening=0.0))
        frames.append(self._step_with_target(place_hover, gripper_opening=0.0))
        frames.append(self._step_with_target(place_position, gripper_opening=0.0))
        frames.append(self._step_with_target(place_position, gripper_opening=0.04))
        self.cube_attached = False
        frames.append(self._step_with_target(place_hover, gripper_opening=0.04))

        return frames

    def run_interactive(self, max_steps: int | None = None) -> None:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            step_count = 0
            while viewer.is_running():
                place_target = self.data.mocap_pos[self.place_target_mocap_id].copy()
                self._step_with_target(place_target, gripper_opening=0.02, controller_steps=1)
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
        frames = task.run_episode()
        print(frames[-1].to_dict())
        if task.recorder is not None:
            task.recorder.close()
        return

    task.run_interactive(max_steps=args.max_steps)
    if task.recorder is not None:
        task.recorder.close()


if __name__ == "__main__":
    main()