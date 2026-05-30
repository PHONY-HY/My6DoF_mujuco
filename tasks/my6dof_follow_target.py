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
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_follow_target.xml"


class FollowTargetTask:
    def __init__(self, interactive: bool = True, record_path: str | None = None) -> None:
        self.interactive = interactive
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.controller = DifferentialIKController(self.model, self.data, "ee_site")
        self.target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "follow_target")
        self.target_mocap_id = int(self.model.body_mocapid[self.target_body_id])
        self.recorder = TrajectoryRecorder(record_path) if record_path is not None else None
        self._initialize_home_pose()

    def _initialize_home_pose(self) -> None:
        self.controller.set_arm_joint_positions(NEUTRAL_Q)
        self.controller.set_gripper_opening(0.02)
        self.data.mocap_quat[self.target_mocap_id] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        mujoco.mj_forward(self.model, self.data)

    def _target_pose(self) -> dict[str, list[float]]:
        return {
            "position": self.data.mocap_pos[self.target_mocap_id].copy().tolist(),
            "quaternion": self.data.mocap_quat[self.target_mocap_id].copy().tolist(),
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
            object_pose=None,
        )

    def step_to_target(self, target_position: np.ndarray, controller_steps: int = 10) -> TrajectoryFrame:
        target_position = np.asarray(target_position, dtype=float)
        self.data.mocap_pos[self.target_mocap_id] = target_position
        mujoco.mj_forward(self.model, self.data)

        for _ in range(controller_steps):
            self.controller.step_to_position(target_position)
            mujoco.mj_step(self.model, self.data)

        frame = self._build_frame()
        if self.recorder is not None:
            self.recorder.write(frame)
        return frame

    def run_interactive(self, max_steps: int | None = None) -> None:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            step_count = 0
            while viewer.is_running():
                target_position = self.data.mocap_pos[self.target_mocap_id].copy()
                self.controller.step_to_position(target_position)
                mujoco.mj_step(self.model, self.data)
                if self.recorder is not None:
                    self.recorder.write(self._build_frame())
                viewer.sync()
                time.sleep(self.model.opt.timestep)
                step_count += 1
                if max_steps is not None and step_count >= max_steps:
                    break


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the my6dof follow-target task.")
    parser.add_argument("--headless", action="store_true", help="Run one headless target step and print the recorded frame.")
    parser.add_argument("--record-path", default=None, help="Optional JSONL output path.")
    parser.add_argument("--target", nargs=3, type=float, default=[0.42, 0.05, 0.42], help="Headless target position.")
    parser.add_argument("--max-steps", type=int, default=None, help="Optional limit for interactive viewer steps.")
    args = parser.parse_args()

    task = FollowTargetTask(interactive=not args.headless, record_path=args.record_path)
    if args.headless:
        frame = task.step_to_target(np.array(args.target, dtype=float))
        print(frame.to_dict())
        if task.recorder is not None:
            task.recorder.close()
        return

    task.run_interactive(max_steps=args.max_steps)
    if task.recorder is not None:
        task.recorder.close()


if __name__ == "__main__":
    main()