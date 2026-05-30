# My6DOF MuJoCo Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a local `wxai`-style MuJoCo compatibility layer for the existing Pinocchio 6DOF arm, including a parallel `my6dof` robot family, pick-and-place and follow-target tasks, lightweight trajectory recording, and Pinocchio-to-MuJoCo verification.

**Architecture:** Keep the existing Pinocchio package in `src/ur5_style_arm` as the offline kinematics and verification layer. Add a separate MuJoCo asset tree, a lightweight controller package, and two task entrypoints that mirror `wxai` ideas without modifying upstream repositories. Use headless smoke tests for loading and stepping tasks, then keep interactive viewer support in the task scripts for human-driven runs.

**Tech Stack:** Python 3.10+, MuJoCo 3.2.3, dm_control, dm_env, NumPy, Pinocchio, pytest, JSONL recording, editable setuptools install

---

## Preconditions

- The implementation target is the current workspace root: `E:\Job\Project_Learning\example`
- The authoritative integration runtime is the existing Linux virtual environment:
  - `trossen_mujoco_env/`
- Environment details already confirmed by the user:
  - Python `3.10.12`
  - MuJoCo `3.2.3`
  - `dm_control`
  - `dm_env`
  - `h5py`
  - `matplotlib`
  - `numpy`
  - `opencv-python`
  - `pyquaternion`
  - `trossen_arm`
- Use that environment for MuJoCo-side execution and integration testing instead of the local Windows-only fallback environment.

Run these commands before Task 1:

```bash
source trossen_mujoco_env/bin/activate
python -m pip install -e .[dev]
python -c "import mujoco, dm_control, dm_env, pinocchio, trossen_arm; print('runtime-ok')"
```

Expected:

- editable install completes successfully
- the import command prints `runtime-ok`

## Planned File Structure

- Modify: `pyproject.toml`
- Create: `mujoco_assets/my6dof/my6dof_base.xml`
- Create: `mujoco_assets/my6dof/scene_my6dof_follow_target.xml`
- Create: `mujoco_assets/my6dof/scene_my6dof_pick_place.xml`
- Create: `mujoco_control/__init__.py`
- Create: `mujoco_control/controller.py`
- Create: `tasks/__init__.py`
- Create: `tasks/recording.py`
- Create: `tasks/my6dof_follow_target.py`
- Create: `tasks/my6dof_pick_place.py`
- Create: `src/ur5_style_arm/mujoco_verify.py`
- Create: `docs/my6dof_mujoco_usage.md`
- Create: `tests/test_mujoco_import.py`
- Create: `tests/test_mujoco_assets.py`
- Create: `tests/test_recording.py`
- Create: `tests/test_follow_target_smoke.py`
- Create: `tests/test_pick_place_smoke.py`
- Create: `tests/test_mujoco_pinocchio_consistency.py`

## Interface Decisions Locked In By This Plan

- The parallel robot family name is `my6dof`
- MuJoCo asset root is `mujoco_assets/my6dof/`
- The end-effector site name is `ee_site`
- The follow-target scene entrypoint is `tasks/my6dof_follow_target.py`
- The pick-and-place scene entrypoint is `tasks/my6dof_pick_place.py`
- The online controller class name is `DifferentialIKController`
- Trajectory data is written as JSONL
- Each recorded frame includes:
  - `timestamp`
  - `joint_positions`
  - `gripper_state`
  - `target_pose`
  - `object_pose`
- Pinocchio remains in `src/ur5_style_arm/` and is not removed or replaced

### Task 1: Bootstrap MuJoCo Scaffolding

**Files:**
- Modify: `pyproject.toml`
- Create: `mujoco_control/__init__.py`
- Create: `tasks/__init__.py`
- Create: `tests/test_mujoco_import.py`

- [ ] **Step 1: Write the failing import smoke test**

Create `tests/test_mujoco_import.py`:

```python
import importlib


def test_mujoco_stack_imports():
    mujoco = importlib.import_module("mujoco")
    controller_module = importlib.import_module("mujoco_control")
    tasks_module = importlib.import_module("tasks")

    assert mujoco.__name__ == "mujoco"
    assert controller_module.__name__ == "mujoco_control"
    assert tasks_module.__name__ == "tasks"
```

- [ ] **Step 2: Run the import test and verify it fails for missing local modules**

Run:

```bash
python -m pytest tests/test_mujoco_import.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'mujoco_control'` or `No module named 'tasks'`

- [ ] **Step 3: Add MuJoCo dependency metadata and minimal package markers**

Replace `pyproject.toml` with:

```toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "ur5-style-arm"
version = "0.1.0"
description = "Pinocchio-based UR5-style 6DOF arm kinematics toolkit"
requires-python = ">=3.10"
dependencies = [
  "mujoco>=3.2",
  "numpy>=1.26",
]

[project.optional-dependencies]
dev = [
  "pytest>=8.0",
]

[tool.setuptools]
package-dir = {"" = "src"}

[tool.setuptools.packages.find]
where = ["src"]
```

Create `mujoco_control/__init__.py`:

```python
"""MuJoCo controller package for the my6dof robot family."""
```

Create `tasks/__init__.py`:

```python
"""Task entrypoints and helpers for the my6dof MuJoCo migration."""
```

- [ ] **Step 4: Re-run the import test**

Run:

```bash
python -m pytest tests/test_mujoco_import.py -q
```

Expected: `1 passed`

- [ ] **Step 5: Commit the MuJoCo bootstrap**

Run:

```bash
git add pyproject.toml mujoco_control/__init__.py tasks/__init__.py tests/test_mujoco_import.py
git commit -m "chore: bootstrap mujoco migration scaffolding"
```

Expected: one commit created for the MuJoCo bootstrap

### Task 2: Build The MuJoCo Robot And Scene XML Assets

**Files:**
- Create: `mujoco_assets/my6dof/my6dof_base.xml`
- Create: `mujoco_assets/my6dof/scene_my6dof_follow_target.xml`
- Create: `mujoco_assets/my6dof/scene_my6dof_pick_place.xml`
- Create: `tests/test_mujoco_assets.py`

- [ ] **Step 1: Write failing tests that load the MuJoCo assets**

Create `tests/test_mujoco_assets.py`:

```python
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = ROOT / "mujoco_assets" / "my6dof"


def test_my6dof_base_model_loads_with_expected_names():
    model = mujoco.MjModel.from_xml_path(str(ASSET_ROOT / "my6dof_base.xml"))

    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(model.njnt)
    ]
    actuator_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
    ]

    assert joint_names[:6] == [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]
    assert "ee_site" in [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        for i in range(model.nsite)
    ]
    assert actuator_names == [
        "joint_1_ctrl",
        "joint_2_ctrl",
        "joint_3_ctrl",
        "joint_4_ctrl",
        "joint_5_ctrl",
        "joint_6_ctrl",
        "left_finger_ctrl",
        "right_finger_ctrl",
    ]


def test_follow_and_pick_place_scenes_load():
    follow_model = mujoco.MjModel.from_xml_path(str(ASSET_ROOT / "scene_my6dof_follow_target.xml"))
    pick_model = mujoco.MjModel.from_xml_path(str(ASSET_ROOT / "scene_my6dof_pick_place.xml"))

    assert follow_model.nq >= 8
    assert pick_model.nq >= 8
    assert mujoco.mj_name2id(follow_model, mujoco.mjtObj.mjOBJ_BODY, "follow_target") >= 0
    assert mujoco.mj_name2id(pick_model, mujoco.mjtObj.mjOBJ_BODY, "grasp_cube") >= 0
```

- [ ] **Step 2: Run the asset tests and verify they fail**

Run:

```bash
python -m pytest tests/test_mujoco_assets.py -q
```

Expected: FAIL with file-not-found errors for the XML assets

- [ ] **Step 3: Create the base robot XML**

Create `mujoco_assets/my6dof/my6dof_base.xml`:

```xml
<mujoco model="my6dof_base">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -9.81" integrator="implicitfast" timestep="0.002"/>

  <default>
    <joint damping="1.0" armature="0.01" limited="true"/>
    <geom rgba="0.3 0.5 0.8 1"/>
    <position kp="100"/>
  </default>

  <worldbody>
    <body name="robot_base" pos="0 0 0">
      <geom type="cylinder" size="0.05 0.02" rgba="0.2 0.2 0.2 1"/>

      <body name="link1" pos="0 0 0.089159">
        <joint name="joint_1" type="hinge" axis="0 0 1" range="-6.28319 6.28319"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.08" size="0.03"/>

        <body name="link2" pos="0 0 0">
          <joint name="joint_2" type="hinge" axis="0 1 0" range="-3.14159 0"/>
          <geom type="capsule" fromto="0 0 0 0.425 0 0" size="0.025"/>

          <body name="link3" pos="0.425 0 0">
            <joint name="joint_3" type="hinge" axis="0 1 0" range="-3.14159 3.14159"/>
            <geom type="capsule" fromto="0 0 0 0.39225 0 0" size="0.022"/>

            <body name="link4" pos="0.39225 0 0">
              <joint name="joint_4" type="hinge" axis="0 1 0" range="-6.28319 6.28319"/>
              <geom type="capsule" fromto="0 0 0 0 0 0.10915" size="0.02"/>

              <body name="link5" pos="0 0 0.10915">
                <joint name="joint_5" type="hinge" axis="0 0 1" range="-6.28319 6.28319"/>
                <geom type="capsule" fromto="0 0 0 0 0 0.09465" size="0.018"/>

                <body name="link6" pos="0 0 0.09465">
                  <joint name="joint_6" type="hinge" axis="0 1 0" range="-6.28319 6.28319"/>
                  <geom type="capsule" fromto="0 0 0 0 0 0.0823" size="0.016"/>

                  <body name="gripper_mount" pos="0 0 0.0823">
                    <site name="ee_site" pos="0 0 0" size="0.01" rgba="0 1 0 1"/>

                    <body name="left_finger" pos="0 0.02 0">
                      <joint name="left_finger_joint" type="slide" axis="0 1 0" range="0 0.04"/>
                      <geom type="box" size="0.01 0.02 0.01" pos="0 0.02 0" rgba="0.7 0.7 0.7 1"/>
                    </body>

                    <body name="right_finger" pos="0 -0.02 0">
                      <joint name="right_finger_joint" type="slide" axis="0 -1 0" range="0 0.04"/>
                      <geom type="box" size="0.01 0.02 0.01" pos="0 -0.02 0" rgba="0.7 0.7 0.7 1"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="joint_1_ctrl" joint="joint_1" ctrlrange="-6.28319 6.28319"/>
    <position name="joint_2_ctrl" joint="joint_2" ctrlrange="-3.14159 0"/>
    <position name="joint_3_ctrl" joint="joint_3" ctrlrange="-3.14159 3.14159"/>
    <position name="joint_4_ctrl" joint="joint_4" ctrlrange="-6.28319 6.28319"/>
    <position name="joint_5_ctrl" joint="joint_5" ctrlrange="-6.28319 6.28319"/>
    <position name="joint_6_ctrl" joint="joint_6" ctrlrange="-6.28319 6.28319"/>
    <position name="left_finger_ctrl" joint="left_finger_joint" ctrlrange="0 0.04"/>
    <position name="right_finger_ctrl" joint="right_finger_joint" ctrlrange="0 0.04"/>
  </actuator>
</mujoco>
```

- [ ] **Step 4: Create the follow-target and pick-place scenes**

Create `mujoco_assets/my6dof/scene_my6dof_follow_target.xml`:

```xml
<mujoco model="scene_my6dof_follow_target">
  <include file="my6dof_base.xml"/>

  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8"/>
  </visual>

  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="follow_target" pos="0.45 0.0 0.45">
      <geom type="sphere" size="0.03" rgba="1 0 0 1"/>
      <site name="follow_target_site" pos="0 0 0" size="0.01" rgba="1 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
```

Create `mujoco_assets/my6dof/scene_my6dof_pick_place.xml`:

```xml
<mujoco model="scene_my6dof_pick_place">
  <include file="my6dof_base.xml"/>

  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="table" pos="0.45 0 0.2">
      <geom type="box" size="0.35 0.35 0.02" rgba="0.55 0.43 0.3 1"/>
    </body>
    <body name="grasp_cube" pos="0.35 0 0.25">
      <freejoint/>
      <geom type="box" size="0.02 0.02 0.02" rgba="0.2 0.8 0.2 1"/>
    </body>
    <body name="pick_target" pos="0.35 0 0.32">
      <geom type="sphere" size="0.015" rgba="1 0.5 0 1"/>
      <site name="pick_target_site" pos="0 0 0" size="0.01" rgba="1 1 0 1"/>
    </body>
    <body name="place_target" pos="0.55 0.1 0.25">
      <geom type="cylinder" size="0.03 0.002" rgba="0 0 1 0.5"/>
      <site name="place_target_site" pos="0 0 0" size="0.01" rgba="0 1 1 1"/>
    </body>
  </worldbody>
</mujoco>
```

- [ ] **Step 5: Run the asset tests**

Run:

```bash
python -m pytest tests/test_mujoco_assets.py -q
```

Expected: `2 passed`

- [ ] **Step 6: Commit the MuJoCo assets**

Run:

```bash
git add mujoco_assets/my6dof/my6dof_base.xml mujoco_assets/my6dof/scene_my6dof_follow_target.xml mujoco_assets/my6dof/scene_my6dof_pick_place.xml tests/test_mujoco_assets.py
git commit -m "feat: add my6dof mujoco assets"
```

Expected: one commit created for the XML assets

### Task 3: Add JSONL Trajectory Recording

**Files:**
- Create: `tasks/recording.py`
- Create: `tests/test_recording.py`

- [ ] **Step 1: Write failing tests for frame serialization and JSONL output**

Create `tests/test_recording.py`:

```python
import json
from pathlib import Path

from tasks.recording import TrajectoryFrame, TrajectoryRecorder


def test_trajectory_frame_serializes_numpy_friendly_payload():
    frame = TrajectoryFrame(
        timestamp=0.1,
        joint_positions=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        gripper_state={"left": 0.01, "right": 0.01},
        target_pose={"position": [0.4, 0.0, 0.4], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        object_pose={"position": [0.3, 0.0, 0.2], "quaternion": [0.0, 0.0, 0.0, 1.0]},
    )

    payload = frame.to_dict()

    assert payload["timestamp"] == 0.1
    assert payload["joint_positions"][-1] == 5.0
    assert payload["gripper_state"]["left"] == 0.01


def test_trajectory_recorder_writes_jsonl(tmp_path: Path):
    output_path = tmp_path / "trajectory.jsonl"
    recorder = TrajectoryRecorder(output_path)
    frame = TrajectoryFrame(
        timestamp=0.2,
        joint_positions=[0.1] * 6,
        gripper_state={"left": 0.02, "right": 0.02},
        target_pose={"position": [0.5, 0.1, 0.4], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        object_pose=None,
    )

    recorder.write(frame)
    recorder.close()

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    data = json.loads(lines[0])
    assert data["object_pose"] is None
    assert data["joint_positions"][0] == 0.1
```

- [ ] **Step 2: Run the recorder tests and verify they fail**

Run:

```bash
python -m pytest tests/test_recording.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tasks.recording'`

- [ ] **Step 3: Implement the recording utility**

Create `tasks/recording.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass
class TrajectoryFrame:
    timestamp: float
    joint_positions: list[float]
    gripper_state: dict[str, float]
    target_pose: dict[str, list[float]]
    object_pose: dict[str, list[float]] | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": float(self.timestamp),
            "joint_positions": [float(value) for value in self.joint_positions],
            "gripper_state": {key: float(value) for key, value in self.gripper_state.items()},
            "target_pose": self.target_pose,
            "object_pose": self.object_pose,
        }


class TrajectoryRecorder:
    def __init__(self, output_path: str | Path) -> None:
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.output_path.open("w", encoding="utf-8")

    def write(self, frame: TrajectoryFrame) -> None:
        self._handle.write(json.dumps(frame.to_dict(), ensure_ascii=True) + "\n")
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()
```

- [ ] **Step 4: Re-run the recorder tests**

Run:

```bash
python -m pytest tests/test_recording.py -q
```

Expected: `2 passed`

- [ ] **Step 5: Commit the recorder**

Run:

```bash
git add tasks/recording.py tests/test_recording.py
git commit -m "feat: add mujoco trajectory recording"
```

Expected: one commit created for recording

### Task 4: Implement The Differential IK Controller And Follow-Target Task

**Files:**
- Create: `mujoco_control/controller.py`
- Create: `tasks/my6dof_follow_target.py`
- Create: `tests/test_follow_target_smoke.py`

- [ ] **Step 1: Write a failing smoke test for the headless follow-target task**

Create `tests/test_follow_target_smoke.py`:

```python
import numpy as np

from tasks.my6dof_follow_target import FollowTargetTask


def test_follow_target_headless_step_returns_record():
    task = FollowTargetTask(interactive=False, record_path=None)
    record = task.step_to_target(np.array([0.42, 0.05, 0.42], dtype=float))

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert record.object_pose is None
    assert abs(record.target_pose["position"][0] - 0.42) < 1e-9
```

- [ ] **Step 2: Run the smoke test and verify it fails**

Run:

```bash
python -m pytest tests/test_follow_target_smoke.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tasks.my6dof_follow_target'`

- [ ] **Step 3: Implement the controller and headless follow-target task**

Create `mujoco_control/controller.py`:

```python
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
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, ee_site_name: str, config: ControllerConfig | None = None) -> None:
        self.model = model
        self.data = data
        self.config = config or ControllerConfig()
        self.ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)
        self.arm_joint_names = [f"joint_{index}" for index in range(1, 7)]
        self.arm_joint_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.arm_joint_names
        ]
        self.arm_actuator_names = [f"joint_{index}_ctrl" for index in range(1, 7)]
        self.arm_actuator_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.arm_actuator_names
        ]

    @staticmethod
    def _left_gram(matrix: np.ndarray) -> np.ndarray:
        return np.einsum("ik,jk->ij", matrix, matrix)

    @staticmethod
    def _matvec(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
        return np.einsum("ij,j->i", matrix, vector)

    @staticmethod
    def _solve_linear_system(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
        augmented = np.hstack((matrix.astype(float).copy(), vector.astype(float).reshape(-1, 1).copy()))
        n = augmented.shape[0]
        for pivot_idx in range(n):
            max_row = pivot_idx + int(np.argmax(np.abs(augmented[pivot_idx:, pivot_idx])))
            if abs(augmented[max_row, pivot_idx]) <= 1e-12:
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
        return np.array([self.data.joint(name).qpos[0] for name in self.arm_joint_names], dtype=float)

    def step_to_target(self, target_position: np.ndarray) -> np.ndarray:
        jacp = np.zeros((3, self.model.nv), dtype=float)
        jacr = np.zeros((3, self.model.nv), dtype=float)
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.ee_site_id)

        qpos = self.current_joint_positions()
        error = np.asarray(target_position, dtype=float) - self.data.site_xpos[self.ee_site_id]
        task_jacobian = jacp[:, :6]
        lhs = self._left_gram(task_jacobian) + self.config.damping * np.eye(3)
        qdot = self._matvec(task_jacobian.T, self._solve_linear_system(lhs, self.config.position_gain * error))
        q_next = qpos + self.config.step_scale * qdot

        for actuator_id, value in zip(self.arm_actuator_ids, q_next):
            self.data.ctrl[actuator_id] = value

        return q_next
```

Create `tasks/my6dof_follow_target.py`:

```python
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from mujoco_control.controller import DifferentialIKController
from tasks.recording import TrajectoryFrame, TrajectoryRecorder


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_follow_target.xml"


class FollowTargetTask:
    def __init__(self, interactive: bool = True, record_path: str | None = None) -> None:
        self.interactive = interactive
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.controller = DifferentialIKController(self.model, self.data, "ee_site")
        self.target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "follow_target")
        self.recorder = TrajectoryRecorder(record_path) if record_path is not None else None

    def _current_gripper_state(self) -> dict[str, float]:
        return {"left": float(self.data.joint("left_finger_joint").qpos[0]), "right": float(self.data.joint("right_finger_joint").qpos[0])}

    def step_to_target(self, target_position: np.ndarray) -> TrajectoryFrame:
        self.model.body_pos[self.target_body_id] = np.asarray(target_position, dtype=float)
        self.controller.step_to_target(target_position)
        mujoco.mj_step(self.model, self.data)
        frame = TrajectoryFrame(
            timestamp=float(self.data.time),
            joint_positions=self.controller.current_joint_positions().tolist(),
            gripper_state=self._current_gripper_state(),
            target_pose={"position": np.asarray(target_position, dtype=float).tolist(), "quaternion": [0.0, 0.0, 0.0, 1.0]},
            object_pose=None,
        )
        if self.recorder is not None:
            self.recorder.write(frame)
        return frame
```

- [ ] **Step 4: Run the follow-target smoke test**

Run:

```bash
python -m pytest tests/test_follow_target_smoke.py -q
```

Expected: `1 passed`

- [ ] **Step 5: Commit the controller and follow-target task**

Run:

```bash
git add mujoco_control/controller.py tasks/my6dof_follow_target.py tests/test_follow_target_smoke.py
git commit -m "feat: add my6dof follow target task"
```

Expected: one commit created for the controller and follow-target task

### Task 5: Implement The Pick-And-Place Task

**Files:**
- Create: `tasks/my6dof_pick_place.py`
- Create: `tests/test_pick_place_smoke.py`

- [ ] **Step 1: Write a failing headless smoke test for pick and place**

Create `tests/test_pick_place_smoke.py`:

```python
from tasks.my6dof_pick_place import PickPlaceTask


def test_pick_place_headless_episode_returns_record_with_object_pose():
    task = PickPlaceTask(interactive=False, record_path=None)
    record = task.run_scripted_step()

    assert len(record.joint_positions) == 6
    assert "position" in record.target_pose
    assert "position" in record.object_pose
    assert set(record.gripper_state.keys()) == {"left", "right"}
```

- [ ] **Step 2: Run the smoke test and verify it fails**

Run:

```bash
python -m pytest tests/test_pick_place_smoke.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'tasks.my6dof_pick_place'`

- [ ] **Step 3: Implement the pick-and-place task**

Create `tasks/my6dof_pick_place.py`:

```python
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from mujoco_control.controller import DifferentialIKController
from tasks.recording import TrajectoryFrame, TrajectoryRecorder


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_pick_place.xml"


class PickPlaceTask:
    def __init__(self, interactive: bool = True, record_path: str | None = None) -> None:
        self.interactive = interactive
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        self.controller = DifferentialIKController(self.model, self.data, "ee_site")
        self.pick_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pick_target")
        self.place_target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "place_target")
        self.grasp_cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "grasp_cube")
        self.recorder = TrajectoryRecorder(record_path) if record_path is not None else None

    def _set_gripper(self, opening: float) -> dict[str, float]:
        opening = float(np.clip(opening, 0.0, 0.04))
        left_actuator = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_finger_ctrl")
        right_actuator = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_finger_ctrl")
        self.data.ctrl[left_actuator] = opening
        self.data.ctrl[right_actuator] = opening
        return {"left": opening, "right": opening}

    def run_scripted_step(self) -> TrajectoryFrame:
        target_position = np.array(self.model.body_pos[self.pick_target_body_id], dtype=float)
        self.controller.step_to_target(target_position)
        gripper_state = self._set_gripper(0.02)
        mujoco.mj_step(self.model, self.data)
        object_position = self.data.xpos[self.grasp_cube_body_id].copy()
        frame = TrajectoryFrame(
            timestamp=float(self.data.time),
            joint_positions=self.controller.current_joint_positions().tolist(),
            gripper_state=gripper_state,
            target_pose={"position": target_position.tolist(), "quaternion": [0.0, 0.0, 0.0, 1.0]},
            object_pose={"position": object_position.tolist(), "quaternion": [0.0, 0.0, 0.0, 1.0]},
        )
        if self.recorder is not None:
            self.recorder.write(frame)
        return frame
```

- [ ] **Step 4: Run the pick-and-place smoke test**

Run:

```bash
python -m pytest tests/test_pick_place_smoke.py -q
```

Expected: `1 passed`

- [ ] **Step 5: Commit the pick-and-place task**

Run:

```bash
git add tasks/my6dof_pick_place.py tests/test_pick_place_smoke.py
git commit -m "feat: add my6dof pick place task"
```

Expected: one commit created for the pick-and-place task

### Task 6: Add Pinocchio-To-MuJoCo Verification And User-Facing Documentation

**Files:**
- Create: `src/ur5_style_arm/mujoco_verify.py`
- Create: `docs/my6dof_mujoco_usage.md`
- Create: `tests/test_mujoco_pinocchio_consistency.py`

- [ ] **Step 1: Write failing consistency tests**

Create `tests/test_mujoco_pinocchio_consistency.py`:

```python
from pathlib import Path

import mujoco
import numpy as np

from ur5_style_arm import UR5StyleArm
from ur5_style_arm.mujoco_verify import compare_end_effector_positions


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_follow_target.xml"


def test_pinocchio_and_mujoco_end_effector_positions_are_close_at_neutral_pose():
    arm = UR5StyleArm()
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    report = compare_end_effector_positions(arm, model, data, arm.neutral_q)

    assert report["pinocchio_position"].shape == (3,)
    assert report["mujoco_position"].shape == (3,)
    assert report["position_error_norm"] < 0.35
```

- [ ] **Step 2: Run the consistency test and verify it fails**

Run:

```bash
python -m pytest tests/test_mujoco_pinocchio_consistency.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'ur5_style_arm.mujoco_verify'`

- [ ] **Step 3: Implement the verification helper and usage guide**

Create `src/ur5_style_arm/mujoco_verify.py`:

```python
from __future__ import annotations

import mujoco
import numpy as np

from .pose_utils import pose_input_to_se3


def compare_end_effector_positions(arm, model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray) -> dict[str, np.ndarray | float]:
    q = np.asarray(q, dtype=float)
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)

    pinocchio_pose = arm.forward_kinematics(q, output="matrix")
    pinocchio_position = pinocchio_pose[:3, 3]
    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    mujoco_position = data.site_xpos[ee_site_id].copy()
    error = mujoco_position - pinocchio_position

    return {
        "pinocchio_position": pinocchio_position,
        "mujoco_position": mujoco_position,
        "position_error": error,
        "position_error_norm": float(np.linalg.norm(error)),
    }
```

Create `docs/my6dof_mujoco_usage.md`:

```markdown
# My6DOF MuJoCo Usage

## Overview

This workspace contains a local `my6dof` MuJoCo migration that mirrors the structure of `wxai` concepts without modifying upstream repositories.

## Components

- `src/ur5_style_arm/`: Pinocchio kinematics and offline verification
- `mujoco_assets/my6dof/`: MuJoCo XML robot and scene assets
- `mujoco_control/`: online differential IK controller
- `tasks/`: follow-target and pick-and-place task entrypoints

## Running Headless Smoke Tests

```bash
python -m pytest tests/test_mujoco_assets.py tests/test_follow_target_smoke.py tests/test_pick_place_smoke.py tests/test_mujoco_pinocchio_consistency.py -q
```

## Running The Tasks

```bash
python -c "from tasks.my6dof_follow_target import FollowTargetTask; task = FollowTargetTask(interactive=False, record_path='logs/follow.jsonl'); print(task.step_to_target([0.42, 0.05, 0.42]).to_dict())"
python -c "from tasks.my6dof_pick_place import PickPlaceTask; task = PickPlaceTask(interactive=False, record_path='logs/pick.jsonl'); print(task.run_scripted_step().to_dict())"
```

## Recording Format

Each JSONL line contains:

- `timestamp`
- `joint_positions`
- `gripper_state`
- `target_pose`
- `object_pose`
```

- [ ] **Step 4: Run the consistency test and then the full MuJoCo regression subset**

Run:

```bash
python -m pytest tests/test_mujoco_pinocchio_consistency.py -q
python -m pytest tests/test_mujoco_import.py tests/test_mujoco_assets.py tests/test_recording.py tests/test_follow_target_smoke.py tests/test_pick_place_smoke.py tests/test_mujoco_pinocchio_consistency.py -q
```

Expected:

- the consistency test passes
- the MuJoCo subset prints `8 passed`

- [ ] **Step 5: Commit the verifier and docs**

Run:

```bash
git add src/ur5_style_arm/mujoco_verify.py docs/my6dof_mujoco_usage.md tests/test_mujoco_pinocchio_consistency.py
git commit -m "feat: add mujoco verification and docs"
```

Expected: one commit created for verification and documentation

## Self-Review

### Spec Coverage

- Parallel `my6dof` robot family: covered by Tasks 2, 4, and 5
- Preserve Pinocchio geometry and keep it for offline verification: covered by Task 6
- `wxai`-style MuJoCo asset structure: covered by Tasks 2 and 4
- Only pick-and-place and follow-target tasks: covered by Tasks 4 and 5
- Recording only, no replay: covered by Task 3 and task integrations in Tasks 4 and 5
- Interactive in-simulator targets: covered by the scene targets and task stepping in Tasks 4 and 5
- Tests and documentation: covered by Tasks 1 through 6, especially Task 6

No spec gaps found.

### Placeholder Scan

- Searched mentally for `TBD`, `TODO`, “similar to”, and vague “write tests” phrasing
- Every task includes exact files, concrete code, commands, and expected results

### Type Consistency

- Robot family name is consistently `my6dof`
- Controller class name is consistently `DifferentialIKController`
- Recording schema is consistently `timestamp`, `joint_positions`, `gripper_state`, `target_pose`, `object_pose`
- End-effector site name is consistently `ee_site`

No naming or type mismatches found.
