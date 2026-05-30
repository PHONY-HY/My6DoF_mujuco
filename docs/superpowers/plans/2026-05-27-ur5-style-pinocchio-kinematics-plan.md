# UR5 Style Arm Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python Pinocchio package that programmatically constructs a UR5-style 6DOF arm and exposes forward kinematics, inverse kinematics, forward velocity kinematics, and inverse velocity kinematics with tests and a runnable demo.

**Architecture:** Use a small `src/ur5_style_arm` package. Keep pose conversion, robot construction, exceptions, and kinematics in separate files so the numerical code stays readable. Build the robot directly in code with Pinocchio, then expose a `UR5StyleArm` class that owns the model/data and wraps the public API.

**Tech Stack:** Python 3.10+, NumPy, Pinocchio Python bindings, pytest, setuptools editable install

---

## Preconditions

- The current workspace is not a Git repository yet, so initialize one before the first commit.
- The current Python environment does not have `pinocchio` installed yet.
- Use the official Pinocchio Conda install path before running the tests:

```bash
conda install pinocchio -c conda-forge -y
python -c "import pinocchio; print(pinocchio.__version__)"
```

Expected: the second command prints a version string and exits with code `0`.

## Planned File Structure

- Create: `pyproject.toml`
- Create: `.gitignore`
- Create: `src/ur5_style_arm/__init__.py`
- Create: `src/ur5_style_arm/exceptions.py`
- Create: `src/ur5_style_arm/pose_utils.py`
- Create: `src/ur5_style_arm/robot_model.py`
- Create: `src/ur5_style_arm/kinematics.py`
- Create: `demo.py`
- Create: `tests/test_package_import.py`
- Create: `tests/test_pose_utils.py`
- Create: `tests/test_robot_model.py`
- Create: `tests/test_forward_kinematics.py`
- Create: `tests/test_inverse_kinematics.py`
- Create: `tests/test_velocity_kinematics.py`
- Create: `tests/test_demo_smoke.py`

## Interface Decisions Locked In By This Plan

- Public class: `UR5StyleArm`
- Public pose input forms:
  - a dictionary with keys `position` and `quaternion`
  - a `numpy.ndarray` with shape `(4, 4)`
- Public FK output forms:
  - `output="matrix"` returns a `numpy.ndarray` with shape `(4, 4)`
  - `output="quat"` returns `{"position": np.ndarray(shape=(3,)), "quaternion": np.ndarray(shape=(4,))}`
- `q` and `qdot` are always `numpy.ndarray` with shape `(6,)`
- End-effector twist uses `[vx, vy, vz, wx, wy, wz]` in the base frame
- IK returns `(q_solution, info_dict)` on success
- IK raises `IKConvergenceError` on failure

### Task 1: Bootstrap The Package Skeleton

**Files:**
- Create: `pyproject.toml`
- Create: `.gitignore`
- Create: `src/ur5_style_arm/__init__.py`
- Create: `src/ur5_style_arm/exceptions.py`
- Create: `src/ur5_style_arm/kinematics.py`
- Create: `tests/test_package_import.py`

- [ ] **Step 1: Initialize Git and verify the Pinocchio runtime**

Run:

```bash
git init
conda install pinocchio -c conda-forge -y
python -c "import pinocchio; print(pinocchio.__version__)"
```

Expected:

- `git init` prints `Initialized empty Git repository`
- the version check prints a Pinocchio version

- [ ] **Step 2: Write the failing import smoke test**

Create `tests/test_package_import.py`:

```python
from ur5_style_arm import IKConvergenceError, UR5StyleArm


def test_public_api_is_exported():
    assert UR5StyleArm is not None
    assert IKConvergenceError.__name__ == "IKConvergenceError"
```

- [ ] **Step 3: Run the smoke test and confirm the package does not exist yet**

Run:

```bash
pytest tests/test_package_import.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'ur5_style_arm'`

- [ ] **Step 4: Create the minimal package skeleton**

Create `pyproject.toml`:

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

Create `.gitignore`:

```gitignore
__pycache__/
.pytest_cache/
.venv/
*.pyc
dist/
build/
*.egg-info/
```

Create `src/ur5_style_arm/exceptions.py`:

```python
class IKConvergenceError(RuntimeError):
    """Raised when inverse kinematics does not converge."""
```

Create `src/ur5_style_arm/kinematics.py`:

```python
class UR5StyleArm:
    """UR5-style 6DOF manipulator wrapper."""

    pass
```

Create `src/ur5_style_arm/__init__.py`:

```python
from .exceptions import IKConvergenceError
from .kinematics import UR5StyleArm

__all__ = ["IKConvergenceError", "UR5StyleArm"]
```

- [ ] **Step 5: Install the package in editable mode and rerun the test**

Run:

```bash
python -m pip install -e .[dev]
pytest tests/test_package_import.py -q
```

Expected:

- the editable install completes successfully
- pytest prints `1 passed`

- [ ] **Step 6: Commit the bootstrap**

Run:

```bash
git add pyproject.toml .gitignore src/ur5_style_arm/__init__.py src/ur5_style_arm/exceptions.py src/ur5_style_arm/kinematics.py tests/test_package_import.py
git commit -m "chore: bootstrap ur5 style arm package"
```

Expected: one commit created with the package skeleton

### Task 2: Implement Pose Conversion Utilities

**Files:**
- Create: `src/ur5_style_arm/pose_utils.py`
- Create: `tests/test_pose_utils.py`

- [ ] **Step 1: Write failing tests for pose input normalization and output conversion**

Create `tests/test_pose_utils.py`:

```python
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
```

- [ ] **Step 2: Run the pose utility tests and confirm they fail**

Run:

```bash
pytest tests/test_pose_utils.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'ur5_style_arm.pose_utils'`

- [ ] **Step 3: Implement pose normalization helpers**

Create `src/ur5_style_arm/pose_utils.py`:

```python
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
```

- [ ] **Step 4: Rerun the pose utility tests**

Run:

```bash
pytest tests/test_pose_utils.py -q
```

Expected: `3 passed`

- [ ] **Step 5: Commit the pose utilities**

Run:

```bash
git add src/ur5_style_arm/pose_utils.py tests/test_pose_utils.py
git commit -m "feat: add pose conversion utilities"
```

Expected: one commit created for pose handling

### Task 3: Build The UR5-Style Pinocchio Model

**Files:**
- Create: `src/ur5_style_arm/robot_model.py`
- Create: `tests/test_robot_model.py`

- [ ] **Step 1: Write failing tests for the robot model builder**

Create `tests/test_robot_model.py`:

```python
import numpy as np

from ur5_style_arm.robot_model import build_ur5_style_model


def test_robot_model_has_six_dof_and_tool_frame():
    model, data, ee_frame_id, lower, upper, neutral = build_ur5_style_model()

    assert model.nq == 6
    assert model.nv == 6
    assert model.frames[ee_frame_id].name == "tool0"
    assert lower.shape == (6,)
    assert upper.shape == (6,)
    assert neutral.shape == (6,)
    assert np.all(lower <= neutral)
    assert np.all(neutral <= upper)
```

- [ ] **Step 2: Run the robot model tests and confirm they fail**

Run:

```bash
pytest tests/test_robot_model.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'ur5_style_arm.robot_model'`

- [ ] **Step 3: Implement the hand-built UR5-style model**

Create `src/ur5_style_arm/robot_model.py`:

```python
from __future__ import annotations

import numpy as np
import pinocchio as pin


LINKS = {
    "base_height": 0.089159,
    "upper_arm": 0.425000,
    "forearm": 0.392250,
    "wrist_1": 0.109150,
    "wrist_2": 0.094650,
    "tool": 0.082300,
}

JOINT_LIMITS = np.deg2rad(
    np.array(
        [
            [-360.0, 360.0],
            [-180.0, 0.0],
            [-180.0, 180.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
            [-360.0, 360.0],
        ],
        dtype=float,
    )
)

NEUTRAL_Q = np.deg2rad(np.array([0.0, -90.0, 90.0, -90.0, 0.0, 0.0], dtype=float))


def _append_body(model: pin.Model, joint_id: int) -> None:
    inertia = pin.Inertia.FromSphere(1.0, 0.05)
    model.appendBodyToJoint(joint_id, inertia, pin.SE3.Identity())
    model.addJointFrame(joint_id)


def build_ur5_style_model() -> tuple[pin.Model, pin.Data, int, np.ndarray, np.ndarray, np.ndarray]:
    model = pin.Model()
    parent = 0

    joint_1 = model.addJoint(
        parent,
        pin.JointModelRZ(),
        pin.SE3(np.eye(3), np.array([0.0, 0.0, LINKS["base_height"]], dtype=float)),
        "joint_1",
    )
    _append_body(model, joint_1)

    joint_2 = model.addJoint(joint_1, pin.JointModelRY(), pin.SE3.Identity(), "joint_2")
    _append_body(model, joint_2)

    joint_3 = model.addJoint(
        joint_2,
        pin.JointModelRY(),
        pin.SE3(np.eye(3), np.array([LINKS["upper_arm"], 0.0, 0.0], dtype=float)),
        "joint_3",
    )
    _append_body(model, joint_3)

    joint_4 = model.addJoint(
        joint_3,
        pin.JointModelRY(),
        pin.SE3(np.eye(3), np.array([LINKS["forearm"], 0.0, 0.0], dtype=float)),
        "joint_4",
    )
    _append_body(model, joint_4)

    joint_5 = model.addJoint(
        joint_4,
        pin.JointModelRZ(),
        pin.SE3(np.eye(3), np.array([0.0, 0.0, LINKS["wrist_1"]], dtype=float)),
        "joint_5",
    )
    _append_body(model, joint_5)

    joint_6 = model.addJoint(
        joint_5,
        pin.JointModelRY(),
        pin.SE3(np.eye(3), np.array([0.0, 0.0, LINKS["wrist_2"]], dtype=float)),
        "joint_6",
    )
    _append_body(model, joint_6)

    ee_frame_id = model.addFrame(
        pin.Frame(
            "tool0",
            joint_6,
            joint_6,
            pin.SE3(np.eye(3), np.array([0.0, 0.0, LINKS["tool"]], dtype=float)),
            pin.FrameType.OP_FRAME,
        )
    )

    model.lowerPositionLimit = JOINT_LIMITS[:, 0].copy()
    model.upperPositionLimit = JOINT_LIMITS[:, 1].copy()
    data = model.createData()

    return (
        model,
        data,
        ee_frame_id,
        JOINT_LIMITS[:, 0].copy(),
        JOINT_LIMITS[:, 1].copy(),
        NEUTRAL_Q.copy(),
    )
```

- [ ] **Step 4: Rerun the robot model tests**

Run:

```bash
pytest tests/test_robot_model.py -q
```

Expected: `1 passed`

- [ ] **Step 5: Commit the model builder**

Run:

```bash
git add src/ur5_style_arm/robot_model.py tests/test_robot_model.py
git commit -m "feat: add ur5 style pinocchio model builder"
```

Expected: one commit created for the robot model

### Task 4: Implement Forward Kinematics

**Files:**
- Modify: `src/ur5_style_arm/kinematics.py`
- Create: `tests/test_forward_kinematics.py`

- [ ] **Step 1: Write failing tests for forward kinematics**

Create `tests/test_forward_kinematics.py`:

```python
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
```

- [ ] **Step 2: Run the FK tests and confirm they fail**

Run:

```bash
pytest tests/test_forward_kinematics.py -q
```

Expected: FAIL because `UR5StyleArm` does not implement `forward_kinematics`

- [ ] **Step 3: Replace the placeholder kinematics wrapper with a forward-kinematics implementation**

Replace `src/ur5_style_arm/kinematics.py` with:

```python
from __future__ import annotations

import numpy as np
import pinocchio as pin

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

    def forward_kinematics(self, q: np.ndarray, output: str = "matrix") -> np.ndarray | dict[str, np.ndarray]:
        transform = self._frame_pose(q)

        if output == "matrix":
            return se3_to_matrix(transform)
        if output == "quat":
            return se3_to_pose_dict(transform)

        raise ValueError("output must be either 'matrix' or 'quat'")
```

- [ ] **Step 4: Run the FK tests**

Run:

```bash
pytest tests/test_forward_kinematics.py -q
```

Expected: `2 passed`

- [ ] **Step 5: Commit the FK implementation**

Run:

```bash
git add src/ur5_style_arm/kinematics.py tests/test_forward_kinematics.py
git commit -m "feat: implement forward kinematics"
```

Expected: one commit created for FK

### Task 5: Implement Inverse Kinematics With Detailed Failures

**Files:**
- Modify: `src/ur5_style_arm/exceptions.py`
- Modify: `src/ur5_style_arm/kinematics.py`
- Create: `tests/test_inverse_kinematics.py`

- [ ] **Step 1: Write failing tests for reachable and unreachable IK**

Create `tests/test_inverse_kinematics.py`:

```python
import numpy as np
import pytest

from ur5_style_arm import IKConvergenceError, UR5StyleArm


def test_inverse_kinematics_recovers_a_pose_generated_by_forward_kinematics():
    arm = UR5StyleArm()
    q_target = np.array([0.2, -1.1, 1.0, -0.7, 0.3, 0.2], dtype=float)
    target_pose = arm.forward_kinematics(q_target, output="matrix")

    q_solution, info = arm.inverse_kinematics(target_pose, q0=arm.neutral_q, max_iters=300, tol=1e-6)
    solved_pose = arm.forward_kinematics(q_solution, output="matrix")

    assert solved_pose.shape == (4, 4)
    assert np.allclose(solved_pose, target_pose, atol=1e-4)
    assert info["iterations"] <= 300
    assert info["final_error_norm"] <= 1e-4


def test_inverse_kinematics_raises_detailed_error_for_unreachable_target():
    arm = UR5StyleArm()
    unreachable = np.eye(4)
    unreachable[:3, 3] = np.array([5.0, 5.0, 5.0], dtype=float)

    with pytest.raises(IKConvergenceError) as exc_info:
        arm.inverse_kinematics(unreachable, q0=arm.neutral_q, max_iters=50, tol=1e-8)

    error = exc_info.value
    assert error.final_q.shape == (6,)
    assert error.residual_twist.shape == (6,)
    assert error.iterations == 50
    assert error.final_error_norm > 0.0
    assert error.position_error_norm > 0.0
```

- [ ] **Step 2: Run the IK tests and confirm they fail**

Run:

```bash
pytest tests/test_inverse_kinematics.py -q
```

Expected: FAIL because `inverse_kinematics` is not implemented and `IKConvergenceError` has no diagnostic fields

- [ ] **Step 3: Expand the IK exception to carry residual diagnostics**

Replace `src/ur5_style_arm/exceptions.py` with:

```python
from __future__ import annotations

import numpy as np


class IKConvergenceError(RuntimeError):
    """Raised when inverse kinematics does not converge."""

    def __init__(
        self,
        message: str,
        *,
        final_q: np.ndarray,
        iterations: int,
        residual_twist: np.ndarray,
        position_error_norm: float,
        orientation_error_norm: float,
    ) -> None:
        super().__init__(message)
        self.final_q = np.asarray(final_q, dtype=float)
        self.iterations = int(iterations)
        self.residual_twist = np.asarray(residual_twist, dtype=float)
        self.final_error_norm = float(np.linalg.norm(self.residual_twist))
        self.position_error_norm = float(position_error_norm)
        self.orientation_error_norm = float(orientation_error_norm)
```

- [ ] **Step 4: Add a damped least-squares IK solver**

Replace `src/ur5_style_arm/kinematics.py` with:

```python
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
            lhs = jacobian @ jacobian.T + damping * np.eye(6)
            delta_q = jacobian.T @ np.linalg.solve(lhs, residual)
            q = np.clip(q + delta_q, self.lower_limits, self.upper_limits)

        raise IKConvergenceError(
            "inverse kinematics did not converge",
            final_q=q,
            iterations=max_iters,
            residual_twist=residual,
            position_error_norm=position_error_norm,
            orientation_error_norm=orientation_error_norm,
        )
```

- [ ] **Step 5: Run the IK tests**

Run:

```bash
pytest tests/test_inverse_kinematics.py -q
```

Expected: `2 passed`

- [ ] **Step 6: Commit the IK implementation**

Run:

```bash
git add src/ur5_style_arm/exceptions.py src/ur5_style_arm/kinematics.py tests/test_inverse_kinematics.py
git commit -m "feat: implement numerical inverse kinematics"
```

Expected: one commit created for IK

### Task 6: Implement Forward And Inverse Velocity Kinematics

**Files:**
- Modify: `src/ur5_style_arm/kinematics.py`
- Create: `tests/test_velocity_kinematics.py`

- [ ] **Step 1: Write failing tests for the Jacobian-based velocity mappings**

Create `tests/test_velocity_kinematics.py`:

```python
import numpy as np
import pytest

from ur5_style_arm import UR5StyleArm


def test_forward_and_inverse_velocity_are_consistent_away_from_singularities():
    arm = UR5StyleArm()
    q = np.array([0.3, -1.0, 1.1, -0.7, 0.5, 0.2], dtype=float)
    qdot = np.array([0.1, -0.05, 0.08, 0.02, -0.04, 0.03], dtype=float)

    twist = arm.forward_velocity(q, qdot)
    qdot_hat = arm.inverse_velocity(q, twist, damping=1e-6)

    assert twist.shape == (6,)
    assert qdot_hat.shape == (6,)
    assert np.allclose(qdot_hat, qdot, atol=1e-5)


def test_inverse_velocity_validates_twist_shape():
    arm = UR5StyleArm()
    q = np.array([0.3, -1.0, 1.1, -0.7, 0.5, 0.2], dtype=float)

    with pytest.raises(ValueError, match="shape"):
        arm.inverse_velocity(q, np.zeros(5))
```

- [ ] **Step 2: Run the velocity tests and confirm they fail**

Run:

```bash
pytest tests/test_velocity_kinematics.py -q
```

Expected: FAIL because the velocity methods do not exist yet

- [ ] **Step 3: Add base-frame Jacobian velocity mappings**

Replace `src/ur5_style_arm/kinematics.py` with:

```python
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
            lhs = jacobian @ jacobian.T + damping * np.eye(6)
            delta_q = jacobian.T @ np.linalg.solve(lhs, residual)
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
        return jacobian @ qdot

    def inverse_velocity(self, q: np.ndarray, ee_twist: np.ndarray, damping: float = 1e-6) -> np.ndarray:
        q = self._validate_vector(q, name="q")
        ee_twist = self._validate_vector(ee_twist, name="ee_twist")
        jacobian = self._base_frame_jacobian(q)
        lhs = jacobian @ jacobian.T + damping * np.eye(6)
        return jacobian.T @ np.linalg.solve(lhs, ee_twist)
```

- [ ] **Step 4: Run the velocity tests**

Run:

```bash
pytest tests/test_velocity_kinematics.py -q
```

Expected: `2 passed`

- [ ] **Step 5: Commit the velocity kinematics**

Run:

```bash
git add src/ur5_style_arm/kinematics.py tests/test_velocity_kinematics.py
git commit -m "feat: add jacobian based velocity kinematics"
```

Expected: one commit created for Jacobian velocity methods

### Task 7: Add A Runnable Demo And Final Regression Checks

**Files:**
- Create: `demo.py`
- Create: `tests/test_demo_smoke.py`

- [ ] **Step 1: Write a failing smoke test for the demo flow**

Create `tests/test_demo_smoke.py`:

```python
import numpy as np

from demo import run_demo


def test_demo_runs_and_returns_expected_artifacts():
    result = run_demo()

    assert result["fk_matrix"].shape == (4, 4)
    assert result["ik_solution"].shape == (6,)
    assert result["forward_twist"].shape == (6,)
    assert result["inverse_qdot"].shape == (6,)
    assert np.isfinite(result["ik_info"]["final_error_norm"])
```

- [ ] **Step 2: Run the demo smoke test and confirm it fails**

Run:

```bash
pytest tests/test_demo_smoke.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'demo'`

- [ ] **Step 3: Implement the runnable demo**

Create `demo.py`:

```python
from __future__ import annotations

from pprint import pprint

import numpy as np

from ur5_style_arm import UR5StyleArm


def run_demo() -> dict[str, np.ndarray | dict[str, float | int]]:
    arm = UR5StyleArm()
    q_seed = np.array([0.2, -1.1, 1.0, -0.7, 0.3, 0.2], dtype=float)
    qdot = np.array([0.1, -0.05, 0.08, 0.02, -0.04, 0.03], dtype=float)

    fk_matrix = arm.forward_kinematics(q_seed, output="matrix")
    ik_solution, ik_info = arm.inverse_kinematics(fk_matrix, q0=arm.neutral_q, max_iters=300, tol=1e-6)
    forward_twist = arm.forward_velocity(q_seed, qdot)
    inverse_qdot = arm.inverse_velocity(q_seed, forward_twist, damping=1e-6)

    return {
        "fk_matrix": fk_matrix,
        "ik_solution": ik_solution,
        "ik_info": ik_info,
        "forward_twist": forward_twist,
        "inverse_qdot": inverse_qdot,
    }


if __name__ == "__main__":
    pprint(run_demo())
```

- [ ] **Step 4: Run the smoke test and the full regression suite**

Run:

```bash
pytest tests/test_demo_smoke.py -q
pytest -q
python demo.py
```

Expected:

- the smoke test passes
- the full test suite passes
- `python demo.py` prints a dictionary containing FK, IK, and velocity results

- [ ] **Step 5: Commit the demo and regression pass**

Run:

```bash
git add demo.py tests/test_demo_smoke.py
git commit -m "feat: add demo script and regression coverage"
```

Expected: one commit created for the demo and final checks

## Self-Review

### Spec Coverage

- Robot built directly in Pinocchio code: covered by Task 3
- Full-pose FK with matrix and quaternion outputs: covered by Task 4
- Full-pose numerical IK with one solution and failure diagnostics: covered by Task 5
- Forward and inverse velocity kinematics in base-frame twist form: covered by Task 6
- Demo script and numerical validation tests: covered by Task 7

No spec gaps found.

### Placeholder Scan

- Searched conceptually for `TBD`, `TODO`, and vague instructions
- All tasks include exact files, test code, implementation code, commands, and expected outcomes

### Type Consistency

- Public class name is consistently `UR5StyleArm`
- Failure type is consistently `IKConvergenceError`
- Pose dict keys are consistently `position` and `quaternion`
- Vector sizes are consistently `(6,)`

No naming or type mismatches found.
