# My6DOF MuJoCo Project

`My6DOF MuJoCo Project` is a standalone 6-DOF robot-arm repository that combines:

- a Pinocchio-backed kinematics package exposed as `ur5_style_arm`
- MuJoCo robot and scene assets for a custom `my6dof` arm
- a differential IK control layer for target tracking and task execution
- task modules for `follow_target` and `pick_place`
- pick-place planning, evaluation, and JSONL trajectory recording utilities

The repository is organized as an independent project. It is inspired by the structure of the Trossen / `wxai` ecosystem, but it does not depend on editing upstream repositories in place.

## Current Scope

The current codebase focuses on:

- Pinocchio forward kinematics, inverse kinematics, and velocity kinematics
- MuJoCo scene loading and end-effector consistency checks
- target-following in simulation
- scripted and interactive pick-place flows
- explicit pick-place stage planning and post-run evaluation
- regression tests for assets, kinematics, task flow, and recording

The current codebase does not include:

- replay tooling for recorded trajectories
- imitation-learning data pipelines
- upstream Trossen repository modifications
- hardware drivers or external input devices

## Repository Layout

```text
.
|- demo.py
|- docs/
|  |- my6dof_mujoco_usage.md
|  `- my6dof_qa_rewritten.md
|- mujoco_assets/
|  `- my6dof/
|     |- my6dof_base.xml
|     |- scene_my6dof_follow_target.xml
|     `- scene_my6dof_pick_place.xml
|- mujoco_control/
|  |- __init__.py
|  `- controller.py
|- src/
|  `- ur5_style_arm/
|     |- __init__.py
|     |- exceptions.py
|     |- kinematics.py
|     |- mujoco_verify.py
|     |- pose_utils.py
|     `- robot_model.py
|- tasks/
|  |- my6dof_follow_target.py
|  |- my6dof_pick_place.py
|  |- pick_place_evaluation.py
|  |- pick_place_plan.py
|  `- recording.py
|- tests/
|- README.md
`- pyproject.toml
```

## Main Modules

### `src/ur5_style_arm/`

Pinocchio-based kinematics package with the public API:

- `UR5StyleArm`
- `IKConvergenceError`

It provides:

- forward kinematics
- full-pose inverse kinematics
- inverse kinematics error reporting
- forward velocity and inverse velocity utilities
- pose conversion helpers
- MuJoCo end-effector comparison helpers

### `mujoco_control/controller.py`

Control helpers for the MuJoCo scenes, including:

- differential IK stepping to a target position
- pose-aware stepping to a target position and orientation
- direct actuator command helpers
- gripper opening commands

### `tasks/`

Task-level entry points and utilities:

- `my6dof_follow_target.py`: follow-target task with headless and interactive modes
- `my6dof_pick_place.py`: pick-place task with scripted episode execution and interactive stepping
- `pick_place_plan.py`: explicit multi-stage pick-place plan generation
- `pick_place_evaluation.py`: episode success and failure-reason evaluation
- `recording.py`: JSONL trajectory recording

## Environment And Installation

Recommended baseline:

- Python `3.10+`
- MuJoCo `>=3.2`
- `dm_control`
- `dm_env`
- `numpy>=1.26`
- Pinocchio

Important: `pinocchio` is required by `src/ur5_style_arm/kinematics.py`, but it is not currently declared in `pyproject.toml`. Use a robotics environment where Pinocchio is already installed before running the kinematics package or MuJoCo tasks.

Inside your prepared environment:

```bash
python -m pip install -e .[dev]
```

## Quick Start

### 1. Package smoke demo

Run the local kinematics demo:

```bash
python demo.py
```

This prints:

- a forward-kinematics transform
- an inverse-kinematics solution
- a forward twist
- an inverse-velocity solution

### 2. Follow target

Headless:

```bash
python -m tasks.my6dof_follow_target --headless --target 0.42 0.05 0.42 --record-path logs/follow.jsonl
```

Interactive viewer:

```bash
python -m tasks.my6dof_follow_target --record-path logs/follow.jsonl
```

### 3. Pick and place

Headless scripted episode:

```bash
python -m tasks.my6dof_pick_place --headless --record-path logs/pick.jsonl
```

Interactive viewer:

```bash
python -m tasks.my6dof_pick_place --record-path logs/pick.jsonl
```

The headless pick-place entry point returns an evaluated episode payload that includes:

- `frames`
- `grasp_success`
- `episode_success`
- `failure_reason`

## Recording Format

Trajectory data is written as JSONL. Each frame records:

- `timestamp`
- `joint_positions`
- `gripper_state`
- `target_pose`
- `object_pose`

The repository currently supports recording only. It does not yet include replay or dataset export pipelines.

## Testing

Run the full test suite:

```bash
python -m pytest -q
```

Useful focused checks:

```bash
python -m pytest tests/test_package_import.py -q
python -m pytest tests/test_demo_smoke.py -q
python -m pytest tests/test_mujoco_pinocchio_consistency.py -q
python -m pytest tests/test_pick_place_plan.py -q
python -m pytest tests/test_pick_place_evaluation.py -q
```

Coverage in `tests/` currently includes:

- package import and demo smoke tests
- FK / IK / velocity kinematics tests
- MuJoCo asset and scene layout checks
- MuJoCo / Pinocchio consistency checks
- follow-target smoke coverage
- pick-place plan, evaluation, and interactive-stability checks
- trajectory recording tests

## Notes

- `docs/my6dof_mujoco_usage.md` contains a concise usage-oriented companion document.
- `docs/my6dof_qa_rewritten.md` contains a project Q&A / explanation draft aligned to the repository.
- The MuJoCo task stack is under active iteration, so task logic, stage definitions, and evaluation rules are more detailed than in the earliest version of this repository.

## Related References

- [Trossen Robotics arm description](https://github.com/TrossenRobotics/trossen_arm_description)
- [Trossen Robotics MuJoCo tasks](https://github.com/TrossenRobotics/trossen_arm_mujoco)

## License

Choose and add the license that matches your intended distribution before publishing the repository publicly.
