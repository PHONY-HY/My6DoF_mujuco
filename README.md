# My6DOF MuJoCo Project

Standalone 6DOF robot arm project that combines:

- a Pinocchio kinematics layer for forward/inverse kinematics and verification
- a MuJoCo simulation layer for task execution
- two `wxai`-inspired tasks:
  - `follow_target`
  - `pick_place`
- lightweight JSONL trajectory recording

This project is organized as an independent codebase for your own GitHub repository. It is inspired by the `wxai` structure from the Trossen Robotics ecosystem, but it does **not** modify or depend on editing the upstream repositories in-place.

## Features

- Custom `my6dof` MuJoCo robot family
- 6DOF arm geometry preserved from the earlier Pinocchio implementation
- `wxai`-style gripper integration
- Interactive target-following task
- Scripted pick-and-place task
- Trajectory recording with:
  - `timestamp`
  - `joint_positions`
  - `gripper_state`
  - `target_pose`
  - `object_pose`
- Pinocchio-to-MuJoCo end-effector consistency checks

## Repository Layout

```text
.
|- docs/
|  `- my6dof_mujoco_usage.md
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
|     |- kinematics.py
|     |- mujoco_verify.py
|     |- pose_utils.py
|     `- robot_model.py
|- tasks/
|  |- my6dof_follow_target.py
|  |- my6dof_pick_place.py
|  `- recording.py
|- tests/
`- pyproject.toml
```

## Runtime Environment

Recommended runtime:

- Python `3.10+`
- MuJoCo `3.2.3`
- `dm_control`
- `dm_env`
- `numpy`
- `pinocchio`

Your prepared VMware environment is the recommended place to actually run the MuJoCo tasks:

```bash
source trossen_mujoco_env/bin/activate
python -m pip install -e .[dev]
```

## Installation

Inside your MuJoCo-ready environment:

```bash
python -m pip install -e .[dev]
```

## Quick Start

### 1. Run Follow Target

Headless:

```bash
python -m tasks.my6dof_follow_target --headless --target 0.42 0.05 0.42 --record-path logs/follow.jsonl
```

Interactive viewer:

```bash
python -m tasks.my6dof_follow_target --record-path logs/follow.jsonl
```

### 2. Run Pick And Place

Headless scripted episode:

```bash
python -m tasks.my6dof_pick_place --headless --record-path logs/pick.jsonl
```

Interactive viewer:

```bash
python -m tasks.my6dof_pick_place --record-path logs/pick.jsonl
```

## Recording Format

Trajectory data is written as JSONL. Each line contains one frame with:

- `timestamp`
- `joint_positions`
- `gripper_state`
- `target_pose`
- `object_pose`

This project currently keeps **recording only**. It does not include replay or full dataset-pipeline tooling.

## Pinocchio Layer

The Pinocchio layer remains part of the project for:

- forward kinematics
- inverse kinematics
- velocity kinematics
- MuJoCo cross-verification

It is used as an offline reference and validation tool rather than the online MuJoCo task controller.

## Verification

Run all local tests:

```bash
python -m pytest -q
```

Run the MuJoCo/Pinocchio consistency test specifically:

```bash
python -m pytest tests/test_mujoco_pinocchio_consistency.py -q
```

Note:

- MuJoCo-dependent tests may be skipped automatically in environments where `mujoco` is not installed.
- The intended full execution environment is your VMware `trossen_mujoco_env`.

## Project Scope

Included:

- independent `my6dof` robot family
- `follow_target`
- `pick_place`
- JSONL trajectory recording
- Pinocchio verification helpers

Not included:

- upstream Trossen repository modifications
- replay pipeline
- imitation-learning dataset pipeline
- external input devices

## Related References

- [Trossen Robotics arm description](https://github.com/TrossenRobotics/trossen_arm_description)
- [Trossen Robotics MuJoCo tasks](https://github.com/TrossenRobotics/trossen_arm_mujoco)

## License / Ownership

You can publish this repository under your own GitHub account as an independent project. Before making it public, add the license of your choice and verify that any reused upstream assets or naming remain consistent with your intended distribution terms.
