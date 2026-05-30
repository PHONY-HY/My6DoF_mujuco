# My6DOF MuJoCo Usage

## Overview

This project keeps the original Pinocchio 6DOF arm as an offline kinematics layer and adds a standalone MuJoCo task stack inspired by `wxai`, without modifying the Trossen Robotics upstream repositories.

## Structure

- `src/ur5_style_arm/`
  - Pinocchio FK, IK, velocity kinematics, and MuJoCo verification helpers
- `mujoco_assets/my6dof/`
  - robot XML and two scene XML files
- `mujoco_control/`
  - differential IK controller
- `tasks/`
  - follow-target task
  - pick-and-place task
  - JSONL recording

## Recommended Runtime

Use the already prepared VMware environment:

```bash
source trossen_mujoco_env/bin/activate
python -m pip install -e .[dev]
```

## Run Follow Target

Headless:

```bash
python -m tasks.my6dof_follow_target --headless --target 0.42 0.05 0.42 --record-path logs/follow.jsonl
```

Interactive viewer:

```bash
python -m tasks.my6dof_follow_target --record-path logs/follow.jsonl
```

## Run Pick And Place

Headless scripted episode:

```bash
python -m tasks.my6dof_pick_place --headless --record-path logs/pick.jsonl
```

Interactive viewer:

```bash
python -m tasks.my6dof_pick_place --record-path logs/pick.jsonl
```

## Recording Format

Each JSONL line records:

- `timestamp`
- `joint_positions`
- `gripper_state`
- `target_pose`
- `object_pose`

## Pinocchio Cross-Check

Run the MuJoCo-to-Pinocchio consistency test:

```bash
python -m pytest tests/test_mujoco_pinocchio_consistency.py -q
```
