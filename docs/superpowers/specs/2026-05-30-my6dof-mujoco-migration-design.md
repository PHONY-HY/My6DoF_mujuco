# My6DOF MuJoCo Migration Design

## Overview

This project will migrate the previously built Pinocchio-based 6DOF arm into a MuJoCo task structure that is modeled after the `wxai` organization used in the Trossen Robotics repositories, while remaining a parallel custom robot rather than modifying the official `wxai` assets directly.

The migrated system will:

- preserve the existing 6DOF arm geometry and kinematic structure from the Pinocchio implementation
- adopt a `wxai`-style MuJoCo asset and task layout
- keep only two interactive tasks:
  - pick and place
  - follow target
- retain trajectory recording functionality
- exclude replay and full dataset pipeline features
- keep Pinocchio as an offline verification layer rather than the online control backend

## Goals

- Create a new parallel MuJoCo robot family for the custom 6DOF arm
- Preserve the existing Pinocchio arm geometry and joint organization
- Attach a `wxai`-style gripper to the custom 6DOF arm
- Provide `wxai`-inspired MuJoCo scenes for:
  - pick and place
  - follow target
- Use a MuJoCo-side online controller for task execution
- Keep Pinocchio available for offline FK/IK consistency checking and regression tests
- Record task trajectories with a lightweight task-oriented data format

## Non-Goals

- Modifying the official `trossen_arm_mujoco` repository in-place
- Replacing or overwriting the official `wxai` robot
- Keeping any tasks beyond pick and place and follow target
- Supporting external devices such as spacemice, gamepads, or mocap hardware in the first version
- Preserving the full `wxai` recording, replay, and dataset workflow
- Building imitation-learning or training pipelines

## Source Alignment

The migration is inspired by two upstream sources:

- `TrossenRobotics/trossen_arm_description`
- `TrossenRobotics/trossen_arm_mujoco`

The specific alignment choice is:

- keep the custom Pinocchio robot geometry
- mirror the `wxai` organizational style where useful
- create a parallel custom robot family rather than replacing `wxai`

This project will therefore be structurally inspired by `wxai`, but it will not claim to be a byte-for-byte or behavior-for-behavior clone of the official model stack.

## Chosen Approach

Three migration approaches were considered:

1. Create a parallel custom robot family while preserving the Pinocchio model and mirroring `wxai` task structure
2. Rewrite the custom arm to closely match official `wxai` structure and naming throughout
3. Build only the minimum scripts needed to run two tasks without meaningful `wxai` structural alignment

The selected approach is option 1.

This approach best matches the confirmed constraints:

- preserve the existing 6DOF arm geometry
- keep a strong `wxai`-style structure
- remain isolated from the official repositories
- retain Pinocchio as an offline verification tool

## System Architecture

The migrated project will be divided into four layers:

1. Pinocchio layer
2. MuJoCo asset layer
3. MuJoCo control layer
4. Task and recording layer

### Pinocchio Layer

This layer keeps the existing Pinocchio implementation of the 6DOF arm. It remains responsible for:

- forward kinematics
- inverse kinematics
- forward velocity kinematics
- inverse velocity kinematics
- offline consistency validation against the MuJoCo model

Pinocchio will not be the online task controller in the migrated design.

### MuJoCo Asset Layer

This layer defines the MuJoCo robot and scenes. It will introduce a new parallel robot family, tentatively named `my6dof`, rather than modifying or replacing `wxai`.

Responsibilities:

- define a MuJoCo robot base model for the custom 6DOF arm
- mount a `wxai`-style gripper at the end effector
- define task-specific scenes for pick and place and follow target

### MuJoCo Control Layer

This layer provides the online controller for running tasks in simulation.

Responsibilities:

- read the current MuJoCo joint and scene state
- read the interactive target state
- compute joint commands using a `wxai`-style controller organization
- drive the arm and gripper during task execution

### Task And Recording Layer

This layer exposes only the two supported tasks and a lightweight trajectory recording mechanism.

Responsibilities:

- task startup and scene loading
- task loop logic
- interactive target updates inside the simulator
- per-step trajectory logging

## Robot Model Migration

### Arm Body

The custom 6DOF arm geometry from the Pinocchio implementation will be migrated into MuJoCo by writing a MuJoCo XML robot base model, tentatively named:

- `my6dof_base.xml`

The MuJoCo robot model will preserve:

- 6 revolute joints
- the same arm topology
- the same approximate link lengths
- an end-effector attachment point consistent with the existing Pinocchio tool frame

This means the MuJoCo model is not derived from official `wxai` arm dimensions. Instead, it preserves the custom arm while adopting `wxai`-style organization.

### Gripper

The migrated design will use a `wxai`-style gripper rather than inventing a brand-new end-effector design.

This decision was explicitly chosen to:

- stay close to `wxai` task assumptions
- simplify pick-and-place task integration
- avoid expanding scope into custom gripper design

The gripper will be attached to the custom 6DOF arm at the end-effector mount defined in the MuJoCo base XML.

## Scenes And Tasks

Only two tasks will be supported.

### Pick And Place

The pick-and-place scene will include:

- the custom 6DOF robot
- the `wxai`-style gripper
- a tabletop workspace
- at least one graspable object
- a placement target region
- an interactive in-simulation target entity used by the controller/task logic

Tentative scene file:

- `scene_my6dof_pick_place.xml`

### Follow Target

The follow-target scene will include:

- the custom 6DOF robot
- the `wxai`-style gripper
- a target body or site that can be updated interactively during simulation

Tentative scene file:

- `scene_my6dof_follow_target.xml`

### Interaction Style

The confirmed interaction mode is:

- interactive targets inside the simulator

This means the first version will support in-simulation target manipulation and updates, but will not include external device input support.

## Controller Design

The online control architecture will follow `wxai` style, but not necessarily line-by-line implementation parity.

### Responsibilities

The MuJoCo controller will:

- observe current robot joint state
- observe current target pose or target body state
- compute control commands that move the arm toward the target
- command the gripper when needed for the pick-and-place task

### Relationship To Pinocchio

The controller will not call Pinocchio in the online task loop by default.

Instead:

- MuJoCo handles online task stepping
- Pinocchio is preserved for offline checking and testing

This was explicitly chosen to keep the migrated system structurally close to `wxai`, which is task/controller centered in the simulator.

## Trajectory Recording

The confirmed scope is recording only.

The following are intentionally out of scope:

- replay workflows
- dataset packaging pipelines
- learning-oriented export systems

### Recorded Fields

Each recorded timestep will include:

- `timestamp`
- `joint_positions`
- `gripper_state`
- `target_pose`
- `object_pose`

Meaning:

- `joint_positions`: 6 joint angles for the custom arm
- `gripper_state`: gripper opening/closing state or equivalent actuator state
- `target_pose`: current interactive target pose
- `object_pose`: object pose for pick-and-place; optional or empty for follow-target

### File Format

The preferred first-version storage format is:

- JSONL

Rationale:

- easy to inspect
- easy to append frame by frame
- good enough for lightweight trajectory logging
- easy to transform later into training-ready formats if ever needed

## File And Module Layout

The implementation should use a layout that mirrors `wxai` ideas while remaining local to this workspace.

Tentative structure:

- `src/ur5_style_arm/`
  - existing Pinocchio layer retained
- `mujoco_assets/my6dof/`
  - `my6dof_base.xml`
  - `scene_my6dof_pick_place.xml`
  - `scene_my6dof_follow_target.xml`
- `mujoco_control/`
  - controller implementation for online MuJoCo control
- `tasks/`
  - `my6dof_pick_place.py`
  - `my6dof_follow_target.py`
  - recording utilities
- `tests/`
  - Pinocchio-to-MuJoCo consistency checks
  - task smoke tests

The exact filenames may be adjusted during implementation if repository constraints require it, but the separation of responsibilities should remain the same.

## Verification Strategy

The final migrated system should be verified at three levels.

### Level 1: Structural Verification

- MuJoCo robot model loads successfully
- both scenes load successfully
- task scripts can start without immediate model or asset failures

### Level 2: Functional Verification

- follow-target task moves the end effector toward the interactive target
- pick-and-place task can execute its control loop with the gripper present
- trajectory recording writes valid JSONL data

### Level 3: Cross-Validation

Pinocchio remains part of the success criteria. The implementation should include checks that:

- selected MuJoCo joint configurations correspond to reasonable Pinocchio FK outputs
- target-driven motions remain broadly consistent with the retained kinematic model
- regression tests continue to pass on the Pinocchio layer

## Success Criteria

The migration is complete when:

- a parallel `my6dof` MuJoCo robot family exists in the current workspace
- the custom 6DOF geometry has been migrated into MuJoCo
- a `wxai`-style gripper is attached
- only pick and place and follow target are supported
- both tasks can run in MuJoCo
- both tasks support interactive in-simulation targets
- trajectory recording works
- Pinocchio remains available for offline verification and testing
- the system includes tests and documentation explaining the migration

## Risks And Mitigations

- MuJoCo and Pinocchio models may diverge numerically
  - Mitigation: add explicit cross-validation tests and document tolerated differences
- The `wxai` task assumptions may depend on gripper details not present in the custom arm
  - Mitigation: reuse `wxai`-style gripper concepts rather than inventing a brand-new gripper
- The project may expand toward full dataset tooling
  - Mitigation: lock scope to recording only and exclude replay and training pipelines
- The project may drift toward direct upstream modification
  - Mitigation: keep the migrated robot as a parallel local family named separately from `wxai`
- Interactive target handling may become entangled with future device support
  - Mitigation: keep first-version interaction strictly in-simulator and do not add external input support
