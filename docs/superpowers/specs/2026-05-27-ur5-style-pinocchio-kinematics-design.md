# UR5 Style 6DOF Arm Kinematics Design

## Overview

This project will implement a Python-based kinematics toolkit using Pinocchio for a hand-built UR5-style 6DOF serial manipulator. The model will be created directly in code rather than loaded from URDF so that the joint topology, frame transforms, and kinematic computations remain easy to inspect and learn from.

The toolkit will support:

- Forward kinematics: joint angles to end-effector pose
- Inverse kinematics: target end-effector pose to one valid joint-angle solution
- Forward velocity kinematics: joint angles and joint velocities to end-effector twist
- Inverse velocity kinematics: joint angles and end-effector twist to joint velocities

The design prioritizes clarity and teaching value while keeping the robot structure and dimensions close to a real UR5-style arm.

## Goals

- Build a 6R serial manipulator in Pinocchio with UR5-style joint layout
- Accept full end-effector pose targets, not position-only targets
- Support two pose input forms:
  - position plus quaternion
  - 4x4 homogeneous transform
- Internally normalize pose representations to `pin.SE3`
- Return one inverse-kinematics solution when convergence succeeds
- Raise a clear error with residual information when inverse kinematics does not converge
- Provide both forward and inverse velocity mappings via the Jacobian
- Keep the code compact, readable, and suitable for study or extension

## Non-Goals

- Exact industrial reproduction of the official UR5 or UR5e model
- Closed-form analytic inverse kinematics
- Collision checking, trajectory planning, or dynamics
- ROS, MoveIt, or simulator integration
- Multiple inverse-kinematics solution branches in one call

## Chosen Approach

Three approaches were considered:

1. Hand-build a UR5-style robot directly in Python with Pinocchio
2. Load a real UR5-style URDF with Pinocchio
3. Use a hybrid design with URDF under the hood and a simplified teaching wrapper

The selected approach is option 1. It best matches the goal of understanding how the arm is modeled and how kinematics are solved, while avoiding URDF complexity that would distract from the learning objective.

## Robot Model

### Topology

The arm will be a 6-joint revolute serial chain with a UR5-style structure:

- Joint 1: base rotation
- Joint 2: shoulder
- Joint 3: elbow
- Joint 4: wrist 1
- Joint 5: wrist 2
- Joint 6: wrist 3

Each joint will be created explicitly in Pinocchio. Fixed transforms between joints will encode the link lengths and frame offsets. A dedicated end-effector frame will be added at the tool tip.

### Modeling Strategy

The implementation will:

- Construct the Pinocchio model directly in Python
- Add revolute joints one by one
- Define each joint axis explicitly
- Define parent-child transforms explicitly
- Add one final operational frame for the tool/end-effector

The geometry will be UR5-style rather than guaranteed millimeter-exact. This means the manipulator will preserve the familiar 6-axis industrial-arm structure and realistic proportions without claiming official model parity.

### Joint Limits

Per-joint limits will be included in the model. During inverse-kinematics iteration, each updated joint value will be clamped back into its valid range after every step.

## Pose Representation

### External Inputs

The public API will accept two pose formats:

1. Position plus quaternion
   - position: `(x, y, z)`
   - quaternion order: `(qx, qy, qz, qw)`
2. 4x4 homogeneous transform matrix

### Internal Representation

All pose inputs will be converted to `pin.SE3` internally. This keeps the forward, inverse, and Jacobian-based methods consistent and avoids mixing pose math across multiple formats.

### Output Formats

Forward kinematics will support two output forms:

- `4x4` homogeneous transform matrix
- position plus quaternion

## Public API

The project will expose a lightweight class named `UR5StyleArm`.

### Core Methods

- `forward_kinematics(q, output="matrix" | "quat")`
- `inverse_kinematics(target_pose, q0=None, max_iters=..., tol=...)`
- `forward_velocity(q, qdot)`
- `inverse_velocity(q, ee_twist, damping=...)`

### Data Conventions

- `q`: 6-element NumPy array of joint angles in radians
- `qdot`: 6-element NumPy array of joint velocities in radians per second
- `ee_twist`: 6-element vector ordered as `[vx, vy, vz, wx, wy, wz]`
- All lengths use meters
- All internal angular quantities use radians
- End-effector twist is expressed in the base frame
- In this fixed-base model, the base frame coincides with the world frame

## Forward Kinematics

Forward kinematics will:

- Take a 6D joint-angle vector `q`
- Run Pinocchio forward kinematics
- Extract the end-effector frame pose
- Return either a 4x4 transform or position-plus-quaternion output

Validation checks will ensure:

- `q` has shape `(6,)`
- Rotation outputs are proper rotation matrices or normalized quaternions

## Inverse Kinematics

### Solver Type

Inverse kinematics will use a numerical iterative solver based on pose error and the end-effector Jacobian. This is the right fit for Pinocchio and for a teaching-oriented implementation that still behaves like a practical robotics method.

### Iteration Strategy

Each iteration will:

- Compute current end-effector pose from the current `q`
- Compute pose error between current pose and target pose
- Convert the pose error to a 6D residual twist
- Compute the end-effector Jacobian
- Solve for a joint update using a damped pseudoinverse
- Apply the update to `q`
- Enforce joint limits

### Convergence Behavior

The solver returns one solution only.

Success criteria:

- residual norm below tolerance
- position and orientation residual norms are always reported for diagnostics

Failure behavior:

- raise a custom `IKConvergenceError`
- include detailed residual information

### Failure Information

The inverse-kinematics failure path will report:

- final joint vector
- iteration count
- full residual twist
- final residual norm
- position residual norm
- orientation residual norm
- failure message

### Initial Guess

`q0` may be supplied by the caller. If omitted, the solver will start from a neutral default posture inside the valid joint limits. Because the solver is numerical, the returned solution may vary with the initial guess even for the same target pose.

## Velocity Kinematics

### Forward Velocity

`forward_velocity(q, qdot)` will:

- compute the Jacobian at `q`
- map `qdot` into the end-effector twist

### Inverse Velocity

`inverse_velocity(q, ee_twist, damping=...)` will:

- compute the Jacobian at `q`
- solve for `qdot` using a damped pseudoinverse

The same 6D twist convention will be used throughout:

- linear velocity first
- angular velocity second
- base-frame expression

## Error Handling

The code will prefer explicit, informative failures over silent fallback behavior.

Examples:

- invalid pose shape should raise `ValueError`
- invalid quaternion shape or zero-norm quaternion should raise `ValueError`
- wrong joint-vector size should raise `ValueError`
- inverse-kinematics non-convergence should raise `IKConvergenceError`

## File Structure

The implementation is expected to use a small module layout:

- `robot_model.py`: build the UR5-style Pinocchio model
- `pose_utils.py`: pose-format conversion helpers
- `kinematics.py`: FK, IK, Jacobian, and velocity mappings
- `demo.py`: runnable examples
- `tests/`: focused numerical validation tests

This layout keeps modeling, math, and demonstration logic separated and easier to read.

## Verification Plan

At minimum, the project will include the following checks:

1. Forward-kinematics sanity checks
   - output dimensions are correct
   - rotation is valid
   - output format conversions agree
2. IK consistency checks
   - generate a target pose from FK
   - solve IK back from that pose
   - verify the recovered pose is within tolerance
3. Velocity consistency checks
   - compute end-effector twist from `q` and `qdot`
   - map twist back to `qdot`
   - verify the result matches within tolerance
4. Failure-path checks
   - use an unreachable target pose
   - verify `IKConvergenceError` is raised
   - verify residual fields are present
5. Demo coverage
   - show pose-to-joint-angle solving
   - show joint-angle-to-pose solving
   - show forward and inverse velocity examples

## Success Criteria

The task is complete when:

- the 6DOF UR5-style arm is built directly in Pinocchio code
- full-pose forward kinematics works
- full-pose numerical inverse kinematics works for reachable targets
- inverse kinematics reports useful residuals for unreachable targets
- forward velocity kinematics works from `q` and `qdot`
- inverse velocity kinematics works from `q` and end-effector twist
- the project includes a demonstration script and basic numerical tests

## Risks And Mitigations

- Numerical IK may be sensitive to the initial guess
  - Mitigation: expose `q0`, damping, iteration limits, and tolerances
- Near-singular Jacobians can destabilize updates
  - Mitigation: use damped pseudoinverse
- Quaternion or frame-convention mistakes can invalidate results
  - Mitigation: centralize pose conversion utilities and test round-trips
- A hand-built UR5-style model may drift from official UR geometry
  - Mitigation: document clearly that the model is UR5-style rather than an exact official replica
