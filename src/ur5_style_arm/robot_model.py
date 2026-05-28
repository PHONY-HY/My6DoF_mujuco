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
