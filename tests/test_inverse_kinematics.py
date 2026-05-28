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
