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
