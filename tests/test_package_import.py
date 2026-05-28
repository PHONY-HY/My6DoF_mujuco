from ur5_style_arm import IKConvergenceError, UR5StyleArm


def test_public_api_is_exported():
    assert UR5StyleArm is not None
    assert IKConvergenceError.__name__ == "IKConvergenceError"
