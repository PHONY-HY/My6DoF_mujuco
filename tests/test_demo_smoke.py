import numpy as np

from demo import run_demo


def test_demo_runs_and_returns_expected_artifacts():
    result = run_demo()

    assert result["fk_matrix"].shape == (4, 4)
    assert result["ik_solution"].shape == (6,)
    assert result["forward_twist"].shape == (6,)
    assert result["inverse_qdot"].shape == (6,)
    assert np.isfinite(result["ik_info"]["final_error_norm"])
