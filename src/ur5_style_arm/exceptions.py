from __future__ import annotations

import numpy as np


class IKConvergenceError(RuntimeError):
    """Raised when inverse kinematics does not converge."""

    def __init__(
        self,
        message: str,
        *,
        final_q: np.ndarray,
        iterations: int,
        residual_twist: np.ndarray,
        position_error_norm: float,
        orientation_error_norm: float,
    ) -> None:
        super().__init__(message)
        self.final_q = np.asarray(final_q, dtype=float)
        self.iterations = int(iterations)
        self.residual_twist = np.asarray(residual_twist, dtype=float)
        self.final_error_norm = float(np.linalg.norm(self.residual_twist))
        self.position_error_norm = float(position_error_norm)
        self.orientation_error_norm = float(orientation_error_norm)
