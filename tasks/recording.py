from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np


def _to_float_list(values: list[float] | tuple[float, ...] | np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=float).tolist()]


def _normalize_pose(pose: dict[str, Any] | None) -> dict[str, list[float]] | None:
    if pose is None:
        return None
    return {
        "position": _to_float_list(pose["position"]),
        "quaternion": _to_float_list(pose["quaternion"]),
    }


@dataclass
class TrajectoryFrame:
    timestamp: float
    joint_positions: list[float] | np.ndarray
    gripper_state: dict[str, float]
    target_pose: dict[str, Any]
    object_pose: dict[str, Any] | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": float(self.timestamp),
            "joint_positions": _to_float_list(self.joint_positions),
            "gripper_state": {key: float(value) for key, value in self.gripper_state.items()},
            "target_pose": _normalize_pose(self.target_pose),
            "object_pose": _normalize_pose(self.object_pose),
        }


class TrajectoryRecorder:
    def __init__(self, output_path: str | Path) -> None:
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.output_path.open("w", encoding="utf-8")

    def write(self, frame: TrajectoryFrame) -> None:
        self._handle.write(json.dumps(frame.to_dict(), ensure_ascii=True) + "\n")
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()
