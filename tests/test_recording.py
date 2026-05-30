import json
from pathlib import Path

from tasks.recording import TrajectoryFrame, TrajectoryRecorder


ROOT = Path(__file__).resolve().parents[1]


def test_trajectory_frame_serializes_numpy_friendly_payload():
    frame = TrajectoryFrame(
        timestamp=0.1,
        joint_positions=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        gripper_state={"left": 0.01, "right": 0.01},
        target_pose={"position": [0.4, 0.0, 0.4], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        object_pose={"position": [0.3, 0.0, 0.2], "quaternion": [0.0, 0.0, 0.0, 1.0]},
    )

    payload = frame.to_dict()

    assert payload["timestamp"] == 0.1
    assert payload["joint_positions"][-1] == 5.0
    assert payload["gripper_state"]["left"] == 0.01


def test_trajectory_recorder_writes_jsonl():
    output_dir = ROOT / ".tmp" / "recording_test"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "trajectory.jsonl"
    if output_path.exists():
        output_path.unlink()

    recorder = TrajectoryRecorder(output_path)
    frame = TrajectoryFrame(
        timestamp=0.2,
        joint_positions=[0.1] * 6,
        gripper_state={"left": 0.02, "right": 0.02},
        target_pose={"position": [0.5, 0.1, 0.4], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        object_pose=None,
    )

    recorder.write(frame)
    recorder.close()

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    data = json.loads(lines[0])
    assert data["object_pose"] is None
    assert data["joint_positions"][0] == 0.1
