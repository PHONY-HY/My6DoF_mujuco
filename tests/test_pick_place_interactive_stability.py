import numpy as np
import pytest


pytest.importorskip("mujoco")

from tasks.my6dof_pick_place import PickPlaceTask


def _run_interactive_loop_without_viewer(task: PickPlaceTask, steps: int = 700) -> tuple[float, np.ndarray]:
    max_cube_speed = 0.0

    for _ in range(steps):
        task._interactive_tick()
        cube_speed = float(np.linalg.norm(task.data.qvel[task.cube_qvel_adr : task.cube_qvel_adr + 6]))
        max_cube_speed = max(max_cube_speed, cube_speed)

    return max_cube_speed, task.data.xpos[task.cube_body_id].copy()


def test_interactive_pick_place_does_not_launch_cube():
    task = PickPlaceTask(interactive=False, record_path=None)

    max_cube_speed, cube_position = _run_interactive_loop_without_viewer(task)

    assert max_cube_speed < 20.0
    assert float(cube_position[2]) < 1.0


def test_interactive_pick_place_advances_past_hover_pick():
    task = PickPlaceTask(interactive=False, record_path=None)

    for _ in range(320):
        task._interactive_tick()

    assert task.failure_reason is None
    assert task.phase_index >= 1


def test_place_descent_waits_for_contact_before_advancing(monkeypatch: pytest.MonkeyPatch):
    task = PickPlaceTask(interactive=False, record_path=None)
    task.interactive_plan = task._build_episode_plan()
    task.phase_index = next(
        index for index, step in enumerate(task.interactive_plan)
        if step["name"] == "descend_place_contact"
    )
    task.phase_tick = int(task.interactive_plan[task.phase_index]["controller_steps"]) - 1
    task.current_target_position = np.asarray(
        task.interactive_plan[task.phase_index]["target_position"],
        dtype=float,
    ).copy()

    monkeypatch.setattr(task, "_cube_touching_table", lambda: False)

    task._interactive_tick()

    assert task.phase_index == next(
        index for index, step in enumerate(task.interactive_plan)
        if step["name"] == "descend_place_contact"
    )
    assert task.failure_reason == "place_contact_not_reached"


def test_place_descent_does_not_advance_on_contact_until_aligned(monkeypatch: pytest.MonkeyPatch):
    task = PickPlaceTask(interactive=False, record_path=None)
    task.interactive_plan = task._build_episode_plan()
    task.phase_index = next(
        index for index, step in enumerate(task.interactive_plan)
        if step["name"] == "descend_place_contact"
    )
    task.phase_tick = 0
    task.current_target_position = np.asarray(
        task.interactive_plan[task.phase_index]["target_position"],
        dtype=float,
    ).copy()

    monkeypatch.setattr(task, "_cube_touching_table", lambda: True)
    monkeypatch.setattr(task, "_cube_xy_distance_to_target", lambda target_position: 0.05, raising=False)

    task._interactive_tick()

    assert task.phase_index == next(
        index for index, step in enumerate(task.interactive_plan)
        if step["name"] == "descend_place_contact"
    )
    assert task.failure_reason is None


def test_release_phase_opens_gripper_gradually():
    task = PickPlaceTask(interactive=False, record_path=None)

    start = task._interpolate_release_opening(0, 40)
    mid = task._interpolate_release_opening(20, 40)
    end = task._interpolate_release_opening(39, 40)

    assert start == pytest.approx(0.0145)
    assert 0.0145 < mid < 0.025
    assert end == pytest.approx(0.025)


def test_contact_descent_target_applies_xy_correction_before_contact():
    task = PickPlaceTask(interactive=False, record_path=None)
    start_position = np.array([0.50, 0.02, 0.28], dtype=float)
    end_position = np.array([0.56, 0.10, 0.14], dtype=float)
    cube_position = np.array([0.52, 0.04, 0.22], dtype=float)

    raw_target = task._interpolate_stage_target(start_position, end_position, 119, 240)
    corrected_target = task._compute_contact_descent_target(
        start_position,
        end_position,
        step_index=119,
        total_steps=240,
        cube_position=cube_position,
        freeze_z=False,
        frozen_z=None,
    )

    assert corrected_target[0] > raw_target[0]
    assert corrected_target[1] > raw_target[1]
    assert corrected_target[2] == pytest.approx(raw_target[2])


def test_contact_descent_target_holds_height_after_contact():
    task = PickPlaceTask(interactive=False, record_path=None)
    start_position = np.array([0.54, 0.08, 0.26], dtype=float)
    end_position = np.array([0.56, 0.10, 0.14], dtype=float)
    cube_position = np.array([0.53, 0.06, 0.16], dtype=float)
    frozen_z = 0.231

    corrected_target = task._compute_contact_descent_target(
        start_position,
        end_position,
        step_index=200,
        total_steps=240,
        cube_position=cube_position,
        freeze_z=True,
        frozen_z=frozen_z,
    )

    assert corrected_target[2] == pytest.approx(frozen_z)
    assert corrected_target[0] > start_position[0]
    assert corrected_target[1] > start_position[1]
