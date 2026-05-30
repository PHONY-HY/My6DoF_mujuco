import importlib

import pytest


pytest.importorskip("mujoco")


def test_mujoco_stack_imports():
    mujoco = importlib.import_module("mujoco")
    controller_module = importlib.import_module("mujoco_control")
    tasks_module = importlib.import_module("tasks")

    assert mujoco.__name__ == "mujoco"
    assert controller_module.__name__ == "mujoco_control"
    assert tasks_module.__name__ == "tasks"
