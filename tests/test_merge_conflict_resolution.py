from __future__ import annotations

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PYTHON_FILES = [
    ROOT / "tasks" / "recording.py",
    ROOT / "tasks" / "my6dof_follow_target.py",
    ROOT / "tasks" / "my6dof_pick_place.py",
    ROOT / "tasks" / "pick_place_evaluation.py",
    ROOT / "tasks" / "pick_place_plan.py",
    ROOT / "mujoco_control" / "controller.py",
]
XML_FILES = [
    ROOT / "mujoco_assets" / "my6dof" / "my6dof_base.xml",
]
CONFLICT_MARKERS = ("<<<<<<<", "=======", ">>>>>>>")


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_python_sources_have_no_merge_conflict_markers_and_parse() -> None:
    for path in PYTHON_FILES:
        source = _read_text(path)
        assert not any(marker in source for marker in CONFLICT_MARKERS), path
        ast.parse(source, filename=str(path))


def test_xml_sources_have_no_merge_conflict_markers_and_parse() -> None:
    for path in XML_FILES:
        source = _read_text(path)
        assert not any(marker in source for marker in CONFLICT_MARKERS), path
        ET.fromstring(source)
