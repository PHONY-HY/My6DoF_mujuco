"""Standalone MuJoCo tasks for the my6dof project."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC_PATH = ROOT / "src"
if SRC_PATH.exists():
	src_str = str(SRC_PATH)
	if src_str not in sys.path:
		sys.path.insert(0, src_str)
