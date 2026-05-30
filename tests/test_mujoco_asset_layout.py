from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BASE_XML = ROOT / "mujoco_assets" / "my6dof" / "my6dof_base.xml"


def test_gripper_finger_bodies_are_not_double_offset():
    root = ET.parse(BASE_XML).getroot()
    left_body = root.find(".//body[@name='left_finger']")
    right_body = root.find(".//body[@name='right_finger']")
    left_geom = root.find(".//geom[@name='left_finger_geom']")
    right_geom = root.find(".//geom[@name='right_finger_geom']")

    assert left_body is not None
    assert right_body is not None
    assert left_geom is not None
    assert right_geom is not None

    assert left_body.attrib.get("pos", "0 0 0") == "0 0 0"
    assert right_body.attrib.get("pos", "0 0 0") == "0 0 0"
    assert left_geom.attrib["pos"] == "0 0.022 0"
    assert right_geom.attrib["pos"] == "0 -0.022 0"