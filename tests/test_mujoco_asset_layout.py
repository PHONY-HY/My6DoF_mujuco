from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BASE_XML = ROOT / "mujoco_assets" / "my6dof" / "my6dof_base.xml"


<<<<<<< HEAD
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
=======
def test_gripper_finger_bodies_are_side_mounted_for_vertical_grasp():
    root = ET.parse(BASE_XML).getroot()
    side_mount = root.find(".//body[@name='tool_mount']")
    finger_carriage = root.find(".//body[@name='finger_carriage']")
    left_body = root.find(".//body[@name='left_finger']")
    right_body = root.find(".//body[@name='right_finger']")
    housing = root.find(".//geom[@name='gripper_housing_geom']")
    left_support = root.find(".//geom[@name='left_finger_support_geom']")
    right_support = root.find(".//geom[@name='right_finger_support_geom']")
    left_geom = root.find(".//geom[@name='left_finger_geom']")
    right_geom = root.find(".//geom[@name='right_finger_geom']")

    assert side_mount is not None
    assert finger_carriage is not None
    assert left_body is not None
    assert right_body is not None
    assert housing is not None
    assert left_support is not None
    assert right_support is not None
    assert left_geom is not None
    assert right_geom is not None

    assert side_mount.attrib.get("pos", "") == "0 0 0"
    assert side_mount.attrib.get("quat", "") == "0.70710678 0 0.70710678 0"
    assert finger_carriage.attrib.get("pos", "") == "0 0 -0.08"
    assert left_body.attrib.get("pos", "0 0 0") == "0 0 0"
    assert right_body.attrib.get("pos", "0 0 0") == "0 0 0"
    assert housing.attrib["pos"] == "0 0 -0.032"
    assert housing.attrib["pos"] == "0 0 -0.032"
    assert left_support.attrib["pos"] == "0 0.018 0.025"
    assert right_support.attrib["pos"] == "0 -0.018 0.025"
    assert left_geom.attrib["pos"] == "0 0.018 0"
    assert right_geom.attrib["pos"] == "0 -0.018 0"
>>>>>>> 1f097dd (Update .gitignore and restage clean files)


def test_follow_target_body_is_mocap_target():
    follow_scene = ROOT / "mujoco_assets" / "my6dof" / "scene_my6dof_follow_target.xml"
    root = ET.parse(follow_scene).getroot()
    target_body = root.find(".//body[@name='follow_target']")
<<<<<<< HEAD

    assert target_body is not None
    assert target_body.attrib.get("mocap") == "true"
=======
    free_joint = root.find(".//body[@name='follow_target']/joint[@type='free']")

    assert target_body is not None
    assert target_body.attrib.get("mocap") == "true"
    assert free_joint is None


def test_grasp_site_exists_for_visual_target_alignment():
    root = ET.parse(BASE_XML).getroot()
    grasp_site = root.find(".//site[@name='grasp_site']")
    finger_carriage = root.find(".//body[@name='finger_carriage']")

    assert grasp_site is not None
    assert finger_carriage is not None
    assert finger_carriage.attrib.get("pos", "") == "0 0 -0.08"
    assert grasp_site.attrib.get("pos") == "0 0 0"


def test_position_actuator_defaults_use_nontrivial_gains():
    root = ET.parse(BASE_XML).getroot()
    position_default = root.find("./default/position")

    assert position_default is not None
    assert float(position_default.attrib["kp"]) >= 1200.0
    assert float(position_default.attrib["kv"]) > 0.0


def test_gripper_support_geoms_are_visual_only():
    root = ET.parse(BASE_XML).getroot()
    left_support = root.find(".//geom[@name='left_finger_support_geom']")
    right_support = root.find(".//geom[@name='right_finger_support_geom']")

    assert left_support is not None
    assert right_support is not None
    assert left_support.attrib.get("contype") == "0"
    assert left_support.attrib.get("conaffinity") == "0"
    assert right_support.attrib.get("contype") == "0"
    assert right_support.attrib.get("conaffinity") == "0"


def test_gripper_support_geoms_leave_visual_clearance_under_link6():
    root = ET.parse(BASE_XML).getroot()
    finger_carriage = root.find(".//body[@name='finger_carriage']")
    left_support = root.find(".//geom[@name='left_finger_support_geom']")
    housing = root.find(".//geom[@name='gripper_housing_geom']")

    assert finger_carriage is not None
    assert left_support is not None
    assert housing is not None

    carriage_pos_z = float(finger_carriage.attrib["pos"].split()[2])
    support_pos_z = float(left_support.attrib["pos"].split()[2])
    support_half_z = float(left_support.attrib["size"].split()[2])
    housing_pos_z = float(housing.attrib["pos"].split()[2])
    housing_half_z = float(housing.attrib["size"].split()[2])

    support_top = carriage_pos_z + support_pos_z + support_half_z
    housing_top = housing_pos_z + housing_half_z

    assert support_top < housing_top


def test_grasp_site_is_far_enough_below_and_ahead_of_wrist_tip_for_side_grasp():
    root = ET.parse(BASE_XML).getroot()
    grasp_site = root.find(".//site[@name='grasp_site']")
    finger_carriage = root.find(".//body[@name='finger_carriage']")

    assert grasp_site is not None
    assert finger_carriage is not None

    carriage_pos = [float(value) for value in finger_carriage.attrib["pos"].split()]
    grasp_site_pos = [float(value) for value in grasp_site.attrib["pos"].split()]

    assert carriage_pos[0] == 0.0
    assert abs(carriage_pos[2]) >= 0.08
    assert grasp_site_pos == [0.0, 0.0, 0.0]


def test_wrist_geoms_collide_with_cube_without_reenabling_broad_self_collision():
    root = ET.parse(BASE_XML).getroot()
    link5_geom = root.find(".//geom[@name='link5_geom']")
    link6_geom = root.find(".//geom[@name='link6_geom']")

    assert link5_geom is not None
    assert link6_geom is not None
    assert link5_geom.attrib.get("contype") == "8"
    assert link5_geom.attrib.get("conaffinity") == "2"
    assert link6_geom.attrib.get("contype") == "8"
    assert link6_geom.attrib.get("conaffinity") == "2"
>>>>>>> 1f097dd (Update .gitignore and restage clean files)
