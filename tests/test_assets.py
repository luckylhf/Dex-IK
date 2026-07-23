import re
import unittest
import xml.etree.ElementTree as ElementTree
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION_ROOT = (
    PROJECT_ROOT / "src" / "dex_ik" / "assets" / "dex_description"
)
URDF_PATH = DESCRIPTION_ROOT / "urdf" / "dex.urdf"


class DexAssetTests(unittest.TestCase):
    def test_urdf_uses_dex_names(self):
        root = ElementTree.parse(URDF_PATH).getroot()
        self.assertEqual(root.attrib["name"], "dex")
        self.assertNotIn("tiangong2dex_urdf", URDF_PATH.read_text())

    def test_all_package_resources_exist(self):
        text = URDF_PATH.read_text()
        resources = re.findall(r"package://dex_description/([^\"']+)", text)
        self.assertTrue(resources)
        missing = [
            resource
            for resource in resources
            if not (DESCRIPTION_ROOT / resource).is_file()
        ]
        self.assertEqual(missing, [])
        self.assertTrue((DESCRIPTION_ROOT / "NOTICE").is_file())
        self.assertTrue(
            (
                DESCRIPTION_ROOT
                / "OpenAtom-Open-Hardware-License-1.0.txt"
            ).is_file()
        )

    def test_each_arm_has_seven_revolute_joints_and_tcp(self):
        root = ElementTree.parse(URDF_PATH).getroot()
        joints = {
            joint.attrib["name"]: joint.attrib["type"]
            for joint in root.findall("joint")
        }
        for side in ("l", "r"):
            arm_names = (
                f"shoulder_pitch_{side}_joint",
                f"shoulder_roll_{side}_joint",
                f"shoulder_yaw_{side}_joint",
                f"elbow_pitch_{side}_joint",
                f"elbow_yaw_{side}_joint",
                f"wrist_pitch_{side}_joint",
                f"wrist_roll_{side}_joint",
            )
            self.assertEqual([joints[name] for name in arm_names], ["revolute"] * 7)
        self.assertEqual(joints["left_tcp_joint"], "fixed")
        self.assertEqual(joints["right_tcp_joint"], "fixed")


if __name__ == "__main__":
    unittest.main()
