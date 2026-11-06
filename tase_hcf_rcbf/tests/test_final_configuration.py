#!/usr/bin/env python3
"""验证最终六组实验配置及其 launch 文件引用关系。"""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parent


class FinalExperimentConfigurationTest(unittest.TestCase):
    def test_final_hcf_parameters(self):
        config = yaml.safe_load((PKG / "config/hcf_ke3_eta15_eta150_p2_7.yaml").read_text())
        p = config["tase_comparison"]
        self.assertEqual(p["ke"], 3.0)
        self.assertEqual(p["eta1"], 15.0)
        self.assertEqual(p["eta2"], 150.0)
        self.assertEqual(p["p1"], 7.0)
        self.assertEqual(p["p2"], 7.0)

    def test_final_rcbf_parameters(self):
        config = yaml.safe_load((PKG / "config/rcbf_b30_k50.yaml").read_text())
        p = config["tase_comparison"]
        self.assertEqual(p["rcbf_k1"], 50.0)
        self.assertEqual(p["rcbf_k2"], 50.0)
        self.assertEqual(p["kp"], 1600.0)
        self.assertEqual(p["kd"], 320.0)

    def test_launch_load_order_and_record_topics(self):
        tree = ET.parse(PKG / "launch/disturbance_comparison.launch").getroot()
        loaded = [node.attrib["file"] for node in tree.findall("rosparam") if "file" in node.attrib]
        self.assertEqual(len(loaded), 2)
        self.assertIn("$(find tase_hcf_rcbf)/config/controller.yaml", loaded[0])
        self.assertEqual(loaded[1], "$(arg tuning_file)")

        recorder = next(node for node in tree.findall(".//node") if node.attrib.get("name") == "tase_record")
        topics = recorder.attrib["args"]
        self.assertIn("/tase_comparison/sample", topics)
        self.assertIn("/tase_comparison/configuration", topics)
        self.assertIn("/franka_state_controller/franka_states", topics)
        self.assertNotIn("rosout", topics)

    def test_hardware_wrappers_select_final_configs(self):
        hcf = (ROOT / "run_moving_hcf_p2_7.sh").read_text()
        rcbf = (ROOT / "run_moving_rcbf_b30_k50.sh").read_text()
        self.assertIn("hcf_ke3_eta15_eta150_p2_7.yaml", hcf)
        self.assertIn("base_budget:=0.0", hcf)
        self.assertIn("rcbf_b30_k50.yaml", rcbf)
        self.assertIn("base_budget:=28.0", rcbf)
        self.assertIn("mode:=HCF", hcf)
        self.assertIn("mode:=RCBF", rcbf)


if __name__ == "__main__":
    unittest.main()
