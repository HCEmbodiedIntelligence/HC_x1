import unittest
from pathlib import Path

import yaml


class ProductBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]

    def test_required_sources_and_vendor_runtime_are_local(self):
        required = (
            "src/hc_tj/hc_tj_marvin_bridge_cpp/package.xml",
            "src/hc_tj/hc_tj_body_bridge/package.xml",
            "src/hc_tj/hc_tj_camera_bridge/package.xml",
            "src/omnihand_o10_rs485_bridge/package.xml",
            "src/hc_d405_stereo_camera/package.xml",
            "src/io_msgs2/package.xml",
            "src/camera_driver/package.xml",
            "src/camera_msgs/package.xml",
            "vendor/marvin/contrlSDK/libMarvinSDK.so",
            "vendor/marvin/kinematicsSDK/libKine.so",
            "vendor/marvin/CommonConfig/ccs_m6_40.MvKDCfg",
            "vendor/omnihand/lib/libomnihand.so",
            "vendor/angstrong/libs/libAngstrongCameraSdk.so",
            "vendor/berxel/libs/libBerxelHawk.so",
            "config/camera/camera_config.yaml",
            "config/camera/calibration.yaml",
        )
        for relative in required:
            self.assertTrue((self.root / relative).is_file(), relative)

    def test_runtime_sources_do_not_reference_other_source_workspaces(self):
        files = [self.root / "start.sh", self.root / "build.sh"]
        files.extend((self.root / "hc_x1").glob("*.py"))
        files.extend((self.root / "src").rglob("*.sh"))
        files.extend((self.root / "src").rglob("CMakeLists.txt"))
        text = "\n".join(path.read_text(encoding="utf-8") for path in files)
        self.assertNotIn("hc_io_suit", text)
        self.assertNotIn("/home/niic", text)

    def test_smoothing_is_owned_by_x1_hardware_project(self):
        config = yaml.safe_load(
            (self.root / "config/x1_hardware.yaml").read_text(encoding="utf-8")
        )["control"]["command_smoothing"]
        self.assertEqual(config["input_topic"], "/hc_teleop/joint_cmd")
        self.assertEqual(config["output_topic"], "/io_teleop/joint_cmd")
        self.assertEqual(config["rate_hz"], 100.0)
        self.assertEqual(config["time_constant"], 0.04)
        self.assertEqual(config["max_acceleration"], 12.0)

    def test_control_and_camera_adapters_are_process_isolated(self):
        control = (self.root / "hc_x1/control_adapter.py").read_text(
            encoding="utf-8"
        )
        camera = (self.root / "hc_x1/camera_adapter.py").read_text(
            encoding="utf-8"
        )
        launcher = (self.root / "start.sh").read_text(encoding="utf-8")

        self.assertIn('"/io_teleop/joint_states"', control)
        self.assertIn('"/hc_teleop/joint_states"', control)
        self.assertIn("depth=1", control)
        self.assertNotIn("CompressedImage", control)
        self.assertNotIn("/cameras/", control)
        self.assertNotIn("/vrdata", control)

        self.assertIn("CompressedImage", camera)
        self.assertNotIn('"/io_teleop/joint_states"', camera)
        self.assertNotIn("sensor_msgs.msg import Image", camera)
        self.assertNotIn("cv2", camera)

        self.assertIn("hc_x1/control_adapter.py", launcher)
        self.assertIn("hc_x1/camera_adapter.py", launcher)


if __name__ == "__main__":
    unittest.main()
