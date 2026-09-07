"""Static contract: Digital Twin may receive port10000, but Driver never writes it."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"
CMAKE = ROOT / "src/jaka_ros2/src/jaka_driver/CMakeLists.txt"
CONFIG = ROOT / "dual_arm_app/config/robots.yaml"

class DriverPort10000RateContractTests(unittest.TestCase):
    def test_driver_does_not_configure_controller_port10000(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertNotIn("port10000_rate_config", source)
        self.assertNotIn("port10000_feedback_period_ms", source)
        self.assertNotIn("configure_port10000_feedback_rate", source)
        self.assertIn("robot.login_in(robot_ip.c_str(), false)", source)
        self.assertIn("robot.set_status_data_update_time_interval(100)", source)
        self.assertIn("phase5_sdk_control_window_active", source)

    def test_rate_helper_is_not_linked_into_production_driver(self):
        source = CMAKE.read_text(encoding="utf-8")
        driver_target = source.split("add_executable(jaka_driver", 1)[1].split(")", 1)[0]
        self.assertNotIn("port10000_rate_config.cpp", driver_target)
        self.assertNotIn("port10000_rate_config_test", source)

    def test_backend_receive_only_feedback_is_enabled_by_configuration(self):
        source = CONFIG.read_text(encoding="utf-8")
        block = source.split("actual_feedback:", 1)[1].split("motion:", 1)[0]
        self.assertIn("port10000:", block)
        self.assertIn("enabled: true", block)
        self.assertIn("expected_period_ms: 100", block)
        self.assertIn("freshness_threshold_ms: 400", block)

if __name__ == "__main__":
    unittest.main()
