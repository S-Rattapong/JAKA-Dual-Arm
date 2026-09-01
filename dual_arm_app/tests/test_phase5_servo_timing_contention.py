"""Offline/source contracts for the Phase-5 timing and SDK contention pass."""

from pathlib import Path
import unittest

import yaml

from dual_arm_app.backend.phase5_motion_quality import (
    normalize_phase5_motion_settings,
)
from dual_arm_app.backend.phase5_execution_feedback import (
    normalize_driver_execution_status,
)


ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"
EXECUTE_SRV = ROOT / "src/jaka_ros2/src/jaka_msgs/srv/ExecuteJointTrajectory.srv"
STATUS_SRV = ROOT / "src/jaka_ros2/src/jaka_msgs/srv/GetExecutionStatus.srv"


class Phase5ServoTimingSourceContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.driver = DRIVER.read_text()
        cls.callback = cls.driver.split(
            "bool execute_joint_trajectory_callback", 1
        )[1].split("bool stop_move_callback", 1)[0]
        cls.cleanup = cls.driver.split(
            "static Phase5SdkCleanupResult cleanup_phase5_sdk_control_window", 1
        )[1].split("static void finish_phase5_trajectory", 1)[0]
        cls.telemetry = cls.driver.split("void get_conn_scoket_state()", 1)[1].split(
            "int main(", 1
        )[0]

    def test_filter_is_configured_before_enable_and_acceptance(self):
        apply_index = self.callback.index("apply_phase5_filter(filter_config)")
        enable_index = self.callback.index("robot.servo_move_enable(TRUE)")
        accepted_index = self.callback.index("response->accepted = true")
        self.assertLess(apply_index, enable_index)
        self.assertLess(enable_index, accepted_index)
        self.assertIn("robot.servo_speed_foresight", self.driver)
        self.assertIn("LEGACY_FORESIGHT", self.driver)

    def test_cleanup_disables_before_restoring_baseline_outside_servo_mode(self):
        disable_index = self.cleanup.index("robot.servo_move_enable(FALSE)")
        restore_index = self.cleanup.index(
            "restore_phase5_legacy_foresight_baseline()"
        )
        self.assertLess(disable_index, restore_index)
        self.assertIn("result.disable_ret == 0", self.cleanup)
        self.assertNotIn("servo_move_use_none_filter", self.cleanup)
        self.assertLess(
            restore_index,
            self.cleanup.index("phase5_sdk_control_window_active.store(false)"),
        )

    def test_active_stream_branch_makes_no_sdk_call_and_freezes_real_telemetry(self):
        exclusive_branch = self.telemetry.split(
            "if (phase5_sdk_control_window_active.load())", 1
        )[1].split("continue;", 1)[0]
        self.assertNotIn("robot.", exclusive_branch)
        for query in (
            "get_robot_status",
            "get_joint_position",
            "get_tcp_position",
            "is_in_pos",
            "is_in_collision",
            "is_in_drag_mode",
            "is_in_estop",
            "get_robot_status_simple",
            "get_program_state",
        ):
            self.assertNotIn(query, exclusive_branch)
        self.assertNotIn("publish(", exclusive_branch)
        self.assertIn("phase5_telemetry_suppressed_poll_count.fetch_add", exclusive_branch)
        self.assertIn("last genuinely observed ROS telemetry", exclusive_branch)
        self.assertIn("unique_lock<mutex> telemetry_gate", self.telemetry)

    def test_cleanup_restores_normal_telemetry_only_after_sdk_cleanup(self):
        clear_index = self.cleanup.index(
            "phase5_sdk_control_window_active.store(false)"
        )
        self.assertGreater(clear_index, self.cleanup.index("robot.servo_move_enable(FALSE)"))
        self.assertGreater(
            clear_index,
            self.cleanup.index("restore_phase5_legacy_foresight_baseline()"),
        )
        normal_branch = self.telemetry.split("continue;", 1)[1]
        self.assertIn("robot.get_joint_position", normal_branch)
        self.assertIn("tool_position_callback", normal_branch)
        self.assertIn("joint_position_callback", normal_branch)
        self.assertIn("robot_states_callback", normal_branch)

    def test_worker_keeps_absolute_steady_pacing_and_all_samples(self):
        worker = self.driver.split(
            "static void execute_phase5_joint_trajectory_worker", 1
        )[1].split("bool execute_joint_trajectory_callback", 1)[0]
        self.assertIn("steady_start + sample_offset", worker)
        self.assertIn("this_thread::sleep_until", worker)
        self.assertIn("sample_index < resampled.size()", worker)
        self.assertIn("MoveMode::ABS, static_cast<int>(servo_step_num)", worker)
        self.assertNotIn("pthread_", worker)
        self.assertNotIn("sched_", worker)

    def test_control_window_and_fail_closed_stream_guard_precede_sdk_setup(self):
        self.assertLess(
            self.callback.index("validate_stream_command_velocity"),
            self.callback.index("phase5_sdk_control_window_active.store(true)"),
        )
        self.assertLess(
            self.callback.index("phase5_sdk_control_window_active.store(true)"),
            self.callback.index("apply_phase5_filter(filter_config)"),
        )
        self.assertIn("request->servo_step_num < 1U", self.callback)
        self.assertIn("request->servo_step_num > 4U", self.callback)

    def test_stop_cancel_is_rechecked_before_servo_enable(self):
        cancel_index = self.callback.index("cancelled_before_servo_enable")
        enable_index = self.callback.index("robot.servo_move_enable(TRUE)")
        self.assertLess(cancel_index, enable_index)
        pre_enable = self.callback[cancel_index:enable_index]
        self.assertIn("phase5_trajectory_cancel_requested.load()", pre_enable)
        self.assertIn("cleanup_phase5_sdk_control_window(false)", pre_enable)

    def test_servo_enable_failure_cleanup_does_not_assume_servo_mode_entered(self):
        enable_index = self.callback.index("const int enable_ret = robot.servo_move_enable(TRUE)")
        after_enable = self.callback[enable_index:]
        failure_block = after_enable.split("bool cancelled_during_preparation", 1)[0]
        self.assertIn("if (enable_ret != 0)", failure_block)
        self.assertIn("cleanup_phase5_sdk_control_window(false)", failure_block)
        self.assertNotIn("cleanup_phase5_sdk_control_window(true)", failure_block)

    def test_status_and_backend_contract_include_new_diagnostics(self):
        status = STATUS_SRV.read_text()
        backend = (
            ROOT / "dual_arm_app/backend/phase5_execution_feedback.py"
        ).read_text()
        ui = (ROOT / "dual_arm_app/web/digital_twin_phase5_execution.js").read_text()
        for field in (
            "mean_servo_j_call_duration_ms",
            "p95_servo_j_call_duration_ms",
            "p99_servo_j_call_duration_ms",
            "max_servo_j_call_duration_ms",
            "servo_j_overrun_count",
            "first_dispatch_unix_ns",
            "first_servo_return_unix_ns",
            "last_dispatch_unix_ns",
            "last_servo_return_unix_ns",
            "servo_step_num",
            "command_period_ms",
            "telemetry_mode",
            "telemetry_suppressed_poll_count",
            "stream_guard_limit_rad_s",
            "stream_guard_observed_max_rad_s",
        ):
            self.assertIn(field, status)
            self.assertIn(field, self.driver)
        self.assertIn('"servo_j_call_duration"', backend)
        self.assertIn('"servo_stream"', backend)
        self.assertIn("servo_j call samples", ui)
        self.assertIn("overruns=", ui)
        self.assertIn("telemetry=", ui)
        self.assertIn("first dispatch/return offset", ui)
        self.assertIn("last dispatch/return offset", ui)
        self.assertIn("telemetry polls suppressed", ui)

    def test_current_driver_uses_exclusive_frozen_telemetry_mode(self):
        self.assertIn('"PHASE5_SDK_EXCLUSIVE_TELEMETRY_FROZEN"', self.callback)
        self.assertNotIn('"PHASE5_PORT10004_SNAPSHOT"', self.callback)
        self.assertLess(
            self.callback.index("phase5_telemetry_suppressed_poll_count.store(0U)"),
            self.callback.index("phase5_sdk_control_window_active.store(true)"),
        )

    def test_jaka_sdk_rpath_is_origin_relative_for_symlink_install(self):
        cmake = (
            ROOT / "src/jaka_ros2/src/jaka_driver/CMakeLists.txt"
        ).read_text()
        self.assertIn("JAKA_SDK_BUILD_RUNTIME_DIR ${CMAKE_BINARY_DIR}/..", cmake)
        self.assertIn("file(COPY ${JAKA_SDK_SOURCE_LIBRARY}", cmake)
        self.assertIn('BUILD_RPATH "$ORIGIN/.."', cmake)
        self.assertIn("BUILD_RPATH_USE_ORIGIN TRUE", cmake)
        self.assertIn('INSTALL_RPATH "$ORIGIN/.."', cmake)
        self.assertNotIn('BUILD_RPATH "${LIBRARY_DIR}"', cmake)

    def test_filter_and_step_request_schema_mapping_is_stable(self):
        request = EXECUTE_SRV.read_text()
        self.assertIn("uint8 SERVO_FILTER_NONE=0", request)
        self.assertIn("uint8 SERVO_FILTER_LPF=1", request)
        self.assertIn("uint8 SERVO_FILTER_NLF=2", request)
        self.assertIn("uint8 SERVO_FILTER_LEGACY_FORESIGHT=3", request)
        self.assertIn("int32 servo_filter_legacy_max_buf", request)
        self.assertIn("float64 servo_filter_legacy_kp", request)
        self.assertIn("uint8 servo_step_num", request)


class Phase5ServoTimingConfigurationContracts(unittest.TestCase):
    def test_exclusive_telemetry_diagnostics_normalize_without_claiming_samples(self):
        normalized = normalize_driver_execution_status(
            {
                "valid": True,
                "ret": 1,
                "state": "RUNNING",
                "trajectory_id": "offline-contract",
                "start_time_unix_ns": 1,
                "terminal_time_unix_ns": 0,
                "duration_s": 8.0,
                "elapsed_s": 4.0,
                "progress_0_to_1": 0.5,
                "sample_index": 250,
                "sample_count": 501,
                "servo_step_num": 2,
                "command_period_ms": 16.0,
                "telemetry_mode": "PHASE5_SDK_EXCLUSIVE_TELEMETRY_FROZEN",
                "telemetry_suppressed_poll_count": 80,
            },
            side="right",
        )
        self.assertTrue(normalized["valid"])
        self.assertEqual(
            normalized["servo_stream"]["telemetry_mode"],
            "PHASE5_SDK_EXCLUSIVE_TELEMETRY_FROZEN",
        )
        self.assertEqual(
            normalized["servo_stream"]["telemetry_suppressed_poll_count"], 80
        )

    def test_code_fallback_is_8ms_but_current_acceptance_config_is_24ms(self):
        settings = normalize_phase5_motion_settings({})
        self.assertEqual(settings["servo_step_num"], 1)
        self.assertEqual(settings["servo_filter"]["mode"], "LEGACY_FORESIGHT")
        self.assertEqual(settings["servo_filter"]["legacy_max_buf"], 15)
        self.assertEqual(settings["servo_filter"]["legacy_kp"], 0.03)

        config = yaml.safe_load(
            (ROOT / "dual_arm_app/config/robots.yaml").read_text()
        )["motion"]
        self.assertEqual(config["phase5_servo_step_num"], 3)
        self.assertEqual(config["phase5_servo_filter"]["mode"], "LEGACY_FORESIGHT")
        self.assertEqual(config["phase5_servo_filter"]["max_buf"], 15)
        self.assertEqual(config["phase5_servo_filter"]["kp"], 0.03)


if __name__ == "__main__":
    unittest.main()
