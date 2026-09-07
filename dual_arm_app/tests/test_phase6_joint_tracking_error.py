"""P6.1 read-only joint tracking error regressions."""

from __future__ import annotations

from array import array
from pathlib import Path
from types import SimpleNamespace

from dual_arm_app.backend.phase5_execution_feedback import (
    build_phase5_execution_feedback,
    normalize_driver_execution_status,
)
from dual_arm_app.backend.phase6_joint_tracking import compute_joint_tracking_error


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
DRIVER = ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"
SERVICE = ROOT / "src/jaka_ros2/src/jaka_msgs/srv/GetExecutionStatus.srv"
WEB = ROOT / "dual_arm_app/web"


def raw_driver(side: str, commanded: list[float], *, state: str = "RUNNING"):
    del side
    return SimpleNamespace(
        valid=True, ret=1, message="memory snapshot", trajectory_id="p6-track",
        state=state, active=state in {"ARMED", "RUNNING"},
        start_time_unix_ns=1_900_000_000_000_000_000,
        first_dispatch_unix_ns=1_900_000_000_100_000_000,
        first_servo_return_unix_ns=1_900_000_000_101_000_000,
        last_dispatch_unix_ns=1_900_000_001_000_000_000,
        last_servo_return_unix_ns=1_900_000_001_001_000_000,
        terminal_time_unix_ns=0, duration_s=2.0, elapsed_s=1.0,
        progress_0_to_1=0.5, sample_index=41, sample_count=84,
        commanded_sample_valid=True, commanded_time_from_start_s=0.984,
        commanded_joints_rad=commanded, terminal_reason="",
        servo_step_num=3, command_period_ms=24.0,
        telemetry_mode="PHASE5_SDK_EXCLUSIVE_TELEMETRY_FROZEN",
    )


def tracking_fixture():
    left_command = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    right_command = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]
    feedback = build_phase5_execution_feedback(
        expected_trajectory_id="p6-track",
        left_response=raw_driver("left", left_command),
        right_response=raw_driver("right", right_command),
    )
    actual = {
        "ok": True,
        "source": "jaka_port10000_actual_feedback",
        "left": {"valid": True, "joint": [v + 0.01 for v in left_command], "age_ms": 25, "received_at_ms": 1000},
        "right": {"valid": True, "joint": [v - 0.02 for v in right_command], "age_ms": 30, "received_at_ms": 995},
    }
    return feedback, actual


def test_ros2_float64_array_command_vector_is_accepted():
    normalized = normalize_driver_execution_status(
        raw_driver("left", array("d", [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])),
        side="left",
    )
    assert normalized["valid"] is True
    assert normalized["commanded_sample"]["joints_rad"] == [
        0.1, -0.2, 0.3, -0.4, 0.5, -0.6
    ]


def test_driver_status_exposes_exact_latest_successful_command_sample():
    normalized = normalize_driver_execution_status(
        raw_driver("left", [0.1] * 6), side="left"
    )
    assert normalized["commanded_sample"] == {
        "valid": True,
        "sample_index": 41,
        "time_from_start_s": 0.984,
        "joints_rad": [0.1] * 6,
        "source": "LATEST SUCCESSFUL PHASE5 SERVO_J COMMAND — DRIVER MEMORY",
    }


def test_p6_1_computes_signed_per_joint_and_combined_max_error():
    feedback, actual = tracking_fixture()
    result = compute_joint_tracking_error(feedback, actual)
    assert result["status"] == "AVAILABLE"
    assert result["monitoring_only"] is True
    assert result["threshold_applied"] is False
    assert result["coordinated_stop_enabled"] is False
    assert result["actual_source"] == "jaka_port10000_actual_feedback"
    assert all(abs(value - 0.01) < 1e-12 for value in result["left"]["error_rad"])
    assert all(abs(value + 0.02) < 1e-12 for value in result["right"]["error_rad"])
    assert abs(result["left"]["max_abs_error_rad"] - 0.01) < 1e-12
    assert abs(result["right"]["max_abs_error_rad"] - 0.02) < 1e-12
    assert abs(result["combined"]["max_abs_error_rad"] - 0.02) < 1e-12
    assert result["combined"]["max_error_side"] == "right"


def test_p6_1_fails_open_for_observation_when_command_or_actual_is_missing():
    feedback, actual = tracking_fixture()
    feedback["drivers"]["left"]["commanded_sample"]["valid"] = False
    actual["right"]["valid"] = False
    result = compute_joint_tracking_error(feedback, actual)
    assert result["status"] == "NOT_EVALUATED"
    assert result["left"]["reason"] == "COMMAND_SAMPLE_UNAVAILABLE"
    assert result["right"]["reason"] == "ACTUAL_JOINT_FEEDBACK_UNAVAILABLE"
    assert result["coordinated_stop_enabled"] is False


def test_service_and_driver_are_read_only_additive_command_snapshot():
    service = SERVICE.read_text(encoding="utf-8")
    driver = DRIVER.read_text(encoding="utf-8")
    for field in (
        "bool commanded_sample_valid",
        "float64 commanded_time_from_start_s",
        "float64[] commanded_joints_rad",
    ):
        assert field in service
    callback = driver.split("bool get_execution_status_callback", 1)[1].split(
        "struct Phase5ServoFilterConfig", 1
    )[0]
    assert "commanded_joints_rad.assign" in callback
    assert "robot." not in callback
    servo_check = driver.index("if (servo_ret != 0)")
    snapshot_valid = driver.index(
        "phase5_execution_status.commanded_sample_valid = true", servo_check
    )
    snapshot_pose = driver.index(
        "phase5_execution_status.commanded_joints_rad = sample.positions_rad", servo_check
    )
    progress_update = driver.index("phase5_execution_status.elapsed_s = min(", snapshot_pose)
    assert servo_check < snapshot_valid <= snapshot_pose < progress_update


def test_backend_uses_visual_actual_cache_for_p6_without_changing_phase5_actual_authority():
    backend = BACKEND.read_text(encoding="utf-8")
    body = backend.split("def phase5_execution_state", 1)[1].split(
        "def phase5_driver_execution_feedback", 1
    )[0]
    assert 'actual_joints = self.digital_twin_ros_joint_status()' in body
    assert 'result["actual_joints"] = actual_joints' in body
    assert "monitoring_actual_joints = self.digital_twin_joint_status()" in body
    assert "compute_joint_tracking_error" in body


def test_p6_1_ui_is_read_only_and_reuses_existing_phase5_status_poll():
    html = (WEB / "index.html").read_text(encoding="utf-8")
    controller = (WEB / "digital_twin_phase5_execution.js").read_text(encoding="utf-8")
    for element_id in (
        "digitalTwinPhase6Monitoring",
        "digitalTwinP6JointTrackingStatus",
        "digitalTwinP6JointTrackingSource",
        "digitalTwinP6JointTrackingCombinedMax",
        "digitalTwinP6JointTrackingLeftVector",
        "digitalTwinP6JointTrackingRightVector",
    ):
        assert f'id="{element_id}"' in html
    assert "P6.1 Joint Tracking Error — monitoring only" in html
    assert "renderPhase6JointTracking(payload.phase6_joint_tracking)" in controller
    assert "STATUS_POLL_INTERVAL_MS = 100" in controller
    render = controller.split("function renderPhase6JointTracking", 1)[1].split(
        "function formatQuality", 1
    )[0]
    assert "requestJson(" not in render
    assert "addEventListener" not in render


def test_p6_1_module_is_explicitly_observation_only():
    source = (ROOT / "dual_arm_app/backend/phase6_joint_tracking.py").read_text(
        encoding="utf-8"
    )
    assert '"monitoring_only": True' in source
    assert '"threshold_applied": False' in source
    assert '"coordinated_stop_enabled": False' in source
    assert "P6.6 OWNS TIMING/PHASE ERROR" in source
