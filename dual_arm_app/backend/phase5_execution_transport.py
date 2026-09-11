"""Phase-5 dual-arm execution transports.

The original P5.4-P5.7 single-point bridge remains for compatibility. P5.10
uses the isolated absolute-start bridge below: both existing driver sessions
receive complete trajectories carrying the exact same future host wall clock.
No backend SDK session is opened and no hard real-time synchronization is
claimed.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable


TRANSPORT_VERSION = "PHASE5_EXECUTION_TRANSPORT_V1"
COMMON_START_MODE = "HOST_BARRIER_COMMON_RELEASE"
COMMON_START_SEMANTIC = (
    "BEST-EFFORT HOST COMMON RELEASE — DISPATCH SKEW IS MEASURED; "
    "CONTROLLER HARD-REAL-TIME START SYNCHRONIZATION IS NOT GUARANTEED"
)

ABSOLUTE_START_TRANSPORT_VERSION = "PHASE5_ABSOLUTE_START_TRANSPORT_V1"
ABSOLUTE_COMMON_START_MODE = "HOST_TIMED_COMMON_ABSOLUTE_START"
ABSOLUTE_COMMON_START_SEMANTIC = (
    "HOST-TIMED COMMON ABSOLUTE START — BOTH DRIVERS RECEIVE THE SAME "
    "start_time_unix_ns; THIS IS NOT HARD REAL-TIME OR CONTROLLER-SYNCHRONIZED"
)
MINIMUM_ACCEPTANCE_MARGIN_NS = 200_000_000
PHASE5_DRIVER_INTERPOLATION_MODE = "LINEAR_JOINT_SPACE"
PHASE5_DRIVER_CONTRACT_MARKER = "PHASE5_DRIVER_CONTRACT=LINEAR_JOINT_SPACE_V2"


class Phase5ExecutionTransportError(RuntimeError):
    """Raised when a paired dispatch cannot be released safely."""
@dataclass(frozen=True)
class CommonReleaseReceipt:
    left_dispatch_ns: int
    right_dispatch_ns: int
    dispatch_skew_ns: int
    stop_generation: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "left_dispatch_ns": self.left_dispatch_ns,
            "right_dispatch_ns": self.right_dispatch_ns,
            "dispatch_skew_ns": self.dispatch_skew_ns,
            "dispatch_skew_ms": self.dispatch_skew_ns / 1_000_000.0,
            "stop_generation": self.stop_generation,
            "common_start_mode": COMMON_START_MODE,
            "semantic": COMMON_START_SEMANTIC,
        }


class Phase5ExecutionTransport:
    """Injected-client bridge over the already-running left/right drivers."""
    def __init__(
        self,
        *,
        left_move_client: Any,
        right_move_client: Any,
        request_factory: Callable[[Any, float, float], Any],
        stop_generation_getter: Callable[[], int],
        left_service_name: str,
        right_service_name: str,
    ) -> None:
        self._left_move_client = left_move_client
        self._right_move_client = right_move_client
        self._request_factory = request_factory
        self._stop_generation_getter = stop_generation_getter
        self._left_service_name = str(left_service_name)
        self._right_service_name = str(right_service_name)

    @staticmethod
    def _client_ready(client: Any) -> bool:
        checker = getattr(client, "service_is_ready", None)
        return bool(checker()) if callable(checker) else False
    def inspection_state(self) -> dict[str, Any]:
        left_ready = self._client_ready(self._left_move_client)
        right_ready = self._client_ready(self._right_move_client)
        return {
            "transport_version": TRANSPORT_VERSION,
            "p5_4_common_start": {
                "mode": COMMON_START_MODE,
                "host_barrier_release": True,
                "dispatch_skew_measured": True,
                "hard_realtime_controller_sync": False,
                "semantic": COMMON_START_SEMANTIC,
            },
            "p5_5_robot_connection": {
                "uses_existing_driver_sessions": True,
                "new_sdk_session": False,
                "left_joint_move_service": self._left_service_name,
                "right_joint_move_service": self._right_service_name,
                "left_ready": left_ready,
                "right_ready": right_ready,
                "both_ready": left_ready and right_ready,
            },
            "p5_6_stop_home": {
                "stop_generation_guard": True,
                "stop_path": "EXISTING request_stop_all -> /stop_move -> motion_abort",
                "home_path": "EXISTING home -> move_both_joint -> /joint_move",
                "new_stop_or_home_stack": False,
            },
            "p5_7_execution_path": {
                "request_factory": "EXISTING make_joint_move_request",
                "transport": "EXISTING ROS jaka_driver /joint_move services",
                "driver_joint_move_nonblocking": True,
                "direct_sdk_calls_from_backend": False,
                "timed_trajectory_streaming": (
                    "NOT IMPLEMENTED IN P5.4-P5.7 — TRANSPORT FOR THE FULL "
                    "COMMON TIMELINE IS FINALIZED/USED AT P5.10"
                ),
            },
        }

    def _assert_release_preconditions(self, expected_stop_generation: int) -> None:
        current = int(self._stop_generation_getter())
        if current != int(expected_stop_generation):
            raise Phase5ExecutionTransportError(
                "STOP_GENERATION_CHANGED_BEFORE_COMMON_RELEASE"
            )
        if not self._client_ready(self._left_move_client):
            raise Phase5ExecutionTransportError("LEFT_JOINT_MOVE_SERVICE_NOT_READY")
        if not self._client_ready(self._right_move_client):
            raise Phase5ExecutionTransportError("RIGHT_JOINT_MOVE_SERVICE_NOT_READY")

    def dispatch_pair(
        self,
        *,
        left_joint_rad: Any,
        right_joint_rad: Any,
        left_vel: float,
        right_vel: float,
        left_acc: float,
        right_acc: float,
        expected_stop_generation: int,
    ) -> tuple[CommonReleaseReceipt, Any, Any]:
        """Release one Left/Right joint_move pair from a shared host barrier."""
        self._assert_release_preconditions(expected_stop_generation)
        left_request = self._request_factory(left_joint_rad, left_vel, left_acc)
        right_request = self._request_factory(right_joint_rad, right_vel, right_acc)
        barrier = threading.Barrier(3)
        results: dict[str, Any] = {}
        errors: dict[str, BaseException] = {}
        dispatch_ns: dict[str, int] = {}

        def worker(side: str, client: Any, request: Any) -> None:
            try:
                barrier.wait()
                dispatch_ns[side] = time.perf_counter_ns()
                results[side] = client.call_async(request)
            except BaseException as error:  # preserve failure for caller thread
                errors[side] = error

        left_thread = threading.Thread(
            target=worker, args=("left", self._left_move_client, left_request), daemon=True
        )
        right_thread = threading.Thread(
            target=worker, args=("right", self._right_move_client, right_request), daemon=True
        )
        left_thread.start()
        right_thread.start()
        if int(self._stop_generation_getter()) != int(expected_stop_generation):
            barrier.abort()
            left_thread.join()
            right_thread.join()
            raise Phase5ExecutionTransportError(
                "STOP_GENERATION_CHANGED_AT_COMMON_RELEASE"
            )
        barrier.wait()
        left_thread.join()
        right_thread.join()

        if errors:
            raise Phase5ExecutionTransportError(
                "COMMON_RELEASE_DISPATCH_FAILED: "
                + "; ".join(f"{side}={error}" for side, error in sorted(errors.items()))
            )
        left_ns = dispatch_ns.get("left")
        right_ns = dispatch_ns.get("right")
        if left_ns is None or right_ns is None:
            raise Phase5ExecutionTransportError("COMMON_RELEASE_TIMESTAMP_MISSING")
        receipt = CommonReleaseReceipt(
            left_dispatch_ns=left_ns,
            right_dispatch_ns=right_ns,
            dispatch_skew_ns=abs(left_ns - right_ns),
            stop_generation=int(expected_stop_generation),
        )
        return receipt, results.get("left"), results.get("right")


@dataclass(frozen=True)
class AbsoluteStartSubmissionReceipt:
    trajectory_id: str
    start_time_unix_ns: int
    stop_generation: int
    left_message: str
    right_message: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "trajectory_id": self.trajectory_id,
            "start_time_unix_ns": self.start_time_unix_ns,
            "stop_generation": self.stop_generation,
            "left_accepted": True,
            "right_accepted": True,
            "left_message": self.left_message,
            "right_message": self.right_message,
            "common_start_mode": ABSOLUTE_COMMON_START_MODE,
            "semantic": ABSOLUTE_COMMON_START_SEMANTIC,
        }


class Phase5AbsoluteStartTransport:
    """Submit two complete arm trajectories to existing driver clients."""

    def __init__(
        self,
        *,
        left_client: Any,
        right_client: Any,
        request_factory: Callable[[Any, Any, int, str], Any],
        wait_future_result: Callable[[Any, float], Any],
        stop_generation_getter: Callable[[], int],
        driver_contract_checker: Callable[[], Any],
        wall_clock_ns: Callable[[], int] = time.time_ns,
        left_service_name: str,
        right_service_name: str,
        servo_filter_config: dict[str, Any] | None = None,
        servo_step_num: int = 1,
        configuration_error: str | None = None,
    ) -> None:
        self._left_client = left_client
        self._right_client = right_client
        self._request_factory = request_factory
        self._wait_future_result = wait_future_result
        self._stop_generation_getter = stop_generation_getter
        self._driver_contract_checker = driver_contract_checker
        self._wall_clock_ns = wall_clock_ns
        self._left_service_name = str(left_service_name)
        self._right_service_name = str(right_service_name)
        self._servo_filter_config = dict(
            servo_filter_config
            or {"mode": "LEGACY_FORESIGHT", "legacy_max_buf": 15, "legacy_kp": 0.03}
        )
        self._servo_step_num = int(servo_step_num)
        self._configuration_error = (
            str(configuration_error) if configuration_error else None
        )

    @staticmethod
    def _client_ready(client: Any) -> bool:
        checker = getattr(client, "service_is_ready", None)
        try:
            return bool(checker()) if callable(checker) else False
        except Exception:
            return False

    def both_ready(self) -> bool:
        return self._client_ready(self._left_client) and self._client_ready(
            self._right_client
        )

    def inspection_state(self) -> dict[str, Any]:
        left_ready = self._client_ready(self._left_client)
        right_ready = self._client_ready(self._right_client)
        return {
            "transport_version": ABSOLUTE_START_TRANSPORT_VERSION,
            "common_start": {
                "mode": ABSOLUTE_COMMON_START_MODE,
                "same_absolute_host_start_for_both": True,
                "full_trajectory_submitted_before_start": True,
                "hard_realtime_controller_sync": False,
                "semantic": ABSOLUTE_COMMON_START_SEMANTIC,
            },
            "robot_connection": {
                "uses_existing_driver_sessions": True,
                "new_sdk_session": False,
                "left_service": self._left_service_name,
                "right_service": self._right_service_name,
                "left_ready": left_ready,
                "right_ready": right_ready,
                "both_ready": left_ready and right_ready,
            },
            "execution_path": {
                "service_type": "jaka_msgs/srv/ExecuteJointTrajectory",
                "driver_servo_mode": "ABS",
                "servo_step_num": self._servo_step_num,
                "driver_interpolation_mode": PHASE5_DRIVER_INTERPOLATION_MODE,
                "driver_interpolation_period_ms": self._servo_step_num * 8,
                "driver_contract_marker": PHASE5_DRIVER_CONTRACT_MARKER,
                "driver_contract_preflight_required": True,
                "direct_sdk_calls_from_backend": False,
            },
            "phase5_servo_filter": {
                "valid": self._configuration_error is None,
                "error": self._configuration_error,
                **self._servo_filter_config,
                "scope": "PHASE5_EXECUTE_JOINT_TRAJECTORY_ONLY",
            },
        }

    def _assert_preconditions(self, expected_stop_generation: int) -> None:
        if self._configuration_error is not None:
            raise Phase5ExecutionTransportError(
                f"PHASE5_MOTION_CONFIGURATION_INVALID: {self._configuration_error}"
            )
        if int(self._stop_generation_getter()) != int(expected_stop_generation):
            raise Phase5ExecutionTransportError("STOP_GENERATION_CHANGED")
        if not self._client_ready(self._left_client):
            raise Phase5ExecutionTransportError(
                "LEFT_EXECUTE_JOINT_TRAJECTORY_SERVICE_NOT_READY"
            )
        if not self._client_ready(self._right_client):
            raise Phase5ExecutionTransportError(
                "RIGHT_EXECUTE_JOINT_TRAJECTORY_SERVICE_NOT_READY"
            )
        try:
            contract = self._driver_contract_checker()
        except Exception as error:
            raise Phase5ExecutionTransportError(
                f"PHASE5_DRIVER_CONTRACT_CHECK_FAILED: {error}"
            ) from error
        if not isinstance(contract, dict) or contract.get("ok") is not True:
            raise Phase5ExecutionTransportError(
                f"PHASE5_DRIVER_CONTRACT_MISMATCH: {contract!r}"
            )

    def submit_pair(
        self,
        *,
        common_timestamps_s: Any,
        left_positions_rad: Any,
        right_positions_rad: Any,
        start_time_unix_ns: int,
        trajectory_id: str,
        expected_stop_generation: int,
        response_timeout_s: float = 0.75,
    ) -> AbsoluteStartSubmissionReceipt:
        self._assert_preconditions(expected_stop_generation)
        common_times = [float(value) for value in common_timestamps_s]
        left_flat = [float(value) for sample in left_positions_rad for value in sample]
        right_flat = [float(value) for sample in right_positions_rad for value in sample]
        left_request = self._request_factory(
            common_times,
            left_flat,
            int(start_time_unix_ns),
            str(trajectory_id),
            self._servo_filter_config,
            self._servo_step_num,
        )
        right_request = self._request_factory(
            common_times,
            right_flat,
            int(start_time_unix_ns),
            str(trajectory_id),
            self._servo_filter_config,
            self._servo_step_num,
        )
        if int(self._stop_generation_getter()) != int(expected_stop_generation):
            raise Phase5ExecutionTransportError(
                "STOP_GENERATION_CHANGED_BEFORE_SUBMISSION"
            )
        try:
            # Both requests are queued before waiting for either response.
            left_future = self._left_client.call_async(left_request)
            right_future = self._right_client.call_async(right_request)
        except Exception as error:
            raise Phase5ExecutionTransportError(
                f"TRAJECTORY_SUBMISSION_FAILED: {error}"
            ) from error

        responses = {}
        for side, future in (("left", left_future), ("right", right_future)):
            remaining_s = (int(start_time_unix_ns) - self._wall_clock_ns()) / 1e9
            timeout_s = min(float(response_timeout_s), max(0.0, remaining_s - 0.2))
            if timeout_s <= 0.0:
                raise Phase5ExecutionTransportError(
                    "COMMON_START_REACHED_BEFORE_BOTH_ACCEPTANCES"
                )
            response = self._wait_future_result(future, timeout_s)
            if response is None:
                raise Phase5ExecutionTransportError(
                    f"{side.upper()}_TRAJECTORY_ACCEPTANCE_TIMEOUT"
                )
            responses[side] = response

        for side, response in responses.items():
            if getattr(response, "accepted", False) is not True:
                message = str(getattr(response, "message", "rejected"))
                raise Phase5ExecutionTransportError(
                    f"{side.upper()}_TRAJECTORY_REJECTED: {message}"
                )
            if int(getattr(response, "ret", 0)) != 1:
                raise Phase5ExecutionTransportError(
                    f"{side.upper()}_TRAJECTORY_RET_NOT_ACCEPTED"
                )
            response_id = str(getattr(response, "trajectory_id", ""))
            if response_id != str(trajectory_id):
                raise Phase5ExecutionTransportError(
                    f"{side.upper()}_TRAJECTORY_ID_MISMATCH"
                )
        if int(self._stop_generation_getter()) != int(expected_stop_generation):
            raise Phase5ExecutionTransportError(
                "STOP_GENERATION_CHANGED_DURING_ACCEPTANCE"
            )
        if (
            self._wall_clock_ns()
            >= int(start_time_unix_ns) - MINIMUM_ACCEPTANCE_MARGIN_NS
        ):
            raise Phase5ExecutionTransportError(
                "INSUFFICIENT_MARGIN_BEFORE_COMMON_START_AFTER_ACCEPTANCE"
            )
        return AbsoluteStartSubmissionReceipt(
            trajectory_id=str(trajectory_id),
            start_time_unix_ns=int(start_time_unix_ns),
            stop_generation=int(expected_stop_generation),
            left_message=str(getattr(responses["left"], "message", "accepted")),
            right_message=str(getattr(responses["right"], "message", "accepted")),
        )
