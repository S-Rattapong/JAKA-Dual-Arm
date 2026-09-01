"""P5.8-P5.10 operator authority and fail-closed execution coordinator.

This module owns no ROS node and no robot SDK session. All authorities and the
whole-trajectory transport are injected, which keeps operator/gate behavior
fully offline-testable.
"""

from __future__ import annotations

import threading
import time
import uuid
from typing import Any, Callable

from dual_arm_app.backend.phase5_execution_transport import (
    ABSOLUTE_COMMON_START_MODE,
    ABSOLUTE_COMMON_START_SEMANTIC,
    MINIMUM_ACCEPTANCE_MARGIN_NS,
    Phase5ExecutionTransportError,
)


COORDINATOR_VERSION = "PHASE5_EXECUTION_COORDINATOR_V1"
COMMON_START_LEAD_S = 2.0
ACTIVE_STATES = frozenset({"PREPARING", "ARMED", "RUNNING", "ABORT_REQUESTED"})
MINIMUM_COMMON_START_LEAD_S = 1.0


class Phase5ExecutionCoordinator:
    """Single-artifact, single-use pending Phase-5 operator authority."""

    def __init__(
        self,
        *,
        artifact_snapshot_getter: Callable[[], tuple[Any, int, Any]],
        phase4_gate_getter: Callable[[str], dict[str, Any]],
        transport: Any,
        safe_state_checker: Callable[[str], tuple[bool, str]],
        start_match_checker: Callable[[Any], dict[str, Any]],
        legacy_conflict_getter: Callable[[], list[str]],
        stop_generation_getter: Callable[[], int],
        abort_callback: Callable[[], Any],
        feedback_getter: Callable[[str | None], dict[str, Any]] | None = None,
        wall_clock_ns: Callable[[], int] = time.time_ns,
        trajectory_id_factory: Callable[[], str] = lambda: "p5-" + uuid.uuid4().hex,
        common_start_lead_s: float = COMMON_START_LEAD_S,
    ) -> None:
        self._artifact_snapshot_getter = artifact_snapshot_getter
        self._phase4_gate_getter = phase4_gate_getter
        self._transport = transport
        self._safe_state_checker = safe_state_checker
        self._start_match_checker = start_match_checker
        self._legacy_conflict_getter = legacy_conflict_getter
        self._stop_generation_getter = stop_generation_getter
        self._abort_callback = abort_callback
        self._feedback_getter = feedback_getter
        self._wall_clock_ns = wall_clock_ns
        self._trajectory_id_factory = trajectory_id_factory
        self._common_start_lead_ns = int(float(common_start_lead_s) * 1e9)
        if self._common_start_lead_ns < int(MINIMUM_COMMON_START_LEAD_S * 1e9):
            raise ValueError("common_start_lead_s must be at least 1 second")
        self._lock = threading.Lock()
        self._pending_operator_authority: dict[str, Any] | None = None
        self._last_operator_authority_invalidation_reason: str | None = None
        self._execution: dict[str, Any] = {
            "state": "IDLE",
            "trajectory_id": None,
            "reason": None,
            "start_time_unix_ns": None,
            "duration_s": None,
        }

    def _refresh_execution_state_locked(self, now_ns: int) -> None:
        # Deliberately do not advance execution from wall time. P5.11-P5.15 use
        # only matching GetExecutionStatus feedback as execution authority.
        _ = now_ns

    def _read_execution_feedback(self, trajectory_id: Any) -> dict[str, Any]:
        # Do not query ROS status services while Phase 5 has no current execution.
        # This keeps legacy Jog/Home/Program active checks latency-free when P5 is idle.
        if not trajectory_id:
            return {
                "read_only": True,
                "authoritative": False,
                "feedback_status": "NO_CURRENT_TRAJECTORY",
                "combined_state": "IDLE",
                "terminal": False,
                "result": None,
                "reason": "NO_CURRENT_TRAJECTORY",
                "trajectory_id": None,
            }
        if self._feedback_getter is None:
            return {
                "read_only": True,
                "authoritative": False,
                "feedback_status": "FEEDBACK_UNAVAILABLE",
                "combined_state": "FEEDBACK_UNAVAILABLE",
                "terminal": False,
                "result": None,
                "reason": "Driver execution feedback getter is unavailable",
                "trajectory_id": str(trajectory_id) if trajectory_id else None,
            }
        try:
            feedback = self._feedback_getter(
                str(trajectory_id) if trajectory_id else None
            )
            if not isinstance(feedback, dict):
                raise TypeError("Driver execution feedback must be a dictionary")
            return feedback
        except Exception as error:
            return {
                "read_only": True,
                "authoritative": False,
                "feedback_status": "FEEDBACK_UNAVAILABLE",
                "combined_state": "FEEDBACK_UNAVAILABLE",
                "terminal": False,
                "result": None,
                "reason": str(error),
                "trajectory_id": str(trajectory_id) if trajectory_id else None,
            }

    def _apply_execution_feedback_locked(
        self, expected_trajectory_id: Any, feedback: dict[str, Any]
    ) -> bool:
        if not expected_trajectory_id:
            return False
        if self._execution.get("trajectory_id") != str(expected_trajectory_id):
            return False
        self._execution["driver_feedback"] = feedback
        self._execution["feedback_status"] = feedback.get("feedback_status")
        self._execution["driver_feedback_reason"] = feedback.get("reason")
        timeline = feedback.get("common_timeline")
        if isinstance(timeline, dict):
            for field in ("elapsed_s", "duration_s", "progress_0_to_1"):
                if timeline.get(field) is not None:
                    self._execution[field] = timeline[field]
        if feedback.get("authoritative") is not True:
            return False
        feedback_state = feedback.get("combined_state")
        current_state = self._execution.get("state")
        if current_state == "ABORT_REQUESTED" and feedback_state not in {
            "ABORTED", "FAILED"
        }:
            return False
        if feedback_state in {"COMPLETED", "ABORTED", "FAILED"}:
            self._execution["state"] = feedback_state
            self._execution["reason"] = feedback.get("reason")
            self._execution["result"] = feedback_state
            self._execution["terminal_time_unix_ns"] = max(
                (
                    int(side.get("terminal_time_unix_ns") or 0)
                    for side in (feedback.get("drivers") or {}).values()
                ),
                default=0,
            )
        elif feedback_state == "ABORT_REQUESTED":
            first_partial_terminal = current_state != "ABORT_REQUESTED"
            self._execution["state"] = "ABORT_REQUESTED"
            self._execution["reason"] = feedback.get("reason") or "PEER_TERMINAL_PENDING"
            self._execution["result"] = None
            return first_partial_terminal
        elif feedback_state in {"ARMED", "RUNNING"} and current_state != "FAILED":
            self._execution["state"] = feedback_state
        return False

    def _request_feedback_abort(self, reason: Any) -> None:
        try:
            self._abort_callback()
        except Exception as error:
            with self._lock:
                if self._execution.get("state") == "ABORT_REQUESTED":
                    self._execution["abort_error"] = str(error)
            return
        with self._lock:
            if self._execution.get("state") == "ABORT_REQUESTED":
                self._execution["reason"] = (
                    f"DRIVER_FEEDBACK_ABORT: {reason or 'PEER_TERMINAL_PENDING'}"
                )
                self._execution["abort_requested_from_driver_feedback"] = True

    def is_active(self) -> bool:
        with self._lock:
            self._refresh_execution_state_locked(self._wall_clock_ns())
            trajectory_id = self._execution.get("trajectory_id")
        feedback = self._read_execution_feedback(trajectory_id)
        with self._lock:
            abort_required = self._apply_execution_feedback_locked(trajectory_id, feedback)
        if abort_required:
            self._request_feedback_abort(feedback.get("reason"))
        with self._lock:
            return self._execution.get("state") in ACTIVE_STATES

    def invalidate_authority(self, reason: str) -> None:
        abort_active = False
        with self._lock:
            self._pending_operator_authority = None
            self._last_operator_authority_invalidation_reason = str(reason)
            self._refresh_execution_state_locked(self._wall_clock_ns())
            if self._execution.get("state") in ACTIVE_STATES:
                abort_active = True
                self._execution["reason"] = f"AUTHORITY_INVALIDATED: {reason}"
        if abort_active:
            try:
                self._abort_callback()
            except Exception as error:
                with self._lock:
                    self._execution["state"] = "FAILED"
                    self._execution["abort_error"] = str(error)

    def on_stop(self, stop_generation: int) -> None:
        with self._lock:
            self._pending_operator_authority = None
            self._last_operator_authority_invalidation_reason = (
                "STOP_GENERATION_CHANGED"
            )
            self._refresh_execution_state_locked(self._wall_clock_ns())
            if self._execution.get("state") in ACTIVE_STATES:
                # P5.10 can prove that STOP was requested, but P5.15 owns the
                # authoritative terminal/abort acknowledgement. Stay active
                # and block conflicting commands until feedback closes it.
                self._execution["state"] = "ABORT_REQUESTED"
                self._execution["reason"] = "STOP_GENERATION_CHANGED"
                self._execution["stop_generation"] = int(stop_generation)

    @staticmethod
    def _binding_mismatches(
        pending: dict[str, Any], readiness: dict[str, Any]
    ) -> list[str]:
        return [
            field
            for field in (
                "artifact_fingerprint",
                "plan_fingerprint",
                "artifact_generation",
                "stop_generation",
            )
            if readiness.get(field) != pending.get(field)
        ]

    def _evaluate_readiness(self) -> dict[str, Any]:
        stop_generation = int(self._stop_generation_getter())
        artifact, artifact_generation, artifact_reason = (
            self._artifact_snapshot_getter()
        )
        blockers: list[str] = []
        gate: dict[str, Any] = {}
        if artifact is None:
            blockers.append("FROZEN_ARTIFACT_UNAVAILABLE")
        else:
            gate = self._phase4_gate_getter(artifact.plan_fingerprint)
            if gate.get("execution_ready") is not True:
                blockers.append("PHASE4_GATE_NOT_READY_FOR_ARTIFACT_FINGERPRINT")
        transport_state = self._transport.inspection_state()
        connection = transport_state.get("robot_connection") or {}
        if connection.get("both_ready") is not True:
            blockers.append("PHASE5_TRAJECTORY_SERVICES_NOT_READY")
        filter_state = transport_state.get("phase5_servo_filter")
        if filter_state is not None and filter_state.get("valid") is not True:
            blockers.append("PHASE5_SERVO_FILTER_CONFIGURATION_INVALID")
        safe, safe_message = self._safe_state_checker("both")
        if not safe:
            blockers.append(f"ROBOT_SAFE_STATE_BLOCKED: {safe_message}")
        try:
            start_match = self._start_match_checker(artifact)
        except Exception as error:
            start_match = {
                "match": False,
                "state": "BLOCKED",
                "reason": f"START_MATCH_CHECK_FAILED: {error}",
                "max_delta_rad": None,
                "threshold_rad": None,
            }
        if start_match.get("match") is not True:
            blockers.append(
                "ACTUAL_START_INTERLOCK_BLOCKED: "
                + str(start_match.get("reason") or "START_NOT_MATCHED")
            )
        legacy_conflicts = list(self._legacy_conflict_getter())
        blockers.extend(legacy_conflicts)

        current_artifact, current_generation, _reason = (
            self._artifact_snapshot_getter()
        )
        if artifact is not current_artifact or artifact_generation != current_generation:
            blockers.append("FROZEN_ARTIFACT_CHANGED_DURING_GATE_CHECK")
        if int(self._stop_generation_getter()) != stop_generation:
            blockers.append("STOP_GENERATION_CHANGED_DURING_GATE_CHECK")
        return {
            "ready": not blockers,
            "blocking_reasons": list(dict.fromkeys(blockers)),
            "artifact": artifact,
            "artifact_generation": int(artifact_generation),
            "artifact_reason": artifact_reason,
            "artifact_fingerprint": (
                artifact.artifact_fingerprint if artifact is not None else None
            ),
            "plan_fingerprint": (
                artifact.plan_fingerprint if artifact is not None else None
            ),
            "phase4_gate": gate,
            "transport": transport_state,
            "safe_state_ok": bool(safe),
            "safe_state_message": str(safe_message),
            "legacy_conflicts": legacy_conflicts,
            "start_match": start_match,
            "stop_generation": stop_generation,
        }

    def inspection_state(self) -> dict[str, Any]:
        readiness = self._evaluate_readiness()
        now_ns = self._wall_clock_ns()
        with self._lock:
            trajectory_id = self._execution.get("trajectory_id")
        feedback = self._read_execution_feedback(trajectory_id)
        with self._lock:
            self._refresh_execution_state_locked(now_ns)
            abort_required = self._apply_execution_feedback_locked(trajectory_id, feedback)
            pending = self._pending_operator_authority
            if pending is not None:
                mismatches = self._binding_mismatches(pending, readiness)
                if not readiness["ready"] or mismatches:
                    self._pending_operator_authority = None
                    self._last_operator_authority_invalidation_reason = (
                        "READINESS_OR_AUTHORITY_CHANGED"
                    )
                    pending = None
        if abort_required:
            self._request_feedback_abort(feedback.get("reason"))
        with self._lock:
            execution = dict(self._execution)
        return {
            "ok": True,
            "coordinator_version": COORDINATOR_VERSION,
            "ready_to_prepare": readiness["ready"] and execution["state"] not in ACTIVE_STATES,
            "blocking_reasons": readiness["blocking_reasons"],
            "artifact": {
                "available": readiness["artifact"] is not None,
                "artifact_fingerprint": readiness["artifact_fingerprint"],
                "plan_fingerprint": readiness["plan_fingerprint"],
                "generation": readiness["artifact_generation"],
                "reason": readiness["artifact_reason"],
            },
            "phase4_gate": readiness["phase4_gate"],
            "transport": readiness["transport"],
            "safe_state": {
                "ok": readiness["safe_state_ok"],
                "message": readiness["safe_state_message"],
            },
            "start_match": readiness["start_match"],
            "legacy_conflicts": readiness["legacy_conflicts"],
            "stop_generation": readiness["stop_generation"],
            "operator_authority": {
                "pending": pending is not None,
                "single_use": True,
                "no_expiry": True,
                "last_invalidation_reason": (
                    self._last_operator_authority_invalidation_reason
                ),
            },
            "execution": execution,
            "driver_feedback": execution.get("driver_feedback", feedback),
            "common_start_mode": ABSOLUTE_COMMON_START_MODE,
            "semantic": ABSOLUTE_COMMON_START_SEMANTIC,
            "status_source": (
                "AUTHORITATIVE MATCHING LEFT + RIGHT DRIVER GetExecutionStatus; "
                "WALL TIME NEVER INFERS COMPLETION"
            ),
        }

    def prepare(self) -> dict[str, Any]:
        if self.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ALREADY_ACTIVE"}
        readiness = self._evaluate_readiness()
        if not readiness["ready"]:
            self.invalidate_authority("PREPARE_GATE_BLOCKED")
            return {
                "ok": False,
                "error": "PHASE5_PREPARE_GATE_BLOCKED",
                "blocking_reasons": readiness["blocking_reasons"],
            }
        now_ns = self._wall_clock_ns()
        authority = {
            "created_at_unix_ns": now_ns,
            "artifact_fingerprint": readiness["artifact_fingerprint"],
            "plan_fingerprint": readiness["plan_fingerprint"],
            "artifact_generation": readiness["artifact_generation"],
            "stop_generation": readiness["stop_generation"],
        }
        with self._lock:
            self._refresh_execution_state_locked(now_ns)
            if self._execution.get("state") in ACTIVE_STATES:
                return {"ok": False, "error": "PHASE5_EXECUTION_ALREADY_ACTIVE"}
            self._pending_operator_authority = authority
            self._last_operator_authority_invalidation_reason = None
        artifact = readiness["artifact"]
        return {
            "ok": True,
            "motion_dispatched": False,
            "authority": dict(authority),
            "trajectory": {
                "name": artifact.trajectory_name,
                "sample_count": artifact.sample_count,
                "duration_s": artifact.duration_s,
            },
            "semantic": "NO MOTION — PENDING OPERATOR AUTHORITY PREPARED",
        }

    def execute(self, operator_confirmed: Any) -> dict[str, Any]:
        now_ns = self._wall_clock_ns()
        with self._lock:
            self._refresh_execution_state_locked(now_ns)
            pending = self._pending_operator_authority
            # Every execute attempt consumes the only pending authority.
            self._pending_operator_authority = None
            if pending is None:
                return {
                    "ok": False,
                    "error": "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED",
                }
            if operator_confirmed is not True:
                self._last_operator_authority_invalidation_reason = (
                    "OPERATOR_CONFIRMATION_REQUIRED"
                )
                return {"ok": False, "error": "OPERATOR_CONFIRMATION_REQUIRED"}
            if self._execution.get("state") in ACTIVE_STATES:
                self._last_operator_authority_invalidation_reason = (
                    "PHASE5_EXECUTION_ALREADY_ACTIVE"
                )
                return {"ok": False, "error": "PHASE5_EXECUTION_ALREADY_ACTIVE"}
            self._execution = {
                "state": "PREPARING",
                "trajectory_id": None,
                "reason": None,
                "start_time_unix_ns": None,
                "duration_s": None,
            }

        readiness = self._evaluate_readiness()
        binding_mismatches = self._binding_mismatches(pending, readiness)
        if not readiness["ready"] or binding_mismatches:
            with self._lock:
                self._last_operator_authority_invalidation_reason = (
                    "READINESS_OR_AUTHORITY_CHANGED"
                )
                self._execution["state"] = "FAILED"
                self._execution["reason"] = (
                    "OPERATOR_AUTHORITY_STALE_OR_GATE_BLOCKED"
                )
            return {
                "ok": False,
                "error": "OPERATOR_AUTHORITY_STALE_OR_GATE_BLOCKED",
                "binding_mismatches": binding_mismatches,
                "blocking_reasons": readiness["blocking_reasons"],
            }

        artifact = readiness["artifact"]
        trajectory_id = str(self._trajectory_id_factory())
        start_time_unix_ns = self._wall_clock_ns() + self._common_start_lead_ns
        with self._lock:
            self._execution = {
                "state": "PREPARING",
                "trajectory_id": trajectory_id,
                "reason": None,
                "artifact_fingerprint": artifact.artifact_fingerprint,
                "plan_fingerprint": artifact.plan_fingerprint,
                "artifact_generation": readiness["artifact_generation"],
                "stop_generation": readiness["stop_generation"],
                "start_time_unix_ns": start_time_unix_ns,
                "duration_s": artifact.duration_s,
            }

        try:
            receipt = self._transport.submit_pair(
                common_timestamps_s=artifact.common_timestamps_s,
                left_positions_rad=artifact.left_positions_rad,
                right_positions_rad=artifact.right_positions_rad,
                start_time_unix_ns=start_time_unix_ns,
                trajectory_id=trajectory_id,
                expected_stop_generation=readiness["stop_generation"],
            )
            final_readiness = self._evaluate_readiness()
            final_identity = (
                final_readiness["ready"]
                and final_readiness["artifact_fingerprint"]
                == artifact.artifact_fingerprint
                and final_readiness["artifact_generation"]
                == readiness["artifact_generation"]
                and final_readiness["stop_generation"]
                == readiness["stop_generation"]
                and self._wall_clock_ns()
                < start_time_unix_ns - MINIMUM_ACCEPTANCE_MARGIN_NS
            )
            if not final_identity:
                raise Phase5ExecutionTransportError(
                    "EXECUTION_AUTHORITY_CHANGED_AFTER_DRIVER_ACCEPTANCE"
                )
        except Exception as error:
            abort_error = None
            try:
                self._abort_callback()
            except Exception as stop_error:
                abort_error = str(stop_error)
            with self._lock:
                self._execution["state"] = "FAILED"
                self._execution["reason"] = str(error)
                if abort_error is not None:
                    self._execution["abort_error"] = abort_error
            return {
                "ok": False,
                "error": "PHASE5_DUAL_TRAJECTORY_SUBMISSION_FAILED",
                "detail": str(error),
                "stop_requested": True,
                "stop_error": abort_error,
            }

        with self._lock:
            self._execution["state"] = "ARMED"
            self._execution["receipt"] = receipt.to_dict()
            execution = dict(self._execution)
        return {
            "ok": True,
            "accepted": True,
            "execution": execution,
            "common_start_mode": ABSOLUTE_COMMON_START_MODE,
            "semantic": ABSOLUTE_COMMON_START_SEMANTIC,
        }
