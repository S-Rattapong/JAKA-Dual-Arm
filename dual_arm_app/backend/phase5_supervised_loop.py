"""Supervised, conservative repetition of an already validated Phase-5 trajectory.

No ROS, controller, socket, motion service, or SDK imports. The owner injects
the existing Phase5ExecutionCoordinator, frozen artifact, STOP path and clock.

IMPORTANT: This code can request real motion ONLY after an explicit operator
loop-start POST with operator_confirmed is True. Every cycle obtains a NEW
single-use Phase5 prepare authority and passes the existing live safety gate.
No implicit recovery/home motion. Never replay an open-ended trajectory.
"""
from __future__ import annotations

import math
import secrets
import threading
import time
import uuid
from typing import Any, Callable


LOOP_VERSION = "PHASE5_SUPERVISED_LOOP_V1"
ENDPOINT_JOINT_TOLERANCE_RAD = 0.005
MIN_SETTLE_SECONDS = 1.0
MAX_FIXED_CYCLES = 999
HEARTBEAT_TIMEOUT_SECONDS = 5.0
MAX_FOREVER_SECONDS = 3600.0
FEEDBACK_TIMEOUT_SECONDS = 5.0
POLL_SECONDS = 0.25
# Before each cycle's prepare/execute, the cached actual feedback used by the
# start interlock must be continuously fresh for this window. Recording
# finalization/ARM and HMI reloads at a cycle boundary can stall the backend
# process for a few hundred ms (23 Nov 2026: cycle 3 was rejected with actual
# ages 516/541 ms > 500 ms right after ARM). This only delays the next start;
# it never relaxes the existing 500 ms start interlock.
FRESH_FEEDBACK_WINDOW_SECONDS = 0.5
FRESH_FEEDBACK_TIMEOUT_SECONDS = 10.0
RUNNING_STATES = frozenset({"STARTING", "RUNNING", "WAITING_FOR_SETTLE", "WAITING_FOR_GATE"})


def repeatability(artifact: Any) -> dict[str, Any]:
    """Conservative joint-closure check; physical safety must still be checked."""
    if artifact is None:
        return {"eligible": False, "reason": "FROZEN_TRAJECTORY_MISSING"}
    try:
        values = []
        for side in ("left", "right"):
            track = getattr(artifact, f"{side}_positions_rad")
            if len(track) < 2 or len(track[0]) != 6 or len(track[-1]) != 6:
                raise ValueError("TRAJECTORY_SHAPE_INVALID")
            for first, last in zip(track[0], track[-1]):
                a, b = float(first), float(last)
                if not math.isfinite(a) or not math.isfinite(b):
                    raise ValueError("NONFINITE_JOINT")
                values.append(abs(b - a))
        if not isinstance(getattr(artifact, "artifact_fingerprint"), str):
            raise ValueError("FINGERPRINT_MISSING")
        delta = max(values)
    except (AttributeError, ValueError, TypeError, IndexError) as exc:
        return {"eligible": False, "reason": str(exc)}
    return {
        "eligible": delta <= ENDPOINT_JOINT_TOLERANCE_RAD,
        "reason": ("JOINT_CLOSED_START_END" if delta <= ENDPOINT_JOINT_TOLERANCE_RAD
                   else "OPEN_TRAJECTORY_REQUIRES_VALIDATED_RETURN_PATH"),
        "max_start_end_joint_delta_rad": delta,
        "max_allowed_rad": ENDPOINT_JOINT_TOLERANCE_RAD,
        "semantic": "JOINT-SPACE CLOSURE ONLY, NOT COLLISION OR ACCELERATION CERTIFICATION",
    }


class Phase5LoopSupervisor:
    """One active server-owned loop with browser heartbeat and hard fail-closed gates."""

    def __init__(
        self, *,
        coordinator: Any,
        execute_callback: Callable[[bool], dict] | None = None,
        artifact_snapshot_getter: Callable,
        stop_generation_getter: Callable[[], int],
        abort_callback: Callable[[], Any],
        arm_recording=None, finish_recording=None, disarm_recording=None,
        monotonic: Callable[[], float] = time.monotonic,
        settle_s: float = MIN_SETTLE_SECONDS,
        heartbeat_timeout_s: float = HEARTBEAT_TIMEOUT_SECONDS,
        forever_limit_s: float = MAX_FOREVER_SECONDS,
        feedback_timeout_s: float = FEEDBACK_TIMEOUT_SECONDS,
        poll_s: float = POLL_SECONDS,
        fresh_feedback_checker: Callable[[], dict] | None = None,
        fresh_feedback_window_s: float = FRESH_FEEDBACK_WINDOW_SECONDS,
        fresh_feedback_timeout_s: float = FRESH_FEEDBACK_TIMEOUT_SECONDS,
    ):
        self.arm_recording = arm_recording
        self.finish_recording = finish_recording
        self.disarm_recording = disarm_recording
        self.coordinator = coordinator
        self.execute_callback = execute_callback or coordinator.execute
        self.artifact_snapshot_getter = artifact_snapshot_getter
        self.stop_generation_getter = stop_generation_getter
        self.abort_callback = abort_callback
        self.monotonic = monotonic
        self.settle_s = settle_s
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.forever_limit_s = forever_limit_s
        self.feedback_timeout_s = feedback_timeout_s
        self.poll_s = poll_s
        self.fresh_feedback_checker = fresh_feedback_checker
        self.fresh_feedback_window_s = fresh_feedback_window_s
        self.fresh_feedback_timeout_s = fresh_feedback_timeout_s
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._cancel_after_cycle = False
        self._stopped = False
        self._thread = None
        self._token = None
        self._last_heartbeat = None
        self._session = {
            "id": None, "mode": "once", "count": 1, "completed_cycles": 0,
            "state": "IDLE", "reason": None, "trajectory_id": None,
        }

    def active(self) -> bool:
        with self._lock:
            return self._session["state"] in RUNNING_STATES

    def preview(self) -> dict[str, Any]:
        artifact, generation, _reason = self.artifact_snapshot_getter()
        out = repeatability(artifact)
        out["artifact_fingerprint"] = (
            artifact.artifact_fingerprint if artifact is not None else None)
        out["artifact_generation"] = generation
        return out

    def status(self) -> dict[str, Any]:
        with self._lock:
            s = dict(self._session)
            s["cancel_after_current_requested"] = self._cancel_after_cycle
            s["operator_heartbeat_required"] = s["state"] in RUNNING_STATES
            s["heartbeat_age_s"] = (
                max(0.0, self.monotonic() - self._last_heartbeat)
                if self._last_heartbeat is not None else None
            )
        return {"cycle_recording_available": self.arm_recording is not None, "version": LOOP_VERSION, "loop": s, "repeatability": self.preview()}

    def _update(self, **kwargs) -> None:
        with self._lock:
            self._session.update(kwargs)

    def _get_session(self) -> dict:
        with self._lock:
            return dict(self._session)

    def on_stop(self) -> None:
        """Invoked by existing global STOP generation path. No network calls."""
        with self._lock:
            self._stopped = True
            self._wake.set()

    def heartbeat(self, session_id: str, token: str) -> dict:
        with self._lock:
            if (not self.active() or self._session["id"] != session_id
                    or not isinstance(token, str)
                    or not secrets.compare_digest(token, self._token or "")):
                return {"ok": False, "error": "LOOP_LEASE_NOT_ACTIVE_OR_INVALID"}
            self._last_heartbeat = self.monotonic()
            self._wake.set()
            return {"ok": True, "active": True, "completed_cycles": self._session["completed_cycles"]}

    def cancel_after_current(self, session_id: str, token: str) -> dict:
        with self._lock:
            if (not self.active() or self._session["id"] != session_id
                    or not isinstance(token, str)
                    or not secrets.compare_digest(token, self._token or "")):
                return {"ok": False, "error": "LOOP_LEASE_NOT_ACTIVE_OR_INVALID"}
            self._cancel_after_cycle = True
            self._wake.set()
            return {"ok": True, "cancel_after_current_requested": True,
                    "warning": "Current trajectory continues; use STOP BOTH for immediate stop."}

    def start(self, *, mode: str, count: int, operator_confirmed: Any, recording=None) -> dict:
        if recording is not None and (mode != "count" or not self.arm_recording):
            return {"ok": False, "error": "FIXED_CYCLE_RECORDING_UNAVAILABLE"}
        if operator_confirmed is not True:
            return {"ok": False, "error": "EXPLICIT_OPERATOR_CONFIRMATION_REQUIRED"}
        if mode not in ("count", "forever") or isinstance(count, bool) or not isinstance(count, int):
            return {"ok": False, "error": "INVALID_LOOP_MODE_OR_COUNT"}
        if mode == "count" and not 2 <= count <= MAX_FIXED_CYCLES:
            return {"ok": False, "error": "FIXED_COUNT_MUST_BE_2_TO_999"}
        if mode == "forever" and count != 0:
            return {"ok": False, "error": "FOREVER_COUNT_MUST_BE_ZERO"}
        with self._lock:
            if self.active() or (self._thread and self._thread.is_alive()):
                return {"ok": False, "error": "LOOP_ALREADY_ACTIVE"}
            repeat = self.preview()
            if not repeat["eligible"]:
                return {"ok": False, "error": repeat["reason"], "repeatability": repeat}
            status = self.coordinator.inspection_state()
            if status.get("ready_to_prepare") is not True:
                return {"ok": False, "error": "PHASE5_GATE_NOT_READY",
                        "blocking_reasons": status.get("blocking_reasons", [])}
            artifact = status.get("artifact") or {}
            if (artifact.get("artifact_fingerprint") != repeat["artifact_fingerprint"]
                    or artifact.get("generation") != repeat["artifact_generation"]):
                return {"ok": False, "error": "FROZEN_ARTIFACT_CHANGED_BEFORE_LOOP"}
            self._cancel_after_cycle = False
            self._stopped = False
            self._wake.clear()
            self._token = secrets.token_urlsafe(32)
            self._last_heartbeat = self.monotonic()
            sid = "p5loop-" + uuid.uuid4().hex
            self._session = {
                "id": sid, "mode": mode, "count": count,
                "completed_cycles": 0, "current_cycle": 0,
                "state": "STARTING", "reason": None, "trajectory_id": None,
                "artifact_fingerprint": repeat["artifact_fingerprint"],
                "artifact_generation": repeat["artifact_generation"],
                "start_stop_generation": self.stop_generation_getter(),
                "started_monotonic": self.monotonic(),
                "last_completed_trajectory_id": None,
                "last_driver_result": None,
            }
            self._thread = threading.Thread(
                target=self._worker, args=(sid, recording), name="phase5-supervised-loop", daemon=True
            )
            self._thread.start()
            return {"ok": True, "accepted": True, "id": sid,
                    "lease_token": self._token,
                    "mode": mode, "count": count, "repeatability": repeat,
                    "warning": "Real robot motion. Hold STOP BOTH/E-stop ready. Browser heartbeat required."}

    def _interlock(self, sid: str) -> str | None:
        with self._lock:
            if sid != self._session["id"] or self._stopped:
                return "OPERATOR_STOP_REQUESTED"
            now = self.monotonic()
            if now - (self._last_heartbeat or 0.0) >= self.heartbeat_timeout_s:
                return "OPERATOR_HEARTBEAT_LOST"
            if (self._session["mode"] == "forever"
                    and now - self._session["started_monotonic"] >= self.forever_limit_s):
                return "FOREVER_SESSION_MAX_DURATION_REACHED"
            if self.stop_generation_getter() != self._session["start_stop_generation"]:
                return "STOP_GENERATION_CHANGED"
        return None

    def _wait(self, seconds: float) -> None:
        self._wake.wait(max(0.0, min(self.poll_s, seconds)))
        self._wake.clear()

    def _await_fresh_feedback(self, sid: str) -> None:
        """Block until the start-interlock feedback is fresh for a full window.

        State stays WAITING_FOR_GATE (already an active state for the HMI
        heartbeat); progress is exposed in the gate_wait field. Fails closed on
        interlock faults or when freshness is not restored before the timeout.
        """
        if self.fresh_feedback_checker is None:
            return
        deadline = self.monotonic() + self.fresh_feedback_timeout_s
        fresh_since = None
        last_reason = None
        self._update(gate_wait="FRESH_FEEDBACK")
        while True:
            fault = self._interlock(sid)
            if fault:
                raise RuntimeError(fault)
            with self._lock:
                if self._cancel_after_cycle:
                    raise RuntimeError("OPERATOR_CANCELLED_BEFORE_NEXT_CYCLE")
            try:
                result = self.fresh_feedback_checker() or {}
            except Exception as error:  # checker failure is never treated as fresh
                result = {"match": False, "reason": f"FRESH_FEEDBACK_CHECK_ERROR: {error}"}
            now = self.monotonic()
            if result.get("match") is True:
                if fresh_since is None:
                    fresh_since = now
                if now - fresh_since >= self.fresh_feedback_window_s:
                    self._update(gate_wait=None)
                    return
            else:
                fresh_since = None
                last_reason = result.get("reason")
            if now >= deadline:
                raise RuntimeError("FRESH_FEEDBACK_NOT_RESTORED: " + str(last_reason or "UNKNOWN"))
            self._wait(self.poll_s)

    def _worker(self, sid: str, recording=None) -> None:
        run_id = None
        active_execution = False
        try:
            while True:
                fault = self._interlock(sid)
                if fault:
                    raise RuntimeError(fault)
                with self._lock:
                    if self._cancel_after_cycle:
                        self._update(state="CANCELLED", reason="OPERATOR_CANCELLED_BEFORE_NEXT_CYCLE")
                        return
                    current = self._session["completed_cycles"] + 1
                    self._update(state="WAITING_FOR_GATE", current_cycle=current)

                repeat = self.preview()
                if (not repeat["eligible"]
                        or repeat["artifact_fingerprint"] != self._session["artifact_fingerprint"]
                        or repeat["artifact_generation"] != self._session["artifact_generation"]):
                    raise RuntimeError("ARTIFACT_CHANGED_OR_NOT_CYCLIC")
                status = self.coordinator.inspection_state()
                if status.get("ready_to_prepare") is not True:
                    raise RuntimeError("REPEAT_PREPARE_BLOCKED: " + ",".join(
                        status.get("blocking_reasons") or ["UNKNOWN"]))
                if self._interlock(sid):
                    raise RuntimeError(self._interlock(sid))
                if recording is not None:
                    prefix = recording.get("label") or "Experiment"
                    label = f"{prefix} Run{current:02d}"
                    armed = self.arm_recording({**recording, "label": label,
                        "loop_cycle": {"session_id": sid, "index": current, "count": self._session["count"]}})
                    run_id = armed["run_id"]
                    self._update(recording_run_id=run_id)
                    if self._interlock(sid):
                        raise RuntimeError(self._interlock(sid))
                self._await_fresh_feedback(sid)
                prepared = self.coordinator.prepare()
                if prepared.get("ok") is not True or prepared.get("motion_dispatched") is not False:
                    raise RuntimeError("REPEAT_PREPARE_REJECTED: " + str(prepared.get("error")))
                if self._interlock(sid):
                    raise RuntimeError(self._interlock(sid))
                response = self.execute_callback(True)
                if response.get("ok") is not True or response.get("accepted") is not True:
                    # Keep the actual transport/acceptance/final-gate detail in
                    # the Loop status. The original implementation discarded
                    # this, hiding why Cycle 2 stopped after Cycle 1 completed.
                    diagnostics = {
                        "cycle": current,
                        "error": str(response.get("error") or "UNKNOWN_REJECTION"),
                        "detail": str(response.get("detail") or ""),
                        "blocking_reasons": response.get("blocking_reasons") or [],
                        "binding_mismatches": response.get("binding_mismatches") or [],
                        "stop_requested": bool(response.get("stop_requested")),
                    }
                    self._update(last_rejected_execution=diagnostics)
                    raise RuntimeError(
                        "REPEAT_EXECUTE_REJECTED: " + diagnostics["error"]
                        + (": " + diagnostics["detail"] if diagnostics["detail"] else "")
                        + (": " + ",".join(diagnostics["blocking_reasons"])
                           if diagnostics["blocking_reasons"] else "")
                    )
                active_execution = True
                trajectory_id = (response.get("execution") or {}).get("trajectory_id")
                if not trajectory_id:
                    raise RuntimeError("ACCEPTED_TRAJECTORY_ID_MISSING")
                self._update(state="RUNNING", trajectory_id=trajectory_id)
                last_authoritative = self.monotonic()
                while True:
                    fault = self._interlock(sid)
                    if fault:
                        raise RuntimeError(fault)
                    result = self.coordinator.inspection_state()
                    execution = result.get("execution") or {}
                    feedback = result.get("driver_feedback") or {}
                    if execution.get("trajectory_id") != trajectory_id:
                        raise RuntimeError("TRAJECTORY_ID_CHANGED_DURING_LOOP")
                    if feedback.get("authoritative") is True:
                        last_authoritative = self.monotonic()
                    if (feedback.get("authoritative") is True
                            and feedback.get("trajectory_id") == trajectory_id
                            and feedback.get("combined_state") == "COMPLETED"
                            and execution.get("state") == "COMPLETED"):
                        active_execution = False
                        break
                    if (feedback.get("authoritative") is True
                            and feedback.get("combined_state") in ("FAILED", "ABORTED", "ABORT_REQUESTED")):
                        raise RuntimeError("DRIVER_TERMINAL_NOT_SUCCESS: " + str(feedback.get("combined_state")))
                    if self.monotonic() - last_authoritative >= self.feedback_timeout_s:
                        raise RuntimeError("DRIVER_FEEDBACK_TIMEOUT")
                    self._wait(self.poll_s)
                if run_id:
                    deadline = self.monotonic() + 60
                    while not self.finish_recording(run_id, trajectory_id, result):
                        fault = self._interlock(sid)
                        if fault or self.monotonic() >= deadline:
                            raise RuntimeError(fault or "RECORDER_FINALIZATION_TIMEOUT")
                        self._wait(self.poll_s)
                    self._update(saved_cycles=current)
                    run_id = None
                with self._lock:
                    self._session["completed_cycles"] += 1
                    self._session["last_completed_trajectory_id"] = trajectory_id
                    self._session["last_driver_result"] = "COMPLETED"
                    self._session["trajectory_id"] = None
                    if self._cancel_after_cycle:
                        self._update(state="CANCELLED", reason="OPERATOR_CANCELLED_AFTER_CYCLE")
                        return
                    if (self._session["mode"] == "count"
                            and self._session["completed_cycles"] >= self._session["count"]):
                        self._update(state="COMPLETED", reason="FIXED_CYCLES_COMPLETED")
                        return
                    self._update(state="WAITING_FOR_SETTLE")
                until = self.monotonic() + self.settle_s
                while self.monotonic() < until:
                    fault = self._interlock(sid)
                    if fault:
                        raise RuntimeError(fault)
                    with self._lock:
                        if self._cancel_after_cycle:
                            self._update(state="CANCELLED", reason="OPERATOR_CANCELLED_BETWEEN_CYCLES")
                            return
                    self._wait(until - self.monotonic())
        except Exception as exc:
            reason = str(exc)
            # STOP generation and feedback failure must not permit any further dispatch.
            # A current accepted trajectory is aborted through the EXISTING STOP BOTH path.
            if active_execution and reason not in ("OPERATOR_STOP_REQUESTED", "STOP_GENERATION_CHANGED"):
                try:
                    self.abort_callback()
                except Exception as abort_error:
                    reason += "; STOP_BOTH_ERROR=" + str(abort_error)
            if run_id:
                try:
                    self.disarm_recording(run_id)
                except Exception as cleanup_error:
                    reason += "; RECORDER_CLEANUP_ERROR=" + str(cleanup_error)
            self._update(state="STOPPED" if "STOP" in reason or "CANCELLED" in reason
                         else "FAILED", reason=reason, trajectory_id=None)
        finally:
            with self._lock:
                self._token = None
                self._last_heartbeat = None
                self._wake.set()
