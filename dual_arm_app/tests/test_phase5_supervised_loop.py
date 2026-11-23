"""OFFLINE supervised Phase-5 loop tests. No ROS, controller, or network."""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
import threading
import time
import unittest

from dual_arm_app.backend.phase5_supervised_loop import (
    Phase5LoopSupervisor,
    repeatability,
)

def artifact(closed=True):
    return SimpleNamespace(
        artifact_fingerprint="fixture-closed-or-open",
        left_positions_rad=((0.0,) * 6, (0.0,) * 6 if closed else (0.2,) * 6),
        right_positions_rad=((0.0,) * 6, (0.0,) * 6),
        duration_s=1.0,
    )


class MockCoordinator:
    def __init__(self, shared, complete_after=3, valid_feedback=True):
        self.shared = shared
        self.complete_after = complete_after
        self.valid_feedback = valid_feedback
        self.prepare_count = 0
        self.execute_count = 0
        self.poll_count = 0
        self.running = False
        self.complete = False
        self.tid = None
        self._lock = threading.RLock()
        self.allow_gate = True

    def inspection_state(self):
        with self._lock:
            if self.running:
                self.poll_count += 1
                if self.poll_count >= self.complete_after and self.valid_feedback:
                    self.running = False
                    self.complete = True
            feedback = {
                "authoritative": self.valid_feedback,
                "trajectory_id": self.tid,
                "combined_state": (
                    "COMPLETED" if self.complete
                    else "RUNNING" if self.running else "IDLE"
                ),
            }
            return {
                "ready_to_prepare": not self.running and self.allow_gate,
                "blocking_reasons": [] if self.allow_gate else ["GATE_BLOCKED"],
                "artifact": {
                    "artifact_fingerprint": self.shared["artifact"].artifact_fingerprint,
                    "generation": self.shared["generation"],
                },
                "execution": {
                    "trajectory_id": self.tid,
                    "state": "COMPLETED" if self.complete
                             else "RUNNING" if self.running else "IDLE",
                },
                "driver_feedback": feedback,
            }

    def prepare(self):
        with self._lock:
            if self.running or not self.allow_gate:
                return {"ok": False, "error": "GATE_BLOCKED"}
            self.prepare_count += 1
        return {"ok": True, "motion_dispatched": False}

    def execute(self, operator_confirmed):
        with self._lock:
            assert operator_confirmed is True
            if self.running:
                return {"ok": False, "error": "ALREADY_ACTIVE"}
            self.execute_count += 1
            self.tid = "test-trajectory-" + str(self.execute_count)
            self.complete = False
            self.running = True
            self.poll_count = 0
            return {"ok": True, "accepted": True,
                    "execution": {"trajectory_id": self.tid}}


def make_manager(*, closed=True, complete_after=3, valid_feedback=True,
                 heartbeat_timeout_s=.8, settle_s=.02, feedback_timeout_s=.25):
    shared = {"artifact": artifact(closed=closed), "generation": 7, "stop": 11}
    coordinator = MockCoordinator(shared, complete_after=complete_after, valid_feedback=valid_feedback)
    aborts = []

    def existing_stop():
        aborts.append("STOP BOTH EXISTING PATH")
        shared["stop"] += 1

    mgr = Phase5LoopSupervisor(
        coordinator=coordinator,
        execute_callback=coordinator.execute,
        artifact_snapshot_getter=lambda: (
            shared["artifact"], shared["generation"], None),
        stop_generation_getter=lambda: shared["stop"],
        abort_callback=existing_stop,
        settle_s=settle_s, poll_s=.005,
        heartbeat_timeout_s=heartbeat_timeout_s,
        feedback_timeout_s=feedback_timeout_s,
    )
    return mgr, coordinator, shared, aborts


def await_state(mgr, targets, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = mgr.status()["loop"]
        if s["state"] in targets:
            return s
        time.sleep(.005)
    raise AssertionError(f"Timeout waiting for {targets}; got {mgr.status()['loop']}")


class Phase5LoopOfflineSafetyTests(unittest.TestCase):
    def test_open_trajectory_rejected_without_motion(self):
        mgr, c, _shared, aborts = make_manager(closed=False)
        result = mgr.start(mode="count", count=3, operator_confirmed=True)
        self.assertFalse(result["ok"])
        self.assertIn("OPEN_TRAJECTORY", result["error"])
        self.assertEqual(c.prepare_count, 0)
        self.assertEqual(c.execute_count, 0)
        self.assertEqual(aborts, [])

    def test_unknown_authority_or_count_rejected(self):
        mgr, c, _shared, _ = make_manager()
        for req in (
            {"mode": "count", "count": 3, "operator_confirmed": 1},
            {"mode": "count", "count": 1, "operator_confirmed": True},
            {"mode": "count", "count": 1000, "operator_confirmed": True},
            {"mode": "forever", "count": 3, "operator_confirmed": True},
            {"mode": "once", "count": 1, "operator_confirmed": True},
            {"mode": "count", "count": True, "operator_confirmed": True},
        ):
            self.assertFalse(mgr.start(**req)["ok"], req)
        self.assertEqual(c.execute_count, 0)

    def test_fixed_loops_reprepare_each_cycle_after_authoritative_driver_success(self):
        mgr, c, shared, aborts = make_manager()
        result = mgr.start(mode="count", count=3, operator_confirmed=True)
        self.assertTrue(result["ok"])
        s = await_state(mgr, {"COMPLETED"})
        self.assertEqual(s["completed_cycles"], 3)
        self.assertEqual(s["reason"], "FIXED_CYCLES_COMPLETED")
        self.assertEqual(c.prepare_count, 3)
        self.assertEqual(c.execute_count, 3)
        self.assertEqual(aborts, [])
        self.assertEqual(shared["stop"], 11)

    def test_duplicate_start_is_rejected(self):
        mgr, c, shared, aborts = make_manager(complete_after=5000)
        one = mgr.start(mode="count", count=2, operator_confirmed=True)
        self.assertTrue(one["ok"])
        await_state(mgr, {"RUNNING"})
        two = mgr.start(mode="forever", count=0, operator_confirmed=True)
        self.assertEqual(two["error"], "LOOP_ALREADY_ACTIVE")
        shared["stop"] += 1
        mgr.on_stop()
        await_state(mgr, {"STOPPED"})
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(aborts, [])

    def test_cancel_after_current_does_not_abort_motion_or_send_next(self):
        mgr, c, _shared, aborts = make_manager(complete_after=15)
        started = mgr.start(mode="forever", count=0, operator_confirmed=True)
        await_state(mgr, {"RUNNING"})
        self.assertFalse(mgr.cancel_after_current(started["id"], "wrong-token")["ok"])
        self.assertTrue(mgr.cancel_after_current(started["id"], started["lease_token"])["ok"])
        s = await_state(mgr, {"CANCELLED"})
        self.assertEqual(s["completed_cycles"], 1)
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(aborts, [])

    def test_global_stop_generation_cancels_future_iterations(self):
        mgr, c, shared, aborts = make_manager(complete_after=5000)
        started = mgr.start(mode="forever", count=0, operator_confirmed=True)
        await_state(mgr, {"RUNNING"})
        shared["stop"] += 1
        mgr.on_stop()
        await_state(mgr, {"STOPPED"})
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(aborts, [])  # external STOP path already issued physical stop

    def test_heartbeat_loss_requests_existing_stop_path_and_halts(self):
        mgr, c, shared, aborts = make_manager(complete_after=99999, heartbeat_timeout_s=.06)
        started = mgr.start(mode="forever", count=0, operator_confirmed=True)
        await_state(mgr, {"RUNNING"})
        s = await_state(mgr, {"FAILED", "STOPPED"})
        self.assertIn("HEARTBEAT_LOST", s["reason"])
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(aborts, ["STOP BOTH EXISTING PATH"])
        self.assertEqual(shared["stop"], 12)

    def test_driver_feedback_timeout_requests_existing_stop_path(self):
        mgr, c, shared, aborts = make_manager(
            complete_after=99999, valid_feedback=False, feedback_timeout_s=.07)
        mgr.start(mode="forever", count=0, operator_confirmed=True)
        s = await_state(mgr, {"FAILED", "STOPPED"})
        self.assertIn("FEEDBACK_TIMEOUT", s["reason"])
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(len(aborts), 1)

    def test_frozen_artifact_change_blocks_next_iteration(self):
        mgr, c, shared, aborts = make_manager(settle_s=.15)
        mgr.start(mode="forever", count=0, operator_confirmed=True)
        await_state(mgr, {"WAITING_FOR_SETTLE"})
        shared["generation"] += 1
        s = await_state(mgr, {"FAILED"})
        self.assertIn("ARTIFACT_CHANGED", s["reason"])
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(aborts, [])

    def test_gate_failure_after_first_completion_blocks_second(self):
        mgr, c, shared, aborts = make_manager(settle_s=.14)
        mgr.start(mode="count", count=2, operator_confirmed=True)
        await_state(mgr, {"WAITING_FOR_SETTLE"})
        c.allow_gate = False
        s = await_state(mgr, {"FAILED"})
        self.assertIn("GATE", s["reason"])
        self.assertEqual(c.execute_count, 1)
        self.assertEqual(aborts, [])

    def test_lease_token_is_required_and_renewable(self):
        mgr, c, shared, _aborts = make_manager(complete_after=5000)
        started = mgr.start(mode="forever", count=0, operator_confirmed=True)
        await_state(mgr, {"RUNNING"})
        self.assertFalse(mgr.heartbeat(started["id"], "bad")["ok"])
        self.assertFalse(mgr.heartbeat("bad-id", started["lease_token"])["ok"])
        self.assertTrue(mgr.heartbeat(started["id"], started["lease_token"])["ok"])
        shared["stop"] += 1
        mgr.on_stop()
        await_state(mgr, {"STOPPED"})

    def test_repeatability_requires_finite_six_joint_closure(self):
        self.assertTrue(repeatability(artifact())["eligible"])
        self.assertFalse(repeatability(artifact(closed=False))["eligible"])
        invalid = artifact()
        invalid.left_positions_rad = ((0, 0, 0, 0, 0, float("nan")), (0,) * 6)
        self.assertFalse(repeatability(invalid)["eligible"])


if __name__ == "__main__":
    unittest.main()


def test_repeat_loop_failure_preserves_driver_acceptance_detail():
    mgr, coordinator, _shared, aborts = make_manager(settle_s=.02)
    original_execute = coordinator.execute

    def fail_on_second_cycle(confirmed):
        if coordinator.execute_count == 1:
            return {
                "ok": False,
                "error": "PHASE5_DUAL_TRAJECTORY_SUBMISSION_FAILED",
                "detail": "LEFT_TRAJECTORY_ACCEPTANCE_TIMEOUT",
                "stop_requested": True,
            }
        return original_execute(confirmed)

    mgr.execute_callback = fail_on_second_cycle
    started = mgr.start(mode="count", count=5, operator_confirmed=True)
    assert started["ok"] is True
    report = await_state(mgr, {"FAILED"})
    assert report["completed_cycles"] == 1
    assert report["current_cycle"] == 2
    assert "LEFT_TRAJECTORY_ACCEPTANCE_TIMEOUT" in report["reason"]
    assert report["last_rejected_execution"]["detail"] == "LEFT_TRAJECTORY_ACCEPTANCE_TIMEOUT"
    assert report["last_rejected_execution"]["cycle"] == 2
    assert coordinator.execute_count == 1
    assert not aborts  # Coordinator owns any STOP caused by submission failure
