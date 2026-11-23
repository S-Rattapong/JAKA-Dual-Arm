"""OFFLINE: supervised loop waits for continuously fresh actual feedback before each cycle.

Regression for 23 Nov 2026: a 50-cycle recorded loop stopped at cycle 3 because the
start interlock saw actual feedback ages 516/541 ms (> 500 ms) right after the
recorder was armed. No ROS, controller or network is used here.
"""
from pathlib import Path
import re
import threading
import time
import unittest

from dual_arm_app.tests.test_phase5_supervised_loop import make_manager, await_state

APP = Path(__file__).resolve().parents[1]

FRESH = {"match": True, "reason": None}
STALE = {"match": False, "reason": "LEFT_ACTUAL_FEEDBACK_STALE: age_ms=516.000 > 500"}


class ScriptedChecker:
    """Returns stale for the first ``stale_calls`` calls, then fresh."""

    def __init__(self, coordinator, stale_calls=0, always=None, raise_error=False):
        self.coordinator = coordinator
        self.stale_calls = stale_calls
        self.always = always
        self.raise_error = raise_error
        self.calls = 0
        self.executes_seen_while_stale = []
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            self.calls += 1
            if self.raise_error:
                raise ValueError("ros cache unavailable")
            if self.always is not None:
                result = self.always
            else:
                result = STALE if self.calls <= self.stale_calls else FRESH
            if result is not FRESH:
                self.executes_seen_while_stale.append(self.coordinator.execute_count)
            return dict(result)


def manager_with(checker_factory, *, window=.05, timeout=.4, **kwargs):
    mgr, coordinator, shared, aborts = make_manager(complete_after=1, heartbeat_timeout_s=30, **kwargs)
    checker = checker_factory(coordinator)
    mgr.fresh_feedback_checker = checker
    mgr.fresh_feedback_window_s = window
    mgr.fresh_feedback_timeout_s = timeout
    return mgr, coordinator, checker, aborts


class FreshFeedbackWaitTests(unittest.TestCase):
    def test_cycle_waits_until_feedback_fresh_for_full_window(self):
        mgr, coordinator, checker, aborts = manager_with(lambda c: ScriptedChecker(c, stale_calls=6))
        started = mgr.start(mode="count", count=3, operator_confirmed=True)
        self.assertTrue(started["ok"], started)
        status = await_state(mgr, {"COMPLETED", "FAILED", "STOPPED"}, 5)
        self.assertEqual(status["state"], "COMPLETED", status)
        self.assertEqual(coordinator.execute_count, 3)
        # Every stale observation happened before the first execute.
        self.assertEqual(set(checker.executes_seen_while_stale), {0})
        # A full window needs several fresh polls per cycle (window .05 s / poll .005 s).
        self.assertGreaterEqual(checker.calls, 6 + 3 * 5)
        self.assertIsNone(status.get("gate_wait"))
        self.assertFalse(aborts)

    def test_never_fresh_fails_closed_without_execute(self):
        mgr, coordinator, checker, aborts = manager_with(lambda c: ScriptedChecker(c, always=STALE))
        mgr.start(mode="count", count=5, operator_confirmed=True)
        status = await_state(mgr, {"COMPLETED", "FAILED", "STOPPED"}, 5)
        self.assertEqual(status["state"], "FAILED", status)
        self.assertIn("FRESH_FEEDBACK_NOT_RESTORED", status["reason"])
        self.assertIn("LEFT_ACTUAL_FEEDBACK_STALE", status["reason"])
        self.assertEqual(coordinator.execute_count, 0)
        self.assertEqual(coordinator.prepare_count, 0)
        self.assertFalse(aborts)  # nothing was moving, no STOP BOTH needed

    def test_flapping_freshness_never_satisfies_window(self):
        class Flapping(ScriptedChecker):
            def __call__(self):
                with self._lock:
                    self.calls += 1
                    return dict(FRESH if self.calls % 3 else STALE)

        mgr, coordinator, _checker, _ = manager_with(lambda c: Flapping(c))
        mgr.start(mode="count", count=2, operator_confirmed=True)
        status = await_state(mgr, {"COMPLETED", "FAILED", "STOPPED"}, 5)
        self.assertEqual(status["state"], "FAILED", status)
        self.assertIn("FRESH_FEEDBACK_NOT_RESTORED", status["reason"])
        self.assertEqual(coordinator.execute_count, 0)

    def test_checker_error_is_never_treated_as_fresh(self):
        mgr, coordinator, _checker, _ = manager_with(lambda c: ScriptedChecker(c, raise_error=True))
        mgr.start(mode="count", count=2, operator_confirmed=True)
        status = await_state(mgr, {"COMPLETED", "FAILED", "STOPPED"}, 5)
        self.assertEqual(status["state"], "FAILED", status)
        self.assertIn("FRESH_FEEDBACK_CHECK_ERROR", status["reason"])
        self.assertEqual(coordinator.execute_count, 0)

    def test_stale_after_first_cycle_disarms_recording_and_stops(self):
        class StaleAfterFirst(ScriptedChecker):
            def __call__(self):
                with self._lock:
                    self.calls += 1
                    return dict(FRESH if self.coordinator.execute_count == 0 else STALE)

        mgr, coordinator, _checker, aborts = manager_with(lambda c: StaleAfterFirst(c))
        armed, finished, disarmed = [], [], []

        def arm(metadata):
            run_id = "run-%02d" % (len(armed) + 1)
            armed.append(run_id)
            return {"run_id": run_id}

        mgr.arm_recording = arm
        mgr.finish_recording = lambda run_id, tid, result: finished.append(run_id) or True
        mgr.disarm_recording = lambda run_id: disarmed.append(run_id)
        mgr.start(mode="count", count=4, operator_confirmed=True, recording={"label": "T"})
        status = await_state(mgr, {"COMPLETED", "FAILED", "STOPPED"}, 5)
        self.assertEqual(status["state"], "FAILED", status)
        self.assertEqual(coordinator.execute_count, 1)
        self.assertEqual(status["saved_cycles"], 1)
        self.assertEqual(finished, ["run-01"])
        self.assertEqual(disarmed, ["run-02"])  # armed for cycle 2, never executed
        self.assertFalse(aborts)

    def test_cancel_during_wait_stops_without_execute(self):
        mgr, coordinator, _checker, _ = manager_with(
            lambda c: ScriptedChecker(c, always=STALE), timeout=5)
        started = mgr.start(mode="count", count=3, operator_confirmed=True)
        deadline = time.monotonic() + 2
        while mgr.status()["loop"].get("gate_wait") != "FRESH_FEEDBACK":
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.005)
        mgr.cancel_after_current(started["id"], started["lease_token"])
        status = await_state(mgr, {"COMPLETED", "FAILED", "STOPPED", "CANCELLED"}, 3)
        self.assertEqual(status["state"], "STOPPED", status)
        self.assertIn("CANCELLED", status["reason"])
        self.assertEqual(coordinator.execute_count, 0)


class FreshFeedbackWiringTests(unittest.TestCase):
    def test_backend_passes_existing_start_match_checker(self):
        source = (APP / "backend" / "dual_jaka_web_backend.py").read_text(encoding="utf-8")
        block = source[source.index("self.phase5_loop_manager = Phase5LoopSupervisor("):]
        block = block[:block.index("\n        )\n")]
        self.assertIn("fresh_feedback_checker=self.phase5_start_match_state", block)

    def test_start_interlock_threshold_not_relaxed(self):
        source = (APP / "backend" / "phase5_supervised_loop.py").read_text(encoding="utf-8")
        self.assertIn("FRESH_FEEDBACK_WINDOW_SECONDS = 0.5", source)
        self.assertIn("FRESH_FEEDBACK_TIMEOUT_SECONDS = 10.0", source)

    def test_hmi_skips_auto_load_while_loop_active(self):
        js = (APP / "web" / "digital_twin_experiment.js").read_text(encoding="utf-8")
        self.assertIn("if (supervisedLoopActive())", js)
        self.assertIn("window.dualArmPhase5Loop?.active === true", js)
        self.assertNotIn("/api/digital-twin/phase5", js)
        loop_js = (APP / "web" / "digital_twin_phase5_execution.js").read_text(encoding="utf-8")
        self.assertIn("active: LOOP_ACTIVE_STATES.has(loop.state)", loop_js)
        main_js = (APP / "web" / "digital_twin.js").read_text(encoding="utf-8")
        self.assertIn("digital_twin_phase5_execution.js?v=cycle-recording-2", main_js)
        html = (APP / "web" / "index.html").read_text(encoding="utf-8")
        self.assertIn("digital_twin_experiment.js?v=cycle-recording-2", html)
        self.assertIn("/web-assets/digital_twin.js?v=cycle-recording-2", html)


if __name__ == "__main__":
    unittest.main()
