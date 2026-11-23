"""Offline tests for the EXP-3 controller-inferred time-base comparison (no ROS, no robot)."""
import math
import random
import unittest
from unittest import mock

from dual_arm_app.backend import experimental_controller_timebase as tb

START_NS = 1_800_000_000_000_000_000


def plan(duration=4.0, step=0.024):
    n = int(duration / step) + 1
    samples = []
    for i in range(n):
        t = i * step
        q = [0.3 * t, -0.2 * t, 0.1 * t, 0.05 * t, 0.0, 0.25 * t]
        samples.append({"time_from_start_s": t, "left": {"joints_rad": list(q)},
                        "right": {"joints_rad": [-v for v in q]}})
    return samples


def q_at(side, t):
    t = min(max(t, 0.0), 4.0)
    q = [0.3 * t, -0.2 * t, 0.1 * t, 0.05 * t, 0.0, 0.25 * t]
    return q if side == "left" else [-v for v in q]


def raw_with_packets(outlier=None, jitter_ms=25.0, seed=3):
    rng = random.Random(seed)
    packets = {}
    for side in ("left", "right"):
        series = []
        tau = 0.3
        index = 0
        while tau < 3.7:
            lag = 0.035
            if outlier == (side, index):
                lag = 0.085
            host = tau + 0.090 + rng.uniform(0, jitter_ms) / 1000.0
            series.append({"controller_joint_position_rad": q_at(side, tau),
                           "actual_joints_rad": q_at(side, tau - lag),
                           "received_at_ms": START_NS / 1e6 + host * 1000.0,
                           "received_monotonic_ns": int(5e9 + host * 1e9),
                           "packet_sequence_index": index + 1})
            tau += rng.choice((0.064, 0.096, 0.128))
            index += 1
        packets[side] = series
    return {"run_id": "exp-test", "planned": {"samples": plan()},
            "execution": {"start_time_unix_ns": START_NS, "duration_s": 4.0},
            "actual_packets": packets}


class ControllerTimebaseTests(unittest.TestCase):
    def run_with_stub(self, raw):
        calls = []

        def stub(raw_in, manifest, urdf):
            calls.append(raw_in)
            return {"status": "COMPLETE", "coverage": {"usable_fraction": 1.0},
                    "relative_translation": {"magnitude_m": {"rms": 0.001, "max": 0.002,
                                                             "max_location": {"time_from_start_s": 1.0}}},
                    "relative_orientation": {"angle": {"rms_deg": 0.1, "max_deg": 0.2}},
                    "samples": [{"time_from_start_s": 1.0, "relative_translation_error_magnitude_m": 0.001,
                                 "relative_orientation_error_deg": 0.1}]}

        with mock.patch.object(tb, "_analyze_host_time", side_effect=stub):
            result = tb.analyze_controller_timebase(raw, {}, "unused.urdf")
        return result, calls

    def test_flags_only_the_injected_lag_outlier(self):
        result, calls = self.run_with_stub(raw_with_packets(outlier=("right", 10)))
        self.assertEqual(result["status"], "COMPLETE")
        flagged = [(s, f["packet_index"]) for s in ("left", "right") for f in result["sides"][s]["flagged_packets"]]
        self.assertEqual(flagged, [("right", 10)])
        deviation = result["sides"]["right"]["flagged_packets"][0]["deviation_ms"]
        self.assertAlmostEqual(deviation, 50.0, delta=2.0)
        self.assertEqual(result["sides"]["right"]["flagged_packets"][0]["estimated_stale_cycles_8ms"], 6)
        self.assertIn(round(result["sides"]["right"]["flagged_packets"][0]["controller_spacing_before_ms"]), (64, 96, 128))
        self.assertAlmostEqual(result["sides"]["left"]["tracking_lag_ms"]["median"], 35.0, delta=1.0)
        self.assertEqual(len(calls), 3)  # host, controller, controller excluding flagged
        self.assertEqual(len(calls[2]["actual_packets"]["right"]), len(calls[1]["actual_packets"]["right"]) - 1)

    def test_retimed_packets_follow_inferred_controller_time_and_input_is_untouched(self):
        raw = raw_with_packets()
        original_first = dict(raw["actual_packets"]["left"][0])
        result, calls = self.run_with_stub(raw)
        self.assertEqual(raw["actual_packets"]["left"][0], original_first)  # no mutation of evidence
        self.assertAlmostEqual(result["common_latency_ms"], 102.5, delta=6.0)  # 90 ms + mean jitter
        retimed = calls[1]["actual_packets"]["left"]
        times = [p["received_at_ms"] for p in retimed]
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        # spacing of retimed packets equals the true controller spacing (64/96/128 ms), not host jitter
        gaps = [round(b - a) for a, b in zip(times, times[1:])]
        self.assertTrue(all(any(abs(g - s) <= 1 for s in (64, 96, 128)) for g in gaps), gaps)
        self.assertGreater(result["sides"]["left"]["host_minus_controller_time_ms"]["std"], 5.0)

    def test_unavailable_without_packets(self):
        result = tb.analyze_controller_timebase({"planned": {"samples": plan()}, "execution": {}}, {}, "x")
        self.assertEqual(result["status"], "UNAVAILABLE")


class TimebaseBackendWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def node(self, recorder_state, loads):
        from types import SimpleNamespace
        return SimpleNamespace(
            experiment_recorder=SimpleNamespace(state=lambda: {"state": recorder_state}),
            experiment_store=SimpleNamespace(load=lambda run_id: loads.append(run_id) or {"raw": {}, "manifest": {}}),
        )

    def test_refused_while_recorder_active_without_loading_run(self):
        for state in ("ARMED", "RECORDING", "FINALIZING"):
            loads = []
            with self.subTest(state=state), self.assertRaises(self.backend.ExperimentError):
                self.backend.DualJakaWebNode.analyze_experiment_run_timebase(self.node(state, loads), "exp-id")
            self.assertEqual(loads, [])

    def test_idle_route_is_get_only_and_not_motion(self):
        loads = []
        result = self.backend.DualJakaWebNode.analyze_experiment_run_timebase(self.node("IDLE", loads), "exp-id")
        self.assertTrue(result["ok"])
        self.assertEqual(loads, ["exp-id"])
        self.assertEqual(result["timebase"]["status"], "UNAVAILABLE")  # empty raw evidence
        self.assertFalse(self.backend._d33_is_motion_command("/api/experiments/exp-id/exp3-timebase", "GET"))
        route = next(r for r in self.backend.app.routes
                     if getattr(r, "path", None) == "/api/experiments/{run_id}/exp3-timebase")
        self.assertEqual(route.methods, {"GET"})


if __name__ == "__main__":
    unittest.main()
