"""Offline tests for receive-only JAKA port-10000 actual feedback."""

from __future__ import annotations

import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from dual_arm_app.backend.port10000_actual_feedback import (
    PORT10000_SOURCE,
    Port10000ActualFeedback,
    Port10000StreamParser,
    extract_actual_joints_radians,
    select_visualization_joint_status,
)


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
RECEIVER = ROOT / "dual_arm_app/backend/port10000_actual_feedback.py"
CONFIG = ROOT / "dual_arm_app/config/robots.yaml"


def packet_bytes(joints=None, **extra):
    payload = {
        "joint_actual_position": joints or [0, 30, -45, 90, 180, -180],
        **extra,
    }
    # The decimal digit count of len can change the length itself.
    declared = 0
    while True:
        payload = {"len": declared, **payload}
        encoded = json.dumps(payload, separators=(",", ":")).encode()
        if len(encoded) == declared:
            return encoded
        declared = len(encoded)
        payload.pop("len")


class Port10000ParserTests(unittest.TestCase):
    def test_full_and_split_packet(self):
        frame = packet_bytes()
        full = Port10000StreamParser().feed(frame)
        self.assertEqual(len(full), 1)
        parser = Port10000StreamParser()
        self.assertEqual(parser.feed(frame[:17]), [])
        self.assertEqual(parser.feed(frame[17:]), full)

    def test_sticky_two_packets(self):
        first = packet_bytes([1, 2, 3, 4, 5, 6], sequence=1)
        second = packet_bytes([7, 8, 9, 10, 11, 12], sequence=2)
        packets = Port10000StreamParser().feed(first + second)
        self.assertEqual([item["sequence"] for item in packets], [1, 2])

    def test_malformed_and_oversize_fail_safe(self):
        valid = packet_bytes()
        malformed = b'{"len":20,"bad":xxxxx}'
        parser = Port10000StreamParser()
        packets = parser.feed(malformed + valid)
        self.assertEqual(packets, [json.loads(valid)])
        self.assertGreater(parser.discarded_packet_count, 0)

        oversize = Port10000StreamParser(max_packet_bytes=128)
        self.assertEqual(oversize.feed(b'{"len":999999,'), [])
        self.assertIn("outside allowed bounds", oversize.last_error)
        self.assertEqual(oversize.buffered_bytes, 0)

    def test_missing_and_nonfinite_joint_values_rejected(self):
        invalid = (
            {},
            {"joint_actual_position": [1, 2, 3, 4, 5]},
            {"joint_actual_position": [1, 2, 3, 4, 5, math.nan]},
            {"joint_actual_position": [1, 2, 3, 4, 5, True]},
        )
        for packet in invalid:
            with self.subTest(packet=packet), self.assertRaises(ValueError):
                extract_actual_joints_radians(packet)

    def test_degrees_convert_to_radians_exactly_once_and_uses_first_six(self):
        result = extract_actual_joints_radians(
            {"joint_actual_position": [0, 30, -45, 90, 180, -180, 999]}
        )
        expected = [math.radians(value) for value in [0, 30, -45, 90, 180, -180]]
        self.assertEqual(result, expected)


class _FakeFeedbackSocket:
    def __init__(self, chunks):
        self._chunks = iter(chunks)
        self.timeout = None

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        return False

    def settimeout(self, timeout):
        self.timeout = timeout

    def recv(self, _size):
        return next(self._chunks)


def packet_without_actual_joints_bytes(sequence):
    payload = {"sequence": sequence, "not_actual_joints": [1, 2, 3, 4, 5, 6]}
    declared = 0
    while True:
        candidate = {"len": declared, **payload}
        encoded = json.dumps(candidate, separators=(",", ":")).encode()
        if len(encoded) == declared:
            return encoded
        declared = len(encoded)


class Port10000RecoveryTests(unittest.TestCase):
    def test_stale_stream_watchdog_boundary(self):
        self.assertFalse(Port10000ActualFeedback._stream_stale(10.0, 11.999, 2.0))
        self.assertTrue(Port10000ActualFeedback._stream_stale(10.0, 12.0, 2.0))

    def test_continuous_invalid_frames_trigger_stale_reconnect_without_socket_timeout(self):
        endpoints = {
            "left": ("127.0.0.1", 10000),
            "right": ("127.0.0.1", 10000),
        }
        warnings = []
        receiver = None

        def log_warning(message):
            warnings.append(message)
            receiver.stop()

        receiver = Port10000ActualFeedback(
            endpoints,
            reconnect_backoff_s=0.0,
            stale_reconnect_timeout_s=1.0,
            log_warning=log_warning,
        )
        fake_socket = _FakeFeedbackSocket(
            [
                packet_without_actual_joints_bytes(1),
                packet_without_actual_joints_bytes(2),
            ]
        )
        with patch(
            "dual_arm_app.backend.port10000_actual_feedback.socket.create_connection",
            return_value=fake_socket,
        ) as create_connection, patch(
            "dual_arm_app.backend.port10000_actual_feedback.time.monotonic",
            side_effect=[10.0, 10.6, 11.1],
        ):
            receiver._receive_loop("left")

        create_connection.assert_called_once()
        self.assertEqual(fake_socket.timeout, 0.5)
        self.assertTrue(warnings)
        self.assertIn("stream stale: no valid packet for 1.000 s", warnings[0])
        snapshot = receiver.snapshot()["left"]
        self.assertIsNone(snapshot["joint"])
        self.assertIn("actual feedback unavailable", snapshot["error"])

    def test_stale_reconnect_timeout_must_be_positive(self):
        endpoints = {"left": ("127.0.0.1", 10000), "right": ("127.0.0.1", 10000)}
        with self.assertRaisesRegex(ValueError, "stale_reconnect_timeout_s"):
            Port10000ActualFeedback(endpoints, stale_reconnect_timeout_s=0.0)


class Port10000SelectionTests(unittest.TestCase):
    @staticmethod
    def ros_status():
        return {
            "ok": True,
            "source": "ros_joint_state_cache",
            "unit": "radian",
            "server_time_ms": 1000,
            "left": {"joint": [1] * 6, "valid": True, "received_at_ms": 900},
            "right": {"joint": [-1] * 6, "valid": True, "received_at_ms": 900},
        }

    def test_both_fresh_select_port10000(self):
        cache = {
            "left": {"joint": [0.1] * 6, "received_at_ms": 900, "error": None},
            "right": {"joint": [-0.1] * 6, "received_at_ms": 800, "error": None},
        }
        result = select_visualization_joint_status(
            cache, self.ros_status(), server_time_ms=1000, freshness_threshold_ms=250
        )
        self.assertEqual(result["source"], PORT10000_SOURCE)
        self.assertEqual(result["left"]["joint"], [0.1] * 6)
        self.assertEqual(result["right"]["age_ms"], 200)
        self.assertTrue(result["source_diagnostics"]["both_port10000_arms_fresh"])
        self.assertEqual(result["source_diagnostics"]["expected_period_ms"], 50)

    def test_one_stale_or_invalid_falls_back_to_ros_unchanged(self):
        cache = {
            "left": {"joint": [0.1] * 6, "received_at_ms": 900, "error": None},
            "right": {"joint": [-0.1] * 6, "received_at_ms": 749, "error": None},
        }
        ros = self.ros_status()
        result = select_visualization_joint_status(
            cache, ros, server_time_ms=1000, freshness_threshold_ms=250
        )
        self.assertEqual(result["source"], "ros_joint_state_cache")
        self.assertEqual(result["left"], ros["left"])
        self.assertFalse(result["source_diagnostics"]["both_port10000_arms_fresh"])


class Port10000IntegrationContractTests(unittest.TestCase):
    def test_backend_receiver_is_receive_only_and_sdk_free(self):
        source = RECEIVER.read_text(encoding="utf-8")
        self.assertIn("feedback_socket.recv", source)
        self.assertIn("port10000 stream stale: no valid packet", source)
        self.assertIn("time.monotonic()", source)
        self.assertNotIn(".send(", source)
        for forbidden in ("JAKAZuRobot", "login_in", "10001", "rclpy"):
            self.assertNotIn(forbidden, source)

    def test_phase5_authority_remains_on_ros_cache(self):
        source = BACKEND.read_text(encoding="utf-8")
        for method in (
            "phase5_start_match_state",
            "phase5_replan_from_current",
            "phase5_move_to_initial",
            "phase5_execution_state",
            "phase5_driver_execution_feedback",
        ):
            body = source.split(f"    def {method}", 1)[1].split("\n    def ", 1)[0]
            self.assertIn("digital_twin_ros_joint_status()", body)
            self.assertNotIn("port10000_actual_feedback", body)

    def test_receive_only_dual_arm_feedback_is_enabled_for_visualization(self):
        source = CONFIG.read_text(encoding="utf-8")
        block = source.split("actual_feedback:", 1)[1].split("motion:", 1)[0]
        self.assertIn("port10000:", block)
        self.assertIn("enabled: true", block)
        self.assertIn("expected_period_ms: 100", block)
        self.assertIn("freshness_threshold_ms: 400", block)
        self.assertIn("stale_reconnect_timeout_s: 2.0", block)
        for text in (
            "left_ip: 192.168.0.1",
            "right_ip: 192.168.0.2",
            "port: 10000",
        ):
            self.assertIn(text, block)


if __name__ == "__main__":
    unittest.main()
