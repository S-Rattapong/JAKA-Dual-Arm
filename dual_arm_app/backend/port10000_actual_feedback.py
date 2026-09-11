"""Read-only JAKA port-10000 actual-joint feedback for visualization.

This module deliberately imports no ROS, FastAPI, or JAKA SDK package.  Its
stream parser and source-selection policy are pure and offline-testable.  The
network receiver connects only to the receive-only push port and never sends
application data to a robot controller.
"""

from __future__ import annotations

import json
import math
import re
import socket
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any


JOINT_COUNT = 6
DEFAULT_PORT = 10000
DEFAULT_EXPECTED_PERIOD_MS = 50
DEFAULT_FRESHNESS_THRESHOLD_MS = 250
DEFAULT_MAX_PACKET_BYTES = 64 * 1024
DEFAULT_STALE_RECONNECT_TIMEOUT_S = 2.0
PORT10000_SOURCE = "jaka_port10000_actual_feedback"
ROS_FALLBACK_SOURCE = "ros_joint_state_cache"
_PACKET_PREFIX = re.compile(rb'^\{"len"\s*:\s*(\d+)\s*,')
_PACKET_MARKER = b'{"len"'


def wall_clock_ms() -> int:
    return time.time_ns() // 1_000_000


class Port10000StreamParser:
    """Frame length-prefixed JSON objects across arbitrary TCP recv chunks."""

    def __init__(self, max_packet_bytes: int = DEFAULT_MAX_PACKET_BYTES):
        if max_packet_bytes < 128:
            raise ValueError("max_packet_bytes must be at least 128")
        self.max_packet_bytes = int(max_packet_bytes)
        self._buffer = bytearray()
        self.last_error: str | None = None
        self.discarded_packet_count = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def _discard(self, message: str) -> None:
        self.last_error = message
        self.discarded_packet_count += 1

    def feed(self, chunk: bytes | bytearray | memoryview) -> list[dict[str, Any]]:
        if not isinstance(chunk, (bytes, bytearray, memoryview)):
            raise TypeError("port10000 chunk must be bytes-like")
        self._buffer.extend(bytes(chunk))
        packets: list[dict[str, Any]] = []

        while self._buffer:
            while self._buffer and self._buffer[0] in b" \t\r\n":
                del self._buffer[0]
            if not self._buffer:
                break

            if not self._buffer.startswith(_PACKET_MARKER):
                marker_index = self._buffer.find(_PACKET_MARKER, 1)
                if marker_index >= 0:
                    del self._buffer[:marker_index]
                    self._discard("discarded bytes before port10000 packet prefix")
                    continue
                if len(self._buffer) > 64:
                    self._buffer.clear()
                    self._discard("malformed port10000 packet prefix")
                break

            header = _PACKET_PREFIX.match(self._buffer[:64])
            if header is None:
                if len(self._buffer) >= 64:
                    del self._buffer[0]
                    self._discard("malformed port10000 length header")
                    continue
                break

            packet_length = int(header.group(1))
            if packet_length < header.end() + 1 or packet_length > self.max_packet_bytes:
                self._buffer.clear()
                self._discard(
                    f"port10000 packet length {packet_length} is outside allowed bounds"
                )
                break
            if len(self._buffer) < packet_length:
                break

            frame = bytes(self._buffer[:packet_length])
            del self._buffer[:packet_length]
            try:
                decoded = json.loads(frame.decode("utf-8"))
                if not isinstance(decoded, dict):
                    raise ValueError("port10000 frame must decode to an object")
                if decoded.get("len") != packet_length:
                    raise ValueError("port10000 decoded length does not match frame")
                packets.append(decoded)
                self.last_error = None
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                self._discard(f"malformed port10000 JSON frame: {error}")
        return packets


def finite_sequence(raw: Any, field: str, count: int | None = None) -> list[float]:
    """Copy finite numbers; joint fields consume only their first six values."""
    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence) or len(raw) < (count or 1):
        raise ValueError(f"{field} must contain at least {count or 1} finite values")
    result = []
    for index, value in enumerate(raw[:count] if count is not None else raw):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field}[{index}] must be finite")
        try:
            number = float(value)
        except OverflowError as error:
            raise ValueError(f"{field}[{index}] must be finite") from error
        if not math.isfinite(number):
            raise ValueError(f"{field}[{index}] must be finite")
        result.append(number)
    return result


def extract_actual_joints_radians(packet: Mapping[str, Any]) -> list[float]:
    """Validate joint_actual_position and convert its first six deg values once."""
    if not isinstance(packet, Mapping):
        raise ValueError("port10000 packet must be an object")
    return [math.radians(v) for v in finite_sequence(packet.get("joint_actual_position"), "joint_actual_position", 6)]


def _valid_fresh_cache(
    value: Mapping[str, Any], now_ms: int, freshness_threshold_ms: int
) -> bool:
    joint = value.get("joint")
    received_at_ms = value.get("received_at_ms")
    return bool(
        isinstance(joint, (list, tuple))
        and len(joint) == JOINT_COUNT
        and all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in joint
        )
        and not isinstance(received_at_ms, bool)
        and isinstance(received_at_ms, (int, float))
        and math.isfinite(float(received_at_ms))
        and max(0, now_ms - int(received_at_ms)) <= freshness_threshold_ms
    )


def select_visualization_joint_status(
    port10000_cache: Mapping[str, Mapping[str, Any]],
    ros_joint_status: Mapping[str, Any],
    *,
    server_time_ms: int,
    expected_period_ms: int = DEFAULT_EXPECTED_PERIOD_MS,
    freshness_threshold_ms: int = DEFAULT_FRESHNESS_THRESHOLD_MS,
) -> dict[str, Any]:
    """Prefer port10000 only as an all-or-nothing fresh dual-arm snapshot."""
    now_ms = int(server_time_ms)
    freshness_ms = int(freshness_threshold_ms)
    expected_ms = int(expected_period_ms)
    if expected_ms <= 0:
        raise ValueError("expected_period_ms must be positive")
    if freshness_ms < 0:
        raise ValueError("freshness_threshold_ms must be non-negative")
    fresh = {
        side: _valid_fresh_cache(
            port10000_cache.get(side, {}), now_ms, freshness_ms
        )
        for side in ("left", "right")
    }
    diagnostics = {
        "preferred_source": PORT10000_SOURCE,
        "fallback_source": ROS_FALLBACK_SOURCE,
        "freshness_threshold_ms": freshness_ms,
        "expected_period_ms": expected_ms,
        "both_port10000_arms_fresh": fresh["left"] and fresh["right"],
        "port10000": {},
    }
    for side in ("left", "right"):
        cached = port10000_cache.get(side, {})
        received = cached.get("received_at_ms")
        valid_timestamp = (
            not isinstance(received, bool)
            and isinstance(received, (int, float))
            and math.isfinite(float(received))
        )
        diagnostics["port10000"][side] = {
            "fresh": fresh[side],
            "received_at_ms": int(received) if valid_timestamp else None,
            "age_ms": max(0, now_ms - int(received)) if valid_timestamp else None,
            "error": cached.get("error"),
        }

    if fresh["left"] and fresh["right"]:
        response: dict[str, Any] = {
            "ok": True,
            "source": PORT10000_SOURCE,
            "unit": "radian",
            "server_time_ms": now_ms,
            "visualization_only": True,
            "source_diagnostics": diagnostics,
        }
        for side in ("left", "right"):
            cached = port10000_cache[side]
            received = int(cached["received_at_ms"])
            response[side] = {
                "joint": [float(value) for value in cached["joint"]],
                "received_at_ms": received,
                "age_ms": max(0, now_ms - received),
                "valid": True,
                "mapping": "port10000_joint_actual_position_deg_to_rad",
                "error": None,
                "names": [],
            }
        return response

    response = dict(ros_joint_status)
    for side in ("left", "right"):
        if isinstance(response.get(side), Mapping):
            response[side] = dict(response[side])
    response["source"] = ROS_FALLBACK_SOURCE
    response["visualization_only"] = True
    response["source_diagnostics"] = diagnostics
    return response


class Port10000ActualFeedback:
    """Two daemon receivers and their thread-safe visualization-only cache."""

    def __init__(
        self,
        endpoints: Mapping[str, tuple[str, int]],
        *,
        max_packet_bytes: int = DEFAULT_MAX_PACKET_BYTES,
        connect_timeout_s: float = 0.5,
        recv_timeout_s: float = 0.5,
        reconnect_backoff_s: float = 0.5,
        stale_reconnect_timeout_s: float = DEFAULT_STALE_RECONNECT_TIMEOUT_S,
        log_warning: Callable[[str], None] | None = None,
    ):
        self._endpoints = dict(endpoints)
        if set(self._endpoints) != {"left", "right"}:
            raise ValueError("port10000 endpoints must define left and right")
        self._max_packet_bytes = int(max_packet_bytes)
        self._connect_timeout_s = float(connect_timeout_s)
        self._recv_timeout_s = float(recv_timeout_s)
        self._reconnect_backoff_s = float(reconnect_backoff_s)
        self._stale_reconnect_timeout_s = float(stale_reconnect_timeout_s)
        if self._stale_reconnect_timeout_s <= 0.0:
            raise ValueError("stale_reconnect_timeout_s must be positive")
        self._log_warning = log_warning or (lambda _message: None)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._cache = {
            side: {
                "joint": None,
                "received_at_ms": None,
                "error": "No port10000 actual feedback received",
            }
            for side in ("left", "right")
        }
        self._threads: list[threading.Thread] = []
        self._packet_sequence = {side: 0 for side in ("left", "right")}
        self._packet_observers: list[Callable[[dict[str, Any]], None]] = []

    def start(self) -> None:
        if self._threads:
            return
        for side in ("left", "right"):
            thread = threading.Thread(
                target=self._receive_loop,
                args=(side,),
                name=f"jaka-{side}-port10000-feedback",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()

    def stop(self) -> None:
        self._stop.set()

    def snapshot(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            return {
                side: {
                    "joint": list(value["joint"]) if value["joint"] else None,
                    "received_at_ms": value["received_at_ms"],
                    "error": value["error"],
                }
                for side, value in self._cache.items()
            }

    def _record_error(self, side: str, message: str) -> None:
        with self._lock:
            self._cache[side]["error"] = message

    def subscribe_packets(self, observer: Callable[[dict[str, Any]], None]) -> None:
        """Subscribe an in-memory-only callback; callbacks must never perform I/O.

        Called outside the cache lock. Failures cannot interrupt visualization or
        reconnect handling. Registration is independent of receiver startup.
        """
        with self._lock:
            if observer not in self._packet_observers:
                self._packet_observers.append(observer)

    def _record_packet(self, side: str, packet: Mapping[str, Any]) -> None:
        joints = extract_actual_joints_radians(packet)
        received = wall_clock_ms()
        monotonic_ns = time.monotonic_ns()
        fields = {}
        validation = {"joint_actual_position": "VALID"}
        for source, target, count in (
            ("joint_position", "controller_joint_position_rad", 6),
            ("position", "controller_position_raw", None),
            ("actual_position", "controller_actual_position_raw", None),
        ):
            try:
                values = finite_sequence(packet.get(source), source, count)
                fields[target] = [math.radians(v) for v in values] if count else values
                validation[source] = "VALID"
            except ValueError:
                fields[target] = None
                validation[source] = "MISSING" if source not in packet else "INVALID"
        length = packet.get("len")
        valid_length = isinstance(length, int) and not isinstance(length, bool) and length > 0
        validation["len"] = "VALID" if valid_length else ("MISSING" if "len" not in packet else "INVALID")
        with self._lock:
            self._packet_sequence[side] += 1
            sequence = self._packet_sequence[side]
            self._cache[side] = {
                "joint": joints,
                "received_at_ms": received,
                "error": None,
            }

            observers = tuple(self._packet_observers)
        for observer in observers:
            try:
                observer({"side": side, "joints_rad": list(joints),
                          "actual_joints_rad": list(joints),
                          **{key: list(value) if value is not None else None for key, value in fields.items()},
                          "field_validation": dict(validation),
                          "packet_length_bytes": length if valid_length else None,
                          "packet_sequence_index": sequence,
                          "received_monotonic_ns": monotonic_ns,
                          "controller_joint_position_mapping": "same-frame joint_position first six degrees converted once to radians",
                          "controller_pose_semantic": "same-frame position/actual_position: controller-native raw; units and meaning unverified",
                          "tracking_semantic": "controller-reported same-frame discrepancy; not necessarily original Phase5 command; not external ground truth",
                          "received_at_ms": received, "source": PORT10000_SOURCE,
                          "mapping": "port10000_joint_actual_position_deg_to_rad"})
            except Exception as error:
                self._log_warning(f"Port10000 packet observer failed: {error}")

    @staticmethod
    def _stream_stale(
        last_packet_monotonic_s: float,
        now_monotonic_s: float,
        timeout_s: float,
    ) -> bool:
        return now_monotonic_s - last_packet_monotonic_s >= timeout_s

    def _receive_loop(self, side: str) -> None:
        endpoint = self._endpoints[side]
        while not self._stop.is_set():
            parser = Port10000StreamParser(self._max_packet_bytes)
            try:
                with socket.create_connection(
                    endpoint, timeout=self._connect_timeout_s
                ) as feedback_socket:
                    feedback_socket.settimeout(self._recv_timeout_s)
                    last_packet_monotonic_s = time.monotonic()
                    while not self._stop.is_set():
                        try:
                            chunk = feedback_socket.recv(16 * 1024)
                        except socket.timeout:
                            if self._stream_stale(
                                last_packet_monotonic_s,
                                time.monotonic(),
                                self._stale_reconnect_timeout_s,
                            ):
                                raise ConnectionError(
                                    "port10000 stream stale: no valid packet for "
                                    f"{self._stale_reconnect_timeout_s:.3f} s"
                                )
                            continue
                        if not chunk:
                            raise ConnectionError("port10000 connection closed")
                        for packet in parser.feed(chunk):
                            try:
                                self._record_packet(side, packet)
                                last_packet_monotonic_s = time.monotonic()
                            except ValueError as error:
                                self._record_error(side, str(error))
                        if parser.last_error:
                            self._record_error(side, parser.last_error)
                        if self._stream_stale(
                            last_packet_monotonic_s,
                            time.monotonic(),
                            self._stale_reconnect_timeout_s,
                        ):
                            raise ConnectionError(
                                "port10000 stream stale: no valid packet for "
                                f"{self._stale_reconnect_timeout_s:.3f} s"
                            )
            except (OSError, ConnectionError, ValueError) as error:
                message = f"{side} port10000 actual feedback unavailable: {error}"
                self._record_error(side, message)
                self._log_warning(message)
            self._stop.wait(self._reconnect_backoff_s)
