"""Offline batch run-summary contracts; no robot/network access."""
import csv
import json
from pathlib import Path

from dual_arm_app.tests.test_exp2_endpoint_capture import simulate
from tools.exp2_batch_report import build, render_row


def test_batch_start_stop_and_resolution(tmp_path):
    run = simulate(tmp_path / "saved", right_delay=30, right_period=10)
    parent = run.parent
    rows = build(parent, resolution_ms=20)
    assert len(rows) == 1
    row = rows[0]
    assert row["start_ordering"] == "RIGHT_LATER"
    assert row["start_R_minus_L_lower_ms"] <= 30 <= row["start_R_minus_L_upper_ms"]
    assert row["start_precision_assessment"] == "ORDER_RESOLVED_WITHIN_RESOLUTION"
    assert row["stop_ordering"] == "INCLUDES_ZERO"
    assert row["stop_precision_assessment"] == "ORDER_UNRESOLVED_ZERO_INCLUDED"
    assert row["left_driver_status"] == "UNAVAILABLE"
    assert row["right_driver_status"] == "UNAVAILABLE"
    assert row["start_left_since_bind_upper_ms"] != ""
    assert row["stop_right_since_bind_upper_ms"] != ""


def test_batch_legacy_has_no_fabricated_events(tmp_path):
    run = simulate(tmp_path / "saved", enabled=False)
    raw = json.loads((run / "raw.json").read_text())
    row = render_row(raw, {}, resolution_ms=1)
    assert row["start_ordering"] == "INCOMPLETE"
    assert row["stop_ordering"] == "INCOMPLETE"
    assert row["start_precision_assessment"] == "INCOMPLETE"
    assert row["stop_precision_assessment"] == "INCOMPLETE"
    assert row["start_R_minus_L_lower_ms"] == ""
    assert row["left_driver_status"] == "UNAVAILABLE"


def test_event_zoom_uses_actual_host_samples(tmp_path):
    import struct
    from tools.exp2_event_zoom import plot_zooms
    run = simulate(tmp_path / "saved", right_delay=30, right_period=10)
    raw = json.loads((run / "raw.json").read_text())
    destination = tmp_path / "start_stop_zoom.png"
    result = plot_zooms(raw, destination)
    assert result["start_classification"] == "RIGHT_LATER"
    with destination.open("rb") as stream:
        header = stream.read(24)
    assert header[:8] == b"\x89PNG\r\n\x1a\n"
    w, h = struct.unpack(">II", header[16:24])
    assert w >= 1400 and h >= 900
    assert destination.stat().st_size >= 20000
