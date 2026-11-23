"""Batch EXP2 endpoint report from saved run files (OFFLINE ONLY).

Reads each RUN/raw.json + optional manifest.json, computes per-arm host bounds
and R-L intervals. Original run files are read-only. No ROS/network imports.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from tools.exp2_host_endpoints import analyze, plot_samples
from tools.exp2_event_zoom import plot_zooms


def relative_pair(pair, origin):
    if pair is None or origin is None:
        return ("", "")
    return (round(pair[0] - origin, 4), round(pair[1] - origin, 4))


def pair(pair):
    if pair is None:
        return ("", "", "")
    return (round(pair[0], 4), round(pair[1], 4), round(pair[1] - pair[0], 4))


def render_row(raw, manifest, *, resolution_ms=None):
    report = analyze(raw)
    evidence = raw.get("endpoint_evidence") or {}
    origin = evidence.get("bind_observed_ns")
    if origin is not None:
        origin /= 1e6

    row = {
        "label": manifest.get("label", ""),
        "run_id": raw.get("run_id", ""),
        "terminal_status": report.get("terminal_status") or "",
        "capture_outcome": report.get("capture_outcome") or "",
        "clock": report.get("clock"),
        "right_minus_left_semantics": "positive=right later, negative=right earlier",
    }
    for event in ("start", "stop"):
        for side in ("left", "right"):
            observed = report["arms"][side][f"{event}_host_ms"]
            lo, hi = relative_pair(observed, origin)
            row[f"{event}_{side}_since_bind_lower_ms"] = lo
            row[f"{event}_{side}_since_bind_upper_ms"] = hi
        lo, hi, width = pair(report.get(f"{event}_R_minus_L_host_ms"))
        row[f"{event}_R_minus_L_lower_ms"] = lo
        row[f"{event}_R_minus_L_upper_ms"] = hi
        row[f"{event}_interval_width_ms"] = width
        row[f"{event}_ordering"] = report.get(f"{event}_classification", "")
        if resolution_ms is None:
            row[f"{event}_precision_assessment"] = "NO_PRESET_RESOLUTION"
        elif width == "":
            row[f"{event}_precision_assessment"] = "INCOMPLETE"
        elif width > resolution_ms:
            row[f"{event}_precision_assessment"] = "INSUFFICIENT_RESOLUTION"
        elif row[f"{event}_ordering"] == "INCLUDES_ZERO":
            row[f"{event}_precision_assessment"] = "ORDER_UNRESOLVED_ZERO_INCLUDED"
        else:
            row[f"{event}_precision_assessment"] = "ORDER_RESOLVED_WITHIN_RESOLUTION"

    dispatch = evidence.get("driver_dispatch_summary", {}).get("arms", {})
    for side in ("left", "right"):
        arm = report["arms"][side]
        row[f"{side}_reasons"] = "|".join(arm.get("reasons", []))
        row[f"{side}_receive_gap_max_ms"] = arm.get("receive_gap_ms", {}).get("max")
        row[f"{side}_receive_gap_count"] = arm.get("receive_gap_ms", {}).get("count")
        row[f"{side}_receive_to_recorder_max_ms"] = arm.get("receive_to_delivery_ms", {}).get("max")
        row[f"{side}_recorder_append_max_ms"] = arm.get("delivery_to_append_ms", {}).get("max")
        driver = dispatch.get(side) or {}
        row[f"{side}_driver_status"] = driver.get("status") or "UNAVAILABLE"
        d = driver.get("dispatch_timing") or {}
        servo = driver.get("servo_j_call_duration") or {}
        stream = driver.get("servo_stream") or {}
        row[f"{side}_dispatch_missed_cycles"] = d.get("missed_cycle_count", "")
        row[f"{side}_dispatch_max_lateness_ms"] = d.get("max_abs_lateness_ms", "")
        row[f"{side}_dispatch_p95_lateness_ms"] = d.get("p95_abs_lateness_ms", "")
        row[f"{side}_servo_overrun_count"] = servo.get("overrun_count", "")
        row[f"{side}_servo_p95_call_ms"] = servo.get("p95_ms", "")
        row[f"{side}_servo_period_ms"] = stream.get("command_period_ms", "")
    return row


def build(input_dir, *, label_prefix="", resolution_ms=None, plots_dir=None):
    result = []
    if plots_dir is not None:
        plots_dir.mkdir(parents=True, exist_ok=True)
    for rawfile in sorted(input_dir.glob("*/raw.json")):
        folder = rawfile.parent
        manifestfile = folder / "manifest.json"
        manifest = json.loads(manifestfile.read_text(encoding="utf-8")) if manifestfile.exists() else {}
        if label_prefix and not str(manifest.get("label", "")).startswith(label_prefix):
            continue
        raw = json.loads(rawfile.read_text(encoding="utf-8"))
        row = render_row(raw, manifest, resolution_ms=resolution_ms)
        result.append(row)
        if plots_dir is not None and raw.get("endpoint_evidence"):
            name = str(raw.get("run_id", folder.name))
            # run_id is not trusted as a filesystem path
            safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
            plot_samples(raw, plots_dir / f"{safe}_overview.png", dict(angle=None, speed=None, hold=None, max_gap=None))
            plot_zooms(raw, plots_dir / f"{safe}_start_stop_zoom.png")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=Path, help="Directory containing run-id folders with raw.json")
    parser.add_argument("output_csv", type=Path, help="New CSV path; never overwrites")
    parser.add_argument("--label-prefix", default="")
    parser.add_argument("--resolution-ms", type=float, default=None,
                        help="Predefined maximum R-L interval width in ms; omitted = no resolution decision")
    parser.add_argument("--plots-dir", type=Path, default=None,
                        help="Optional existing/new directory for plots of endpoint-evidence runs")
    args = parser.parse_args()
    if args.resolution_ms is not None and (not (0 < args.resolution_ms < float("inf"))):
        parser.error("--resolution-ms must be finite and positive")
    rows = build(args.run_root, label_prefix=args.label_prefix,
                 resolution_ms=args.resolution_ms, plots_dir=args.plots_dir)
    if not rows:
        parser.error("No matching saved run records")
    args.output_csv.parent.mkdir(exist_ok=True, parents=True)
    with args.output_csv.open("x", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({"ok": True, "runs": len(rows), "output": str(args.output_csv),
                      "start_order_resolved": sum(row["start_ordering"] in ("RIGHT_LATER", "RIGHT_EARLIER") for row in rows),
                      "stop_order_resolved": sum(row["stop_ordering"] in ("RIGHT_LATER", "RIGHT_EARLIER") for row in rows),
                      "note": "Host receive clock only, not controller physical timestamps"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
