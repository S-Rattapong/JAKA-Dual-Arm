#!/usr/bin/env python3
"""Non-motion Measurement Qualification gate for JAKA Dual Arm.
Reads saved evidence, local backend GET endpoints, and optional offline tests.
Never sends robot commands, arms recorder, or mutates experiment evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "dual_arm_app" / "experiment_runs"
MODEL = ROOT / "dual_arm_app" / "web" / "assets" / "dual_jaka_a12_web.urdf"
DEFAULT_RUN = "exp-20260911T082849318Z-bb3bd4b5"
TESTS = [
    "dual_arm_app/tests/test_exp0_experimental_infrastructure.py",
    "dual_arm_app/tests/test_exp1_trajectory_tracking.py",
    "dual_arm_app/tests/test_exp2_measurement_pipeline.py",
    "dual_arm_app/tests/test_exp2_rigid_grasp.py",
    "dual_arm_app/tests/test_port10000_actual_feedback.py",
    "dual_arm_app/tests/test_mq_qualification.py",
]


def canonical_hash(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def check(name, passed, details, checks):
    checks.append({"name": name, "passed": bool(passed), "details": details})


def saved_run_gate(run_dir: Path):
    """Qualify V3 acquisition, saved provenance, and phase-matching on a run."""
    checks = []
    try:
        manifest = json.loads((run_dir / "manifest.json").read_text())
        raw = json.loads((run_dir / "raw.json").read_text())
        analysis = json.loads((run_dir / "exp2_analysis.json").read_text())
    except (OSError, ValueError) as exc:
        check("read_saved_evidence", False, str(exc), checks)
        return checks
    run_id = run_dir.name
    check("run_ids_equal", all(v.get("run_id") == run_id for v in (raw, manifest, analysis)), run_id, checks)
    result = manifest.get("result") or {}
    check("run_closed_completed", manifest.get("state") == "CLOSED" and result.get("combined_state") == "COMPLETED",
          {"state": manifest.get("state"), "execution": result.get("combined_state")}, checks)
    check("v3_packet_schema", raw.get("actual_packet_schema") == "port10000-packets/v2",
          raw.get("actual_packet_schema"), checks)
    provenance = analysis.get("input_provenance") or {}
    expected_provenance = {
        "raw_sha256": canonical_hash(raw),
        "manifest_sha256": canonical_hash(manifest),
        "urdf_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
    }
    for key, expected in expected_provenance.items():
        check("provenance_" + key, provenance.get(key) == expected,
              {"recorded": provenance.get(key), "current": expected}, checks)
    check("analyzer_version", provenance.get("analyzer_version") == "exp2-same-frame-phase/v3",
          provenance.get("analyzer_version"), checks)
    policy = raw.get("actual_packet_policy") or {}
    max_span = policy.get("max_interpolation_span_ms", 250.0)
    check("bounded_interpolation_policy", isinstance(max_span, (float, int)) and 0 < max_span <= 250,
          max_span, checks)
    streams = raw.get("actual_packets") or {}
    duration = float((raw.get("execution") or {}).get("duration_s") or 0)
    check("positive_duration", duration >= 5, duration, checks)
    for side in ("left", "right"):
        packets = streams.get(side) or []
        times, mono_times, sequences = [], [], []
        valid = 0
        for packet in packets:
            ts = packet.get("received_at_ms")
            mono_ns = packet.get("received_monotonic_ns")
            seq = packet.get("packet_sequence_index")
            arr = packet.get("actual_joints_rad")
            cmd = packet.get("controller_joint_position_rad")
            if (isinstance(arr, list) and len(arr) == 6 and
                isinstance(cmd, list) and len(cmd) == 6 and
                all(isinstance(v, (int, float)) and math.isfinite(v) for v in arr + cmd)):
                valid += 1
            if isinstance(ts, (float, int)) and math.isfinite(ts):
                times.append(ts)
            if isinstance(mono_ns, int) and not isinstance(mono_ns, bool) and mono_ns >= 0:
                mono_times.append(mono_ns)
            if isinstance(seq, int):
                sequences.append(seq)
        wall_gaps = [b - a for a, b in zip(times, times[1:])]
        mono_gaps_ms = [(b - a) / 1_000_000 for a, b in zip(mono_times, mono_times[1:])]
        check(side + "_packet_quantity", len(packets) >= max(50, duration * 8.0),
              {"packets": len(packets), "duration_s": duration, "min_nominal_hz": 8}, checks)
        check(side + "_joint_field_coverage", valid == len(packets) and len(packets) > 0,
              {"valid": valid, "packets": len(packets)}, checks)
        # Host wall time has millisecond precision: multiple parsed packets may
        # legitimately share one ms. Strict ordering is provided by monotonic_ns.
        check(side + "_timestamps_monotonic",
              len(times) == len(mono_times) == len(packets)
              and all(g >= 0 for g in wall_gaps)
              and all(g > 0 for g in mono_gaps_ms),
              {"wall_timestamps": len(times), "monotonic_timestamps": len(mono_times),
               "same_wall_millisecond_pairs": sum(g == 0 for g in wall_gaps),
               "max_monotonic_gap_ms": max(mono_gaps_ms) if mono_gaps_ms else None}, checks)
        check(side + "_sequence_monotonic", len(sequences) == len(packets) and all(
              b > a for a, b in zip(sequences, sequences[1:])),
              {"sequences": len(sequences)}, checks)
        check(side + "_receive_gap_bounded", bool(mono_gaps_ms) and max(mono_gaps_ms) <= 250,
              {"max_monotonic_gap_ms": max(mono_gaps_ms) if mono_gaps_ms else None,
               "threshold_ms": 250}, checks)
    phase = analysis.get("phase_synchronized") or {}
    coverage = phase.get("coverage") or {}
    fraction = coverage.get("usable_fraction")
    candidate = coverage.get("synchronized_candidate_samples") or 0
    usable = coverage.get("usable_synchronized_samples") or 0
    check("phase_match_status", phase.get("status") in ("AVAILABLE", "PARTIAL"),
          phase.get("status"), checks)
    check("phase_coverage", isinstance(fraction, (int, float)) and fraction >= 0.95 and candidate >= 100,
          {"fraction": fraction, "usable": usable, "candidate": candidate, "minimum_fraction": 0.95}, checks)
    tracking = analysis.get("same_packet_tracking") or {}
    check("same_packet_tracking_available", tracking.get("status") == "AVAILABLE",
          tracking.get("status"), checks)
    for side in ("left", "right"):
        data = ((tracking.get("sides") or {}).get(side) or {})
        check(side + "_tracking_usable", (data.get("overall_rad") or {}).get("sample_count", 0) >= 300,
              {"joint_sample_count": (data.get("overall_rad") or {}).get("sample_count")}, checks)
    return checks


def backend_get(base: str, endpoint: str):
    with urlopen(base.rstrip("/") + endpoint, timeout=3.5) as response:
        return json.load(response)


def passive_live_gate(base: str, seconds: float, interval: float = .10):
    """Observe existing GET snapshots. This is not raw packet-complete logging."""
    checks = []
    stamps = {"left": [], "right": []}
    freshness = {"left": [], "right": []}
    sources = []
    errors = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            obj = backend_get(base, "/api/digital-twin/joints")
            sources.append(obj.get("source"))
            streams = (obj.get("source_diagnostics") or {}).get("port10000") or {}
            for side in ("left", "right"):
                item = streams.get(side) or {}
                ts = item.get("received_at_ms")
                if isinstance(ts, (int, float)) and math.isfinite(ts):
                    stamps[side].append(ts)
                freshness[side].append(bool(item.get("fresh")) and
                    isinstance(item.get("age_ms"), (int, float)) and item["age_ms"] <= 400)
        except Exception as exc:
            errors.append(str(exc))
        time.sleep(interval)
    check("backend_snapshot_errors", len(errors) == 0, {"errors": errors[:5], "count": len(errors)}, checks)
    check("direct_port10000_selected", bool(sources) and all(
          v == "jaka_port10000_actual_feedback" for v in sources),
          {"snapshots": len(sources), "distinct_sources": sorted(set(str(s) for s in sources))}, checks)
    for side in ("left", "right"):
        unique = sorted(set(stamps[side]))
        gaps = [b-a for a,b in zip(unique,unique[1:])]
        ratio = sum(freshness[side]) / max(1,len(freshness[side]))
        check(side + "_live_packet_cadence", len(unique) >= 7 * seconds and bool(gaps) and max(gaps) <= 300,
              {"distinct_receive_timestamps": len(unique), "max_observed_gap_ms": max(gaps) if gaps else None,
               "mean_observed_gap_ms": statistics.mean(gaps) if gaps else None,
               "expected_min_samples_per_second": 7}, checks)
        check(side + "_live_freshness", ratio >= 0.98,
              {"fresh_fraction": round(ratio,4), "minimum": .98}, checks)
    try:
        robots = backend_get(base, "/api/digital-twin/robot-status")
        recorder = backend_get(base, "/api/experiments/recorder")
        for side in ("left", "right"):
            status = robots.get(side) or {}
            check(side + "_ros_feedback_fresh", status.get("feedback_status") == "LIVE",
                  {"status": status.get("feedback_status"),
                   "joint_age_ms": status.get("joint_feedback_age_ms")}, checks)
        check("recorder_idle", recorder.get("state") == "IDLE",
              {"state": recorder.get("state"), "run_id": recorder.get("run_id")}, checks)
    except Exception as exc:
        check("read_backend_recorder", False, str(exc), checks)
    return checks


def offline_regressions():
    env = dict(__import__("os").environ)
    env.update({"PYTHONPATH": str(ROOT), "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1",
                "PYTHONDONTWRITEBYTECODE": "1"})
    args = [sys.executable, "-m", "pytest", "-q", *TESTS, "-p", "no:cacheprovider"]
    result = subprocess.run(args, cwd=str(ROOT), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=120)
    return [dict(name="offline_regression_suite", passed=result.returncode == 0,
                 details={"exit_code": result.returncode, "output": result.stdout[-3500:]})]


def write_report(out: Path, report: dict):
    out.mkdir(parents=True, exist_ok=True)
    (out / "mq_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    lines = ["# JAKA Dual Arm Measurement Qualification", "",
             "Result: **" + report["overall"] + "**",
             "", "Raw experiment evidence was READ ONLY. No robot motion commanded.",
             "", "## Gate checks", "",
             "| Gate | Result |", "|---|---|"]
    for ch in report["checks"]:
        lines.append("| " + ch["name"] + " | " + ("PASS" if ch["passed"] else "FAIL") + " |")
    lines.extend(["", "## Traceable details", "", "```json",
                  json.dumps(report, ensure_ascii=False, indent=2), "```",
                  "", "Interpretation: PRE_RUN_READY qualifies the offline pipeline and passive receiver, "
                  "not physical robot safety, simultaneous controller time, or an upcoming run's captured data. "
                  "Every new recorded run must pass its own saved-run gate before EXP1–EXP4 results are accepted."])
    (out / "mq_report.md").write_text("\n".join(lines) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", default=DEFAULT_RUN, help="existing saved run directory name")
    parser.add_argument("--live-seconds", type=float, default=12.0)
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--skip-offline-tests", action="store_true")
    parser.add_argument("--out", default=str(ROOT / "docs" / "mq_reports" / "latest"))
    opts = parser.parse_args(argv)
    if opts.live_seconds < 0 or opts.live_seconds > 120:
        parser.error("--live-seconds must be 0..120")
    run_dir = (RUNS / opts.run).resolve()
    if run_dir.parent != RUNS.resolve() or not run_dir.is_dir():
        parser.error("--run must name an existing run in experiment_runs")
    checks = saved_run_gate(run_dir)
    if not opts.skip_offline_tests:
        checks += offline_regressions()
    if opts.live_seconds:
        checks += passive_live_gate(opts.base_url, opts.live_seconds)
    passed = all(ch["passed"] for ch in checks)
    report = {"schema": "jaka-dual-arm-mq/v1", "overall": "PRE_RUN_READY" if passed else "BLOCKED",
              "historical_reference_run": opts.run,
              "historical_reference_is_formal_exp_result": False,
              "checks": checks,
              "limitations": [
                 "Host receive timestamps are not controller sample timestamps.",
                 "Phase-matching is NOT simultaneous physical-time synchronization.",
                 "Encoder+FK is NOT external physical metrology.",
                 "Passive API snapshots do NOT prove packet-complete logging.",
                 "An EXP1–EXP4 run must pass its own V3 captured-evidence check after Execute.",
                 "Robot physical safety, workpiece payload and E-stop checks remain operator responsibilities.",
              ]}
    write_report(Path(opts.out), report)
    print("MQ:", report["overall"], "checks:",len(checks),
          "failed:", [c["name"] for c in checks if not c["passed"]])
    print("Report:", Path(opts.out).resolve() / "mq_report.md")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
