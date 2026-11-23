"""EXP-3 time-base comparison: host receive time vs controller-inferred sample time.

Analysis only. Reads persisted raw evidence, never writes files, never contacts a
robot. The existing host-time and phase-matched EXP-2/EXP-3 results are left
untouched; this module adds a third, clearly labelled view.

Why: Port10000 packets carry no controller timestamp. Host receive times jitter
by tens of ms relative to when the controller sampled the data, which appears as
relative-pose "error" at speed (error ~ speed x timing mismatch). Each packet,
however, also contains the controller-reported joint_position, which follows the
frozen plan in time. Locating that position on the plan gives an inferred sample
time per packet (tau). The same projection applied to joint_actual_position gives
the per-packet tracking lag (tau_ctrl - tau_actual), which is normally ~35 ms;
packets whose lag deviates strongly are flagged as internally inconsistent.

Limitations: tau assumes the controller follows the plan with a constant delay;
it is only informative while the plan is moving; flagged packets may be either a
reporting artifact or a real brief servo deviation (not decidable from this data).
Encoder + URDF FK model based, NOT external metrology.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

try:
    from .experimental_rigid_grasp import _analyze_host_time
except ImportError:  # script/test execution from the backend directory
    from experimental_rigid_grasp import _analyze_host_time

TIMEBASE_VERSION = "exp3-controller-timebase/v1"
SEMANTIC = (
    "ANALYSIS ONLY - CONTROLLER-INFERRED SAMPLE TIME FROM SAME-PACKET joint_position "
    "PHASE ON THE FROZEN PLAN; ENCODER+URDF MODEL BASED; NOT EXTERNAL METROLOGY; "
    "NOT PROOF OF PHYSICAL SIMULTANEITY"
)
SIDES = ("left", "right")
GRID_S = 0.0005
SEARCH_BACK_S = 0.3
SEARCH_FORWARD_S = 1.0


def _plan_arrays(raw: Mapping[str, Any]):
    samples = (raw.get("planned") or {}).get("samples") or []
    if len(samples) < 3:
        raise ValueError("planned samples unavailable")
    times = np.array([float(s["time_from_start_s"]) for s in samples])
    joints = {side: np.array([s[side]["joints_rad"] for s in samples], dtype=float) for side in SIDES}
    grid = np.arange(0.0, times[-1] + 1e-9, GRID_S)
    dense = {side: np.stack([np.interp(grid, times, joints[side][:, j]) for j in range(6)], axis=1) for side in SIDES}
    speed = {side: np.r_[0.0, np.linalg.norm(np.diff(dense[side], axis=0), axis=1) / GRID_S] for side in SIDES}
    return times, grid, dense, speed


def _phase(grid, dense, joints, previous):
    lo = max(0, int((previous - SEARCH_BACK_S) / GRID_S))
    hi = min(len(grid), int((previous + SEARCH_FORWARD_S) / GRID_S) + 1)
    distance = np.linalg.norm(dense[lo:hi] - np.asarray(joints, dtype=float), axis=1)
    k = int(distance.argmin())
    return float(grid[lo + k])


def _stats(values) -> dict[str, float | None]:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"count": 0, "median": None, "std": None, "p5": None, "p95": None, "max_abs_dev": None}
    median = float(np.median(values))
    return {"count": int(values.size), "median": median, "std": float(values.std()),
            "p5": float(np.percentile(values, 5)), "p95": float(np.percentile(values, 95)),
            "max_abs_dev": float(np.abs(values - median).max())}


def _summary(result: Mapping[str, Any]) -> dict[str, Any]:
    translation = (result.get("relative_translation") or {}).get("magnitude_m") or {}
    orientation = (result.get("relative_orientation") or {}).get("angle") or {}
    coverage = result.get("coverage") or {}
    samples = []
    for item in result.get("samples") or []:
        t = item.get("time_from_start_s")
        e = item.get("relative_translation_error_magnitude_m")
        a = item.get("relative_orientation_error_deg")
        if all(isinstance(v, (int, float)) and math.isfinite(v) for v in (t, e, a)):
            samples.append([round(float(t), 4), round(float(e) * 1000.0, 4), round(float(a), 5)])
    location = translation.get("max_location") or {}
    return {
        "status": result.get("status"),
        "usable_fraction": coverage.get("usable_fraction"),
        "usable_samples": coverage.get("usable_synchronized_samples"),
        "translation_rms_mm": None if translation.get("rms") is None else translation["rms"] * 1000.0,
        "translation_max_mm": None if translation.get("max") is None else translation["max"] * 1000.0,
        "translation_max_time_s": location.get("time_from_start_s"),
        "orientation_rms_deg": orientation.get("rms_deg"),
        "orientation_max_deg": orientation.get("max_deg"),
        "samples_t_mm_deg": samples,
    }


def analyze_controller_timebase(
    raw: Mapping[str, Any],
    manifest: Mapping[str, Any],
    urdf_path: str | Path,
    *,
    lag_outlier_threshold_ms: float = 15.0,
    moving_joint_speed_rad_s: float = 0.05,
    flag_min_joint_speed_rad_s: float = 0.12,
) -> dict[str, Any]:
    """Compare EXP-3 relative pose under host-time and controller-inferred time bases."""
    base = {"schema_version": TIMEBASE_VERSION, "run_id": raw.get("run_id") or manifest.get("run_id"),
            "semantic": SEMANTIC,
            "policy": {"lag_outlier_threshold_ms": lag_outlier_threshold_ms,
                       "moving_joint_speed_rad_s": moving_joint_speed_rad_s,
                       "flag_min_joint_speed_rad_s": flag_min_joint_speed_rad_s,
                       "phase_grid_s": GRID_S,
                       "flag_rule": ("packet with planned joint speed >= flag_min_joint_speed_rad_s and "
                                     "|tracking_lag - side median| > lag_outlier_threshold_ms; below that speed "
                                     "the phase (and lag) estimate is too poorly conditioned to judge")}}
    execution = raw.get("execution") or {}
    packets = raw.get("actual_packets") or {}
    start_ns = execution.get("start_time_unix_ns")
    if not isinstance(start_ns, int) or not all(packets.get(s) for s in SIDES):
        return {**base, "status": "UNAVAILABLE", "reason": "execution start or actual packets missing"}
    try:
        times, grid, dense, speed = _plan_arrays(raw)
    except (KeyError, TypeError, ValueError) as error:
        return {**base, "status": "UNAVAILABLE", "reason": f"plan unavailable: {error}"}
    start_ms = start_ns / 1e6
    end_s = float(times[-1])
    per_side: dict[str, Any] = {}
    inferred: dict[str, Any] = {}
    offsets_all = []
    for side in SIDES:
        tau_c, tau_a, host_t, moving, fast = [], [], [], [], []
        pc = pa = 0.0
        for packet in packets[side]:
            controller = packet.get("controller_joint_position_rad")
            actual = packet.get("actual_joints_rad") or packet.get("joints_rad")
            host = (float(packet["received_at_ms"]) - start_ms) / 1000.0
            if controller is None or actual is None:
                tau_c.append(None); tau_a.append(None); host_t.append(host); moving.append(False); fast.append(False)
                continue
            pc = _phase(grid, dense[side], controller, pc)
            pa = _phase(grid, dense[side], actual, pa)
            k = min(int(pc / GRID_S), len(grid) - 1)
            tau_c.append(pc); tau_a.append(pa); host_t.append(host)
            moving.append(bool(speed[side][k] > moving_joint_speed_rad_s and 0.0 < pc < end_s))
            fast.append(bool(speed[side][k] >= flag_min_joint_speed_rad_s and 0.0 < pc < end_s))
        lag = np.array([(c - a) * 1000.0 if m else np.nan for c, a, m in zip(tau_c, tau_a, moving)])
        lag_median = float(np.nanmedian(lag)) if np.isfinite(lag).any() else None
        flagged = []
        for index, (value, m) in enumerate(zip(lag, fast)):
            if m and lag_median is not None and abs(value - lag_median) > lag_outlier_threshold_ms:
                packet = packets[side][index]
                previous = next((tau_c[j] for j in range(index - 1, -1, -1) if tau_c[j] is not None), None)
                flagged.append({"packet_index": index,
                                "packet_sequence_index": packet.get("packet_sequence_index"),
                                "host_time_s": host_t[index], "phase_time_s": tau_c[index],
                                "tracking_lag_ms": float(value), "deviation_ms": float(value - lag_median),
                                # Survey of 26 runs (23 Nov): deviations cluster at whole 8 ms controller
                                # cycles, i.e. the actual field looks k cycles older than joint_position.
                                "estimated_stale_cycles_8ms": int(round((value - lag_median) / 8.0)),
                                "controller_spacing_before_ms": None if previous is None
                                else float((tau_c[index] - previous) * 1000.0)})
        offsets = [h - c for h, c, m in zip(host_t, tau_c, moving) if m]
        offsets_all.extend(offsets)
        per_side[side] = {"packets": len(packets[side]), "moving_packets": int(sum(moving)),
                          "tracking_lag_ms": _stats(lag[np.isfinite(lag)]),
                          "host_minus_controller_time_ms": _stats(np.array(offsets) * 1000.0),
                          "flagged_packets": flagged,
                          "lag_series": [[round(t, 4), round(float(v), 2)] for t, v, m in zip(tau_c, lag, moving) if m]}
        inferred[side] = (tau_c, host_t, moving, {f["packet_index"] for f in flagged})
    if not offsets_all:
        return {**base, "status": "UNAVAILABLE", "reason": "no moving packets", "sides": per_side}
    latency_s = float(np.median(offsets_all))

    def retimed(drop_flagged: bool):
        modified = copy.deepcopy(dict(raw))
        for side in SIDES:
            tau_c, host_t, moving, flagged = inferred[side]
            new_t = [c + latency_s if m else h for c, h, m in zip(tau_c, host_t, moving)]
            for i in range(1, len(new_t)):
                if new_t[i] <= new_t[i - 1]:
                    new_t[i] = new_t[i - 1] + 1e-4
            original = modified["actual_packets"][side]
            mono0 = original[0]["received_monotonic_ns"] - int(host_t[0] * 1e9)
            kept = []
            for index, (packet, t) in enumerate(zip(original, new_t)):
                packet["received_at_ms"] = start_ms + t * 1000.0
                packet["received_monotonic_ns"] = int(mono0 + t * 1e9)
                if not (drop_flagged and index in flagged):
                    kept.append(packet)
            modified["actual_packets"][side] = kept
        return modified

    host = _analyze_host_time(raw, manifest, urdf_path)
    controller = _analyze_host_time(retimed(False), manifest, urdf_path)
    controller_clean = _analyze_host_time(retimed(True), manifest, urdf_path)
    return {
        **base,
        "status": "COMPLETE",
        "common_latency_ms": latency_s * 1000.0,
        "sides": per_side,
        "flagged_packet_count": sum(len(per_side[s]["flagged_packets"]) for s in SIDES),
        "total_packet_count": sum(per_side[s]["packets"] for s in SIDES),
        "methods": {
            "host_time": _summary(host),
            "controller_time": _summary(controller),
            "controller_time_excluding_flagged": _summary(controller_clean),
        },
        "limitations": [
            "Controller time is inferred from joint_position assuming the controller follows the frozen plan with constant delay.",
            "Only moving packets are retimed/flagged; stationary packets keep host receive time.",
            "Flagged packets may be a reporting artifact or a real brief servo deviation; report results with and without exclusion.",
            "Encoder + URDF FK model based; not external metrology.",
        ],
    }
