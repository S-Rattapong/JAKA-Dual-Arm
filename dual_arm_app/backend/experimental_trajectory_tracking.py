"""Pure EXP-1 planned-versus-actual joint tracking analysis.

The input is persisted EXP-0 evidence.  This module performs no I/O and has no
ROS, robot SDK, motion, timing, fault, or safety authority.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
from copy import deepcopy
import math
from typing import Any


EXP1_SCHEMA_VERSION = "jaka-dual-arm-exp1-trajectory-tracking/v1"
EXP1_SEMANTIC = "ANALYSIS ONLY — NOT PHASE6 TIMING, FAULT, OR SAFETY AUTHORITY"
ERROR_DEFINITION = "actual_joint_rad - interpolated_planned_joint_rad"
ALIGNMENT_REFERENCE = "EXP-0 aligned.json using recorded Actual receive timestamps"
SIDES = ("left", "right")
JOINT_COUNT = 6


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _finite_vector(value: Any) -> list[float] | None:
    if (
        isinstance(value, (str, bytes, Mapping))
        or not isinstance(value, Sequence)
        or len(value) != JOINT_COUNT
    ):
        return None
    checked = [_finite_number(item) for item in value]
    return None if any(item is None for item in checked) else checked


def _location(row: Mapping[str, Any]) -> dict[str, Any]:
    timestamp = _finite_number(row.get("time_from_start_s"))
    observation_index = row.get("observation_index")
    if isinstance(observation_index, bool) or not isinstance(observation_index, int):
        observation_index = None
    return {
        "time_from_start_s": timestamp,
        "observation_index": observation_index,
    }


def _metric(values: list[dict[str, Any]]) -> dict[str, Any]:
    if not values:
        return {
            "sample_count": 0,
            "mean_signed_error_rad": None,
            "mae_rad": None,
            "rmse_rad": None,
            "max_abs_error_rad": None,
            "max_abs_error_location": {
                "time_from_start_s": None,
                "observation_index": None,
            },
        }
    errors = [item["error_rad"] for item in values]
    scale = max(abs(error) for error in errors)
    if scale == 0.0:
        mean_signed = mae = rmse = 0.0
    else:
        mean_signed = scale * math.fsum(error / scale for error in errors) / len(errors)
        mae = scale * math.fsum(abs(error) / scale for error in errors) / len(errors)
        rmse = scale * math.sqrt(
            math.fsum((error / scale) ** 2 for error in errors) / len(errors)
        )
    maximum_item = max(values, key=lambda item: abs(item["error_rad"]))
    return {
        "sample_count": len(errors),
        "mean_signed_error_rad": mean_signed,
        "mae_rad": mae,
        "rmse_rad": rmse,
        "max_abs_error_rad": abs(maximum_item["error_rad"]),
        "max_abs_error_location": deepcopy(maximum_item["location"]),
    }


def _flattened_summary(
    joint_metrics: list[dict[str, Any]],
    values: list[dict[str, Any]],
    *,
    sample_count: int,
) -> dict[str, Any]:
    base = _metric(values)
    base.pop("mean_signed_error_rad", None)
    base.pop("max_abs_error_location", None)
    available = [joint for joint in joint_metrics if joint["sample_count"] > 0]
    worst_max = max(available, key=lambda joint: joint["max_abs_error_rad"], default=None)
    worst_rmse = max(available, key=lambda joint: joint["rmse_rad"], default=None)
    return {
        "sample_count": sample_count,
        "scalar_value_count": len(values),
        "mae_rad": base["mae_rad"],
        "rmse_rad": base["rmse_rad"],
        "max_abs_error_rad": base["max_abs_error_rad"],
        "worst_joint_by_max_error": worst_max["joint"] if worst_max else None,
        "worst_joint_by_rmse": worst_rmse["joint"] if worst_rmse else None,
    }


def _actual_source_semantics(
    raw: Mapping[str, Any], manifest: Mapping[str, Any] | None
) -> dict[str, Any]:
    sources: set[str] = set()
    observations = raw.get("observations", [])
    if isinstance(observations, list):
        for observation in observations:
            if not isinstance(observation, Mapping):
                continue
            actual = observation.get("actual")
            if not isinstance(actual, Mapping):
                continue
            for side in SIDES:
                item = actual.get(side)
                source = item.get("source") if isinstance(item, Mapping) else None
                if isinstance(source, str) and source.strip():
                    sources.add(source.strip())
    description = manifest.get("actual_source_semantics") if isinstance(manifest, Mapping) else None
    if not isinstance(description, str) or not description.strip():
        description = "Recorded encoder/model joint evidence; source metadata unavailable"
    return {
        "description": description.strip(),
        "observed_sources": sorted(sources),
        "external_metrology": False,
    }


def analyze_trajectory_tracking(
    raw: Mapping[str, Any],
    aligned: Mapping[str, Any],
    manifest: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a JSON-safe EXP-1 analysis without mutating its inputs.

    Only valid finite six-joint rows whose persisted EXP-0 status is exactly
    ``INTERPOLATED`` contribute.  Commanded snapshots are intentionally not an
    input to any metric because command and Actual snapshots are asynchronous.
    """

    raw_data = raw if isinstance(raw, Mapping) else {}
    aligned_data = aligned if isinstance(aligned, Mapping) else {}
    samples = aligned_data.get("samples", [])
    if not isinstance(samples, list):
        samples = []

    rows_by_side: dict[str, list[Mapping[str, Any]]] = {side: [] for side in SIDES}
    usable_by_side = {side: 0 for side in SIDES}
    skipped_by_side = {side: Counter() for side in SIDES}
    unassigned_skipped = Counter()
    values_by_side: dict[str, list[list[dict[str, Any]]]] = {
        side: [[] for _index in range(JOINT_COUNT)] for side in SIDES
    }
    error_samples: list[dict[str, Any]] = []

    for sample in samples:
        if not isinstance(sample, Mapping):
            unassigned_skipped["INVALID_ROW"] += 1
            continue
        side = sample.get("side")
        if side not in SIDES:
            unassigned_skipped["UNKNOWN_SIDE"] += 1
            continue
        rows_by_side[side].append(sample)
        status = sample.get("status")
        if status != "INTERPOLATED":
            skipped_by_side[side][str(status or "MISSING_STATUS")] += 1
            continue
        actual = _finite_vector(sample.get("actual_joints_rad"))
        planned = _finite_vector(sample.get("planned_joints_rad"))
        if actual is None or planned is None:
            skipped_by_side[side]["INVALID_OR_NON_FINITE_VECTOR"] += 1
            continue
        location = _location(sample)
        errors = [actual[index] - planned[index] for index in range(JOINT_COUNT)]
        if any(not math.isfinite(error) for error in errors):
            skipped_by_side[side]["NON_FINITE_ERROR"] += 1
            continue
        usable_by_side[side] += 1
        error_samples.append({
            "side": side,
            **location,
            "error_joints_rad": errors,
        })
        for index, error in enumerate(errors):
            values_by_side[side][index].append({
                "error_rad": error,
                "location": location,
            })

    arms: dict[str, Any] = {}
    all_joint_metrics: list[dict[str, Any]] = []
    all_values: list[dict[str, Any]] = []
    for side in SIDES:
        label = side.capitalize()
        joints = []
        flattened = []
        for index in range(JOINT_COUNT):
            metric = _metric(values_by_side[side][index])
            metric.update({
                "joint": f"{label} J{index + 1}",
                "side": side,
                "joint_index": index,
            })
            joints.append(metric)
            flattened.extend(values_by_side[side][index])
        total_side_rows = len(rows_by_side[side])
        usable = usable_by_side[side]
        side_status = (
            "UNAVAILABLE" if usable == 0
            else "AVAILABLE" if usable == total_side_rows
            else "PARTIAL"
        )
        arms[side] = {
            "status": side_status,
            "summary": _flattened_summary(joints, flattened, sample_count=usable),
            "joints": joints,
        }
        all_joint_metrics.extend(joints)
        all_values.extend(flattened)

    sides_with_data = sum(usable_by_side[side] > 0 for side in SIDES)
    overall_status = (
        "AVAILABLE" if sides_with_data == 2
        else "PARTIAL" if sides_with_data == 1
        else "UNAVAILABLE"
    )
    combined_summary = _flattened_summary(
        all_joint_metrics,
        all_values,
        sample_count=sum(usable_by_side.values()),
    )
    combined_summary["status"] = overall_status

    coverage_sides = {}
    overall_skipped = Counter(unassigned_skipped)
    for side in SIDES:
        total = len(rows_by_side[side])
        usable = usable_by_side[side]
        skipped = dict(sorted(skipped_by_side[side].items()))
        overall_skipped.update(skipped_by_side[side])
        coverage_sides[side] = {
            "total_aligned_rows": total,
            "usable_rows": usable,
            "skipped_rows": total - usable,
            "usable_fraction": usable / total if total else None,
            "skipped_status_counts": skipped,
        }

    run_id = aligned_data.get("run_id") or raw_data.get("run_id")
    return {
        "schema_version": EXP1_SCHEMA_VERSION,
        "experiment": "EXP-1 Trajectory Tracking Accuracy",
        "run_id": run_id,
        "status": overall_status,
        "semantic": EXP1_SEMANTIC,
        "error_definition": ERROR_DEFINITION,
        "alignment_reference": ALIGNMENT_REFERENCE,
        "commanded_snapshot_role": "DISPLAY ONLY — excluded from primary EXP-1 metrics",
        "actual_source_semantics": _actual_source_semantics(raw_data, manifest),
        "coverage": {
            "total_aligned_rows": len(samples),
            "usable_rows": sum(usable_by_side.values()),
            "skipped_rows": len(samples) - sum(usable_by_side.values()),
            "skipped_status_counts": dict(sorted(overall_skipped.items())),
            "sides": coverage_sides,
        },
        "combined": combined_summary,
        "arms": arms,
        "error_samples": error_samples,
    }
