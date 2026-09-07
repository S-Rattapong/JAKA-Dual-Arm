"""EXP-0 observational experiment recording and offline analysis.

This module has no ROS, robot SDK, motion client, or execution authority.  Raw
samples are authoritative; alignment is explicitly analysis-only.
"""

from __future__ import annotations

import csv
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import threading
import time
import uuid
from collections.abc import Mapping, Sequence
from typing import Any, Callable

try:
    from dual_arm_app.backend.experimental_trajectory_tracking import (
        EXP1_SCHEMA_VERSION,
        analyze_trajectory_tracking,
    )
except ImportError:
    try:
        from .experimental_trajectory_tracking import (
            EXP1_SCHEMA_VERSION,
            analyze_trajectory_tracking,
        )
    except ImportError:
        from experimental_trajectory_tracking import (
            EXP1_SCHEMA_VERSION,
            analyze_trajectory_tracking,
        )

try:
    from dual_arm_app.backend.experimental_rigid_grasp import (
        EXP2_SCHEMA_VERSION,
        EXP2_SEMANTIC,
        analyze_rigid_grasp,
    )
except ImportError:
    try:
        from .experimental_rigid_grasp import EXP2_SCHEMA_VERSION, EXP2_SEMANTIC, analyze_rigid_grasp
    except ImportError:
        from experimental_rigid_grasp import EXP2_SCHEMA_VERSION, EXP2_SEMANTIC, analyze_rigid_grasp


SCHEMA_VERSION = "jaka-dual-arm-experiment-run/v1"
EXP2_URDF_PATH = Path(__file__).resolve().parents[1] / "web" / "assets" / "dual_jaka_a12_web.urdf"
ALIGNMENT_SEMANTIC = "ANALYSIS ONLY — NOT PHASE6 TIMING OR FAULT AUTHORITY"
RUN_ID_PATTERN = re.compile(r"^exp-[0-9]{8}T[0-9]{9}Z-[0-9a-f]{8}$")
PATH_TYPES = frozenset({"LINEAR", "CURVED", "COMPLEX", "CUSTOM"})
FIXTURE_CONDITIONS = frozenset({"NONE", "CLEARANCE", "RIGID", "OTHER"})
TERMINAL_STATES = frozenset({"COMPLETED", "ABORTED", "FAILED"})
JOINT_ORDER = tuple(
    [f"left_joint_{index}" for index in range(1, 7)]
    + [f"right_joint_{index}" for index in range(1, 7)]
)


class ExperimentError(ValueError):
    pass


def _json_copy(value: Any) -> Any:
    try:
        return json.loads(json.dumps(value, allow_nan=False))
    except (TypeError, ValueError) as error:
        raise ExperimentError(f"value is not finite JSON data: {error}") from error


def _finite(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExperimentError(f"{path} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ExperimentError(f"{path} must be finite")
    return number


def _vector(value: Any, count: int, path: str) -> list[float]:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, Sequence):
        raise ExperimentError(f"{path} must contain exactly {count} values")
    if len(value) != count:
        raise ExperimentError(f"{path} must contain exactly {count} values")
    return [_finite(item, f"{path}[{index}]") for index, item in enumerate(value)]


def _text(value: Any, path: str, *, maximum: int, required: bool = False) -> str:
    if value is None and not required:
        return ""
    if not isinstance(value, str):
        raise ExperimentError(f"{path} must be a string")
    result = value.strip()
    if required and not result:
        raise ExperimentError(f"{path} must not be empty")
    if len(result) > maximum:
        raise ExperimentError(f"{path} exceeds {maximum} characters")
    return result


def _utc_iso(now_ns: int | None = None) -> str:
    ns = time.time_ns() if now_ns is None else int(now_ns)
    return datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def configuration_fingerprint(configuration: Any) -> str:
    encoded = json.dumps(
        configuration, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


class TorqueCache:
    """Thread-safe cache for existing read-only joint_torque_raw topics."""

    def __init__(self, *, stale_after_ms: int = 500):
        if isinstance(stale_after_ms, bool) or int(stale_after_ms) < 0:
            raise ValueError("stale_after_ms must be non-negative")
        self.stale_after_ms = int(stale_after_ms)
        self._lock = threading.Lock()
        self._cache = {
            side: {
                "values": None,
                "received_at_ms": None,
                "valid": False,
                "status": "MISSING",
                "error": "No joint torque sample received",
                "source": f"/{side}_jaka_driver/joint_torque_raw",
            }
            for side in ("left", "right")
        }

    def update(self, side: str, values: Any, *, received_at_ms: int | None = None) -> None:
        if side not in self._cache:
            raise ValueError("side must be left or right")
        received = time.time_ns() // 1_000_000 if received_at_ms is None else int(received_at_ms)
        try:
            checked = _vector(list(values), 6, f"{side}.torque")
            replacement = {
                "values": checked,
                "received_at_ms": received,
                "valid": True,
                "status": "VALID",
                "error": None,
                "source": f"/{side}_jaka_driver/joint_torque_raw",
            }
        except (ExperimentError, TypeError, ValueError) as error:
            replacement = {
                "values": None,
                "received_at_ms": received,
                "valid": False,
                "status": "INVALID",
                "error": str(error),
                "source": f"/{side}_jaka_driver/joint_torque_raw",
            }
        with self._lock:
            self._cache[side] = replacement

    def snapshot(self, *, now_ms: int | None = None) -> dict[str, Any]:
        current = time.time_ns() // 1_000_000 if now_ms is None else int(now_ms)
        with self._lock:
            result = deepcopy(self._cache)
        for side in ("left", "right"):
            item = result[side]
            received = item.get("received_at_ms")
            age = max(0, current - received) if isinstance(received, int) else None
            item["age_ms"] = age
            item["fresh"] = item["valid"] is True and age is not None and age <= self.stale_after_ms
            if item["valid"] is True and not item["fresh"]:
                item["status"] = "STALE"
                item["error"] = f"Torque sample age {age} ms exceeds {self.stale_after_ms} ms"
        return {
            "unit": "newton_meter",
            "server_time_ms": current,
            "stale_after_ms": self.stale_after_ms,
            **result,
        }


class ExperimentRunStore:
    """Contained JSON/CSV persistence with immutable server-generated run IDs."""

    def __init__(self, root: str | Path):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def generate_run_id(self, *, now: datetime | None = None) -> str:
        current = now or datetime.now(timezone.utc)
        stamp = current.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S%f")[:18] + "Z"
        return f"exp-{stamp}-{uuid.uuid4().hex[:8]}"

    def _run_dir(self, run_id: str, *, require_exists: bool = False) -> Path:
        if not isinstance(run_id, str) or RUN_ID_PATTERN.fullmatch(run_id) is None:
            raise ExperimentError("Invalid server-generated run_id")
        candidate = (self.root / run_id).resolve()
        if candidate.parent != self.root:
            raise ExperimentError("run path escapes experiment_runs")
        if require_exists and not candidate.is_dir():
            raise ExperimentError("Experiment run not found")
        return candidate

    @staticmethod
    def _write_json(path: Path, payload: Any) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)

    @staticmethod
    def _planned_rows(raw: Mapping[str, Any]):
        for sample in raw.get("planned", {}).get("samples", []):
            yield [
                sample.get("sample_index"), sample.get("time_from_start_s"),
                *sample.get("left", {}).get("joints_rad", []),
                *sample.get("right", {}).get("joints_rad", []),
                json.dumps(sample.get("cartesian"), separators=(",", ":")),
            ]

    @staticmethod
    def _observed_rows(raw: Mapping[str, Any]):
        for sample in raw.get("observations", []):
            yield [
                sample.get("observation_index"), sample.get("capture_wall_time_ms"),
                sample.get("execution_state"), sample.get("timeline_s"),
                json.dumps(sample.get("commanded"), separators=(",", ":")),
                json.dumps(sample.get("actual"), separators=(",", ":")),
                json.dumps(sample.get("torque"), separators=(",", ":")),
            ]

    def _write_csv(self, run_dir: Path, raw: Mapping[str, Any]) -> None:
        with (run_dir / "planned.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["sample_index", "time_from_start_s", *JOINT_ORDER, "cartesian_json"])
            writer.writerows(self._planned_rows(raw))
        with (run_dir / "observed.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["observation_index", "capture_wall_time_ms", "execution_state", "timeline_s", "commanded_json", "actual_json", "torque_json"])
            writer.writerows(self._observed_rows(raw))

    def create(self, manifest: Mapping[str, Any], raw: Mapping[str, Any]) -> str:
        run_id = str(manifest.get("run_id", ""))
        run_dir = self._run_dir(run_id)
        with self._lock:
            if run_dir.exists():
                raise ExperimentError("Experiment run already exists")
            run_dir.mkdir(parents=False)
            self._write_json(run_dir / "manifest.json", manifest)
            self._write_json(run_dir / "raw.json", raw)
            self._write_json(run_dir / "aligned.json", {
                "schema_version": SCHEMA_VERSION,
                "run_id": run_id,
                "semantic": ALIGNMENT_SEMANTIC,
                "status": "NOT_ALIGNED",
                "samples": [],
            })
            self._write_csv(run_dir, raw)
        return run_id

    def update(
        self,
        run_id: str,
        manifest: Mapping[str, Any],
        raw: Mapping[str, Any],
        aligned: Mapping[str, Any] | None = None,
        exp1_analysis: Mapping[str, Any] | None = None,
        exp2_analysis: Mapping[str, Any] | None = None,
    ) -> None:
        run_dir = self._run_dir(run_id, require_exists=True)
        with self._lock:
            self._write_json(run_dir / "manifest.json", manifest)
            self._write_json(run_dir / "raw.json", raw)
            if aligned is not None:
                self._write_json(run_dir / "aligned.json", aligned)
            if exp1_analysis is not None:
                self._write_json(run_dir / "exp1_analysis.json", exp1_analysis)
            if exp2_analysis is not None:
                self._write_json(run_dir / "exp2_analysis.json", exp2_analysis)
            self._write_csv(run_dir, raw)

    def load(self, run_id: str) -> dict[str, Any]:
        run_dir = self._run_dir(run_id, require_exists=True)
        try:
            manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
            raw = json.loads((run_dir / "raw.json").read_text(encoding="utf-8"))
            aligned = json.loads((run_dir / "aligned.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ExperimentError(f"Corrupt experiment run: {error}") from error
        if not all(isinstance(value, Mapping) for value in (manifest, raw, aligned)):
            raise ExperimentError("Corrupt experiment run: JSON roots must be objects")
        if (
            manifest.get("run_id") != run_id
            or raw.get("run_id") != run_id
            or aligned.get("run_id") != run_id
        ):
            raise ExperimentError("Corrupt experiment run identity")
        if aligned.get("semantic") != ALIGNMENT_SEMANTIC:
            raise ExperimentError("Corrupt experiment alignment semantic")
        result = {"manifest": manifest, "raw": raw, "aligned": aligned}
        analysis_path = run_dir / "exp1_analysis.json"
        if analysis_path.exists():
            try:
                exp1_analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                raise ExperimentError(f"Corrupt EXP-1 analysis: {error}") from error
            if not isinstance(exp1_analysis, Mapping):
                raise ExperimentError("Corrupt EXP-1 analysis: JSON root must be an object")
            if (
                exp1_analysis.get("run_id") != run_id
                or exp1_analysis.get("schema_version") != EXP1_SCHEMA_VERSION
            ):
                raise ExperimentError("Corrupt EXP-1 analysis identity or schema")
            result["exp1_analysis"] = exp1_analysis
        exp2_path = run_dir / "exp2_analysis.json"
        if exp2_path.exists():
            try:
                exp2_analysis = json.loads(exp2_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                raise ExperimentError(f"Corrupt EXP-2 analysis: {error}") from error
            if not isinstance(exp2_analysis, Mapping):
                raise ExperimentError("Corrupt EXP-2 analysis: JSON root must be an object")
            if (
                exp2_analysis.get("run_id") != run_id
                or exp2_analysis.get("schema_version") != EXP2_SCHEMA_VERSION
            ):
                raise ExperimentError("Corrupt EXP-2 analysis identity or schema")
            result["exp2_analysis"] = exp2_analysis
        return result

    def write_exp1_analysis(self, run_id: str, analysis: Mapping[str, Any]) -> None:
        run_dir = self._run_dir(run_id, require_exists=True)
        if not isinstance(analysis, Mapping):
            raise ExperimentError("EXP-1 analysis must be an object")
        if (
            analysis.get("run_id") != run_id
            or analysis.get("schema_version") != EXP1_SCHEMA_VERSION
        ):
            raise ExperimentError("EXP-1 analysis identity or schema mismatch")
        checked = _json_copy(analysis)
        with self._lock:
            self._write_json(run_dir / "exp1_analysis.json", checked)

    def analyze_exp1(self, run_id: str) -> dict[str, Any]:
        data = self.load(run_id)
        analysis = analyze_trajectory_tracking(
            data["raw"], data["aligned"], data["manifest"]
        )
        self.write_exp1_analysis(run_id, analysis)
        return analysis

    def write_exp2_analysis(self, run_id: str, analysis: Mapping[str, Any]) -> None:
        run_dir = self._run_dir(run_id, require_exists=True)
        if not isinstance(analysis, Mapping):
            raise ExperimentError("EXP-2 analysis must be an object")
        if (
            analysis.get("run_id") != run_id
            or analysis.get("schema_version") != EXP2_SCHEMA_VERSION
        ):
            raise ExperimentError("EXP-2 analysis identity or schema mismatch")
        checked = _json_copy(analysis)
        with self._lock:
            self._write_json(run_dir / "exp2_analysis.json", checked)

    def analyze_exp2(self, run_id: str) -> dict[str, Any]:
        data = self.load(run_id)
        analysis = analyze_rigid_grasp(
            data["raw"], data["manifest"], EXP2_URDF_PATH
        )
        self.write_exp2_analysis(run_id, analysis)
        return analysis

    def list_runs(self) -> list[dict[str, Any]]:
        rows = []
        for path in sorted(self.root.iterdir(), reverse=True):
            if not path.is_dir() or RUN_ID_PATTERN.fullmatch(path.name) is None:
                continue
            try:
                data = self.load(path.name)
                manifest = data["manifest"]
                rows.append({
                    "run_id": path.name,
                    "label": manifest.get("label", ""),
                    "state": manifest.get("state", "UNKNOWN"),
                    "created_at": manifest.get("created_at"),
                    "trajectory_name": manifest.get("trajectory_name"),
                    "sample_counts": manifest.get("result", {}).get("sample_counts", {}),
                    "warning": None,
                })
            except ExperimentError as error:
                rows.append({"run_id": path.name, "label": path.name, "state": "CORRUPT", "warning": str(error)})
        return rows

    def relabel(self, run_id: str, label: Any) -> dict[str, Any]:
        checked = _text(label, "label", maximum=160)
        data = self.load(run_id)
        data["manifest"]["label"] = checked
        data["manifest"]["updated_at"] = _utc_iso()
        self.update(run_id, data["manifest"], data["raw"], data["aligned"])
        return data["manifest"]

    def delete(self, run_id: str) -> None:
        run_dir = self._run_dir(run_id, require_exists=True)
        allowed = {
            "manifest.json", "raw.json", "planned.csv", "observed.csv",
            "aligned.json", "exp1_analysis.json", "exp2_analysis.json",
        }
        files = list(run_dir.iterdir())
        if any(not item.is_file() or item.name not in allowed for item in files):
            raise ExperimentError("Run contains unexpected entries; refusing delete")
        with self._lock:
            for item in files:
                item.unlink()
            run_dir.rmdir()


def validate_plan_snapshot(plan: Any, artifact: Any) -> dict[str, Any] | None:
    if plan is None:
        return None
    checked = _json_copy(plan)
    if not isinstance(checked, dict):
        raise ExperimentError("plan_snapshot must be an object")
    times = checked.get("combined_timestamps_s")
    points = checked.get("combined_path")
    expected_times = list(artifact.common_timestamps_s)
    if not isinstance(times, list) or not isinstance(points, list) or len(times) != len(expected_times) or len(points) != len(expected_times):
        raise ExperimentError("plan_snapshot sample count does not match frozen artifact")
    for index, (timestamp, point) in enumerate(zip(times, points)):
        if not isinstance(point, dict):
            raise ExperimentError(f"plan_snapshot.combined_path[{index}] must be an object")
        time_value = _finite(timestamp, f"combined_timestamps_s[{index}]")
        point_time = _finite(point.get("time_from_start_s"), f"combined_path[{index}].time_from_start_s")
        left = _vector(point.get("left"), 6, f"combined_path[{index}].left")
        right = _vector(point.get("right"), 6, f"combined_path[{index}].right")
        combined = _vector(point.get("combined"), 12, f"combined_path[{index}].combined")
        sample_index = point.get("sample_index", index)
        if isinstance(sample_index, bool) or sample_index != index:
            raise ExperimentError(
                f"plan_snapshot sample_index mismatch at sample {index}"
            )
        if not math.isclose(time_value, expected_times[index], abs_tol=1e-12, rel_tol=0.0) or not math.isclose(point_time, expected_times[index], abs_tol=1e-12, rel_tol=0.0):
            raise ExperimentError(f"plan_snapshot timestamp mismatch at sample {index}")
        expected = list(artifact.combined_positions_rad[index])
        if any(abs(a - b) > 1e-12 for a, b in zip(combined, left + right)):
            raise ExperimentError(f"plan_snapshot left/right combination mismatch at sample {index}")
        if any(abs(a - b) > 1e-12 for a, b in zip(combined, expected)):
            raise ExperimentError(f"plan_snapshot joint mismatch at sample {index}")
    trajectory_name = checked.get("trajectory_name")
    if trajectory_name is not None and trajectory_name != artifact.trajectory_name:
        raise ExperimentError("plan_snapshot trajectory_name does not match frozen artifact")
    duration = checked.get("combined_duration_s")
    if duration is not None and not math.isclose(
        _finite(duration, "combined_duration_s"),
        float(artifact.duration_s),
        abs_tol=1e-12,
        rel_tol=0.0,
    ):
        raise ExperimentError("plan_snapshot duration does not match frozen artifact")
    objects = checked.get("combined_object_samples")
    if objects is not None:
        if not isinstance(objects, list) or len(objects) != len(expected_times):
            raise ExperimentError("combined_object_samples must match frozen samples")
        for index, sample in enumerate(objects):
            if not isinstance(sample, dict):
                raise ExperimentError(
                    f"plan_snapshot.combined_object_samples[{index}] must be an object"
                )
            if not math.isclose(
                _finite(sample.get("time_from_start_s"), f"combined_object_samples[{index}].time_from_start_s"),
                expected_times[index], abs_tol=1e-12, rel_tol=0.0,
            ):
                raise ExperimentError(f"plan_snapshot object timestamp mismatch at sample {index}")
            sample_index = sample.get("sample_index", index)
            if isinstance(sample_index, bool) or sample_index != index:
                raise ExperimentError(
                    f"plan_snapshot object sample_index mismatch at sample {index}"
                )
            for pose_name in ("object_pose", "left_target", "right_target"):
                pose = sample.get(pose_name)
                if pose is None:
                    continue
                if not isinstance(pose, Mapping):
                    raise ExperimentError(
                        f"combined_object_samples[{index}].{pose_name} must be an object"
                    )
                if "translation_m" in pose:
                    _vector(
                        pose["translation_m"],
                        3,
                        f"combined_object_samples[{index}].{pose_name}.translation_m",
                    )
                if "rpy_rad" in pose:
                    _vector(
                        pose["rpy_rad"],
                        3,
                        f"combined_object_samples[{index}].{pose_name}.rpy_rad",
                    )
                if "matrix" in pose:
                    matrix = pose["matrix"]
                    if (
                        isinstance(matrix, (str, bytes, Mapping))
                        or not isinstance(matrix, Sequence)
                        or len(matrix) != 4
                    ):
                        raise ExperimentError(
                            f"combined_object_samples[{index}].{pose_name}.matrix must be 4x4"
                        )
                    for row_index, row in enumerate(matrix):
                        _vector(
                            row,
                            4,
                            f"combined_object_samples[{index}].{pose_name}.matrix[{row_index}]",
                        )
    return checked


def align_planned_to_actual(raw: Mapping[str, Any]) -> dict[str, Any]:
    source = _json_copy(raw)
    planned = source.get("planned", {}).get("samples", [])
    times = [sample.get("time_from_start_s") for sample in planned]
    output = {"schema_version": SCHEMA_VERSION, "run_id": source.get("run_id"), "semantic": ALIGNMENT_SEMANTIC, "status": "ALIGNED", "samples": []}
    if (
        len(planned) < 2
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in times
        )
        or any(float(right) <= float(left) for left, right in zip(times, times[1:]))
    ):
        output.update({"status": "UNAVAILABLE", "error": "Planned series needs at least two finite samples"})
        return output
    start_ns = source.get("execution", {}).get("start_time_unix_ns")
    for observation in source.get("observations", []):
        for side in ("left", "right"):
            actual = observation.get("actual", {}).get(side, {})
            received = actual.get("received_at_ms")
            row = {"observation_index": observation.get("observation_index"), "side": side, "actual_received_at_ms": received, "actual_joints_rad": deepcopy(actual.get("joints_rad")), "planned_joints_rad": None, "time_from_start_s": None, "status": "MISSING_ACTUAL"}
            if (
                actual.get("valid") is not True
                or isinstance(received, bool)
                or not isinstance(received, (int, float))
                or not math.isfinite(float(received))
            ):
                output["samples"].append(row)
                continue
            if isinstance(start_ns, bool) or not isinstance(start_ns, int) or start_ns <= 0:
                row["status"] = "MISSING_EXECUTION_START"
                output["samples"].append(row)
                continue
            target = (float(received) - start_ns / 1e6) / 1000.0
            row["time_from_start_s"] = target
            if target < times[0] or target > times[-1]:
                row["status"] = "OUT_OF_RANGE"
                output["samples"].append(row)
                continue
            upper = next((index for index, value in enumerate(times) if value >= target), len(times) - 1)
            lower = max(0, upper - 1)
            if upper == lower or times[upper] == times[lower]:
                ratio = 0.0
            else:
                ratio = (target - times[lower]) / (times[upper] - times[lower])
            a = planned[lower][side]["joints_rad"]
            b = planned[upper][side]["joints_rad"]
            row["planned_joints_rad"] = [float(x) + ratio * (float(y) - float(x)) for x, y in zip(a, b)]
            row["status"] = "INTERPOLATED"
            output["samples"].append(row)
    return output


class ExperimentRecorder:
    """Single armed/active observational recorder bound after Phase5 acceptance."""

    def __init__(self, store: ExperimentRunStore, *, actual_stale_after_ms: int = 500, wall_time_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000):
        self.store = store
        if isinstance(actual_stale_after_ms, bool) or int(actual_stale_after_ms) < 0:
            raise ValueError("actual_stale_after_ms must be non-negative")
        self.actual_stale_after_ms = int(actual_stale_after_ms)
        self._wall_time_ms = wall_time_ms
        self._lock = threading.Lock()
        self._manifest: dict[str, Any] | None = None
        self._raw: dict[str, Any] | None = None
        self._last_signature: str | None = None
        self._last_persist_ms: int | None = None
        self._error: str | None = None

    def arm(self, *, artifact: Any, artifact_generation: int, phase4_report: Any, phase4_generation: int, metadata: Mapping[str, Any], plan_snapshot: Any = None, motion_configuration: Any = None, robot_configuration: Any = None, grasp_snapshot: Any = None) -> dict[str, Any]:
        if artifact is None:
            raise ExperimentError("A current frozen artifact is required")
        if not isinstance(metadata, Mapping):
            raise ExperimentError("experiment metadata must be an object")
        if (
            isinstance(artifact_generation, bool)
            or isinstance(phase4_generation, bool)
            or int(artifact_generation) != int(phase4_generation)
        ):
            raise ExperimentError(
                "Frozen artifact and Phase4 PASS generations do not match"
            )
        if not isinstance(phase4_report, Mapping) or phase4_report.get("overall_status") != "PASS" or phase4_report.get("plan_fingerprint") != artifact.plan_fingerprint or phase4_report.get("report_version") != artifact.validation_report_version:
            raise ExperimentError("A current matching Phase4 PASS report is required")
        checked_plan = validate_plan_snapshot(plan_snapshot, artifact)
        path_type = str(metadata.get("path_type", "CUSTOM")).upper()
        fixture = str(metadata.get("fixture_condition", "NONE")).upper()
        if path_type not in PATH_TYPES:
            raise ExperimentError("path_type must be LINEAR, CURVED, COMPLEX, or CUSTOM")
        if fixture not in FIXTURE_CONDITIONS:
            raise ExperimentError("fixture_condition must be NONE, CLEARANCE, RIGID, or OTHER")
        label = _text(metadata.get("label"), "label", maximum=160)
        notes = _text(metadata.get("notes"), "notes", maximum=4000)
        now_ms = self._wall_time_ms()
        run_id = self.store.generate_run_id()
        planned_samples = []
        object_samples = checked_plan.get("combined_object_samples", []) if checked_plan else []
        for index, timestamp in enumerate(artifact.common_timestamps_s):
            cartesian = deepcopy(object_samples[index]) if index < len(object_samples) else None
            planned_samples.append({
                "sample_index": index,
                "time_from_start_s": float(timestamp),
                "left": {"joints_rad": list(artifact.left_positions_rad[index])},
                "right": {"joints_rad": list(artifact.right_positions_rad[index])},
                "combined_joints_rad": list(artifact.combined_positions_rad[index]),
                "cartesian": cartesian,
                "cartesian_source": "PHASE3_PLAN_TARGETS" if cartesian else "OFFLINE_URDF_FK_FALLBACK_REQUIRED",
            })
        speed_acc = []
        for waypoint in checked_plan.get("waypoints", []) if checked_plan else []:
            if isinstance(waypoint, Mapping):
                speed_acc.append({key: waypoint.get(key) for key in ("identifier", "speed_percent", "acceleration_percent")})
        manifest = {
            "schema_version": SCHEMA_VERSION, "run_id": run_id, "label": label,
            "state": "ARMED", "created_at": _utc_iso(now_ms * 1_000_000), "updated_at": _utc_iso(now_ms * 1_000_000),
            "units": {"time": "second", "wall_time": "unix_epoch_millisecond", "joint_position": "radian", "translation": "meter", "rotation": "radian", "joint_torque": "newton_meter"},
            "artifact_fingerprint": artifact.artifact_fingerprint, "plan_fingerprint": artifact.plan_fingerprint,
            "validation_generation": int(phase4_generation), "artifact_generation": int(artifact_generation), "validation_report_version": artifact.validation_report_version,
            "robot_configuration_revision": configuration_fingerprint(robot_configuration or {}),
            "calibration_revision": artifact.calibration_revision, "model_calibration_revision": artifact.model_calibration_revision,
            "grasp_content_revision": artifact.grasp_content_revision, "grasp_lock_generation": artifact.grasp_lock_generation, "grasp_lock_revision": artifact.grasp_lock_revision,
            "locked_grasp": _json_copy(grasp_snapshot), "trajectory_name": artifact.trajectory_name,
            "path_type": path_type, "fixture_condition": fixture, "notes": notes,
            "speed_acceleration_profile": speed_acc, "phase5_motion_configuration": _json_copy(motion_configuration or {}),
            "actual_source_semantics": "Existing visualization Actual selection: fresh dual Port10000 when available, otherwise cached ROS JointState; no SDK call",
            "recording_semantic": "OBSERVATIONAL ONLY — NO MOTION AUTHORITY",
            "raw_semantic": "AUTHORITATIVE RECORDED EVIDENCE",
            "aligned_semantic": ALIGNMENT_SEMANTIC,
            "result": {"status": "PENDING", "sample_counts": {"planned": len(planned_samples), "observations": 0, "commanded_left": 0, "commanded_right": 0, "actual_left": 0, "actual_right": 0, "torque_left": 0, "torque_right": 0}},
        }
        raw = {"schema_version": SCHEMA_VERSION, "run_id": run_id, "semantic": "AUTHORITATIVE RECORDED EVIDENCE", "planned": {"joint_order": list(JOINT_ORDER), "common_timestamps_s": list(artifact.common_timestamps_s), "samples": planned_samples}, "plan_snapshot": checked_plan, "execution": {"trajectory_id": None, "start_time_unix_ns": None}, "observations": [], "result": None}
        with self._lock:
            if self._manifest and self._manifest.get("state") in {"ARMED", "RECORDING"}:
                raise ExperimentError("An experiment recorder is already armed")
            self.store.create(manifest, raw)
            self._manifest, self._raw = manifest, raw
            self._last_signature, self._last_persist_ms, self._error = None, now_ms, None
        return self.state()

    def disarm(self) -> dict[str, Any]:
        with self._lock:
            if self._manifest is None or self._manifest.get("state") != "ARMED":
                raise ExperimentError("Only an unbound ARMED recorder may be disarmed")
            self._manifest["state"] = "DISARMED"
            self._manifest["updated_at"] = _utc_iso()
            self._manifest["result"]["status"] = "DISARMED_BEFORE_EXECUTION"
            self.store.update(self._manifest["run_id"], self._manifest, self._raw or {})
        return self.state()

    def bind_execution(self, execution: Mapping[str, Any]) -> None:
        with self._lock:
            if self._manifest is None or self._manifest.get("state") != "ARMED":
                return
            if not isinstance(execution, Mapping):
                raise ExperimentError("Accepted execution must be an object")
            execution_generation = execution.get("artifact_generation")
            if isinstance(execution_generation, bool) or not isinstance(
                execution_generation, int
            ):
                raise ExperimentError("Accepted execution has invalid artifact_generation")
            if execution.get("artifact_fingerprint") != self._manifest.get("artifact_fingerprint") or execution_generation != self._manifest.get("artifact_generation"):
                return
            trajectory_id = execution.get("trajectory_id")
            if not isinstance(trajectory_id, str) or not trajectory_id:
                raise ExperimentError("Accepted execution is missing trajectory_id")
            self._manifest["state"] = "RECORDING"
            self._manifest["active_trajectory_id"] = trajectory_id
            self._manifest["recording_started_at"] = _utc_iso()
            self._raw["execution"] = {key: deepcopy(execution.get(key)) for key in ("trajectory_id", "start_time_unix_ns", "duration_s", "artifact_fingerprint", "plan_fingerprint", "artifact_generation")}
            self.store.update(self._manifest["run_id"], self._manifest, self._raw)
            self._last_persist_ms = self._wall_time_ms()

    def record_error(self, error: Any) -> None:
        with self._lock:
            self._error = str(error)

    @staticmethod
    def _commanded(driver: Any) -> dict[str, Any]:
        command = driver.get("commanded_sample", {}) if isinstance(driver, Mapping) else {}
        if command.get("valid") is not True:
            return {"valid": False, "status": "MISSING", "sample_index": None, "time_from_start_s": None, "joints_rad": None, "source": command.get("source"), "error": driver.get("error") if isinstance(driver, Mapping) else "Driver feedback unavailable"}
        try:
            joints = _vector(command.get("joints_rad"), 6, "commanded.joints_rad")
            timestamp = _finite(command.get("time_from_start_s"), "commanded.time_from_start_s")
            index = command.get("sample_index")
            if isinstance(index, bool) or not isinstance(index, int):
                raise ExperimentError("commanded.sample_index must be an integer")
            return {"valid": True, "status": "VALID", "sample_index": index, "time_from_start_s": timestamp, "joints_rad": joints, "source": command.get("source"), "error": None}
        except ExperimentError as error:
            return {"valid": False, "status": "ERROR", "sample_index": None, "time_from_start_s": None, "joints_rad": None, "source": command.get("source"), "error": str(error)}

    def _actual(self, side_data: Any, source: Any, capture_ms: int) -> dict[str, Any]:
        item = side_data if isinstance(side_data, Mapping) else {}
        received = item.get("received_at_ms")
        valid = item.get("valid") is True
        try:
            joints = _vector(item.get("joint"), 6, "actual.joint") if valid else None
        except ExperimentError as error:
            return {"valid": False, "fresh": False, "status": "ERROR", "joints_rad": None, "received_at_ms": received if isinstance(received, int) else None, "age_ms": None, "source": source, "error": str(error)}
        received_valid = bool(
            not isinstance(received, bool)
            and isinstance(received, (int, float))
            and math.isfinite(float(received))
        )
        if valid and not received_valid:
            return {
                "valid": False,
                "fresh": False,
                "status": "ERROR",
                "joints_rad": joints,
                "received_at_ms": None,
                "age_ms": None,
                "source": source,
                "mapping": item.get("mapping"),
                "error": "Actual sample receive timestamp is missing or invalid",
            }
        age = max(0, capture_ms - int(received)) if valid and received_valid else None
        fresh = valid and age is not None and age <= self.actual_stale_after_ms
        return {"valid": valid, "fresh": fresh, "status": "FRESH" if fresh else ("STALE" if valid else "MISSING"), "joints_rad": joints, "received_at_ms": int(received) if received_valid else None, "age_ms": age, "source": source, "mapping": item.get("mapping"), "error": item.get("error") if not valid else (None if fresh else f"Actual sample age {age} ms exceeds {self.actual_stale_after_ms} ms")}

    def observe(self, phase5_state: Mapping[str, Any], actual: Mapping[str, Any], torque: Mapping[str, Any]) -> None:
        with self._lock:
            if self._manifest is None or self._manifest.get("state") != "RECORDING":
                return
            if not all(isinstance(value, Mapping) for value in (phase5_state, actual, torque)):
                raise ExperimentError("Observation inputs must be objects")
            trajectory_id = self._manifest.get("active_trajectory_id")
            execution = phase5_state.get("execution", {})
            feedback = phase5_state.get("driver_feedback", {})
            if execution.get("trajectory_id") != trajectory_id or feedback.get("trajectory_id") != trajectory_id:
                return
            capture_ms = self._wall_time_ms()
            drivers = feedback.get("drivers", {})
            observation = {
                "observation_index": len(self._raw["observations"]), "capture_wall_time_ms": capture_ms, "capture_wall_time": _utc_iso(capture_ms * 1_000_000),
                "execution_state": feedback.get("combined_state") or execution.get("state"), "timeline_s": (feedback.get("common_timeline") or {}).get("elapsed_s"),
                "common_timeline": deepcopy(feedback.get("common_timeline")),
                "commanded": {side: self._commanded(drivers.get(side, {})) for side in ("left", "right")},
                "actual": {side: self._actual(actual.get(side), actual.get("source"), capture_ms) for side in ("left", "right")},
                "torque": {side: deepcopy(torque.get(side, {"valid": False, "status": "MISSING", "values": None, "source": f"/{side}_jaka_driver/joint_torque_raw"})) for side in ("left", "right")},
                "relative_pose": {"valid": False, "status": "NOT_MEASURED", "source": None, "error": "No external relative-pose measurement source configured"},
                "tcp_fk": {"valid": False, "status": "DERIVED_OFFLINE", "source": "URDF_FK_FROM_RECORDED_ACTUAL_JOINTS", "values": None},
            }
            signature_payload = {"state": observation["execution_state"], "timeline": observation["common_timeline"], "commanded": observation["commanded"], "actual_received": {side: observation["actual"][side].get("received_at_ms") for side in ("left", "right")}, "torque_received": {side: observation["torque"][side].get("received_at_ms") for side in ("left", "right")}}
            signature = configuration_fingerprint(signature_payload)
            if signature != self._last_signature:
                self._raw["observations"].append(observation)
                self._last_signature = signature
                self._update_counts()
            if feedback.get("authoritative") is True and feedback.get("terminal") is True and feedback.get("combined_state") in TERMINAL_STATES:
                self._close(feedback)
            elif self._last_persist_ms is None or capture_ms - self._last_persist_ms >= 1000:
                # Persist at a bounded cadence so observational disk I/O cannot
                # dominate the existing 100 ms Phase5 status-poll path.
                self.store.update(self._manifest["run_id"], self._manifest, self._raw)
                self._last_persist_ms = capture_ms

    def _update_counts(self) -> None:
        observations = self._raw["observations"]
        counts = self._manifest["result"]["sample_counts"]
        counts["observations"] = len(observations)
        for kind in ("commanded", "actual", "torque"):
            for side in ("left", "right"):
                counts[f"{kind}_{side}"] = sum(1 for sample in observations if sample[kind][side].get("valid") is True)

    def _close(self, feedback: Mapping[str, Any]) -> None:
        self._update_counts()
        now = _utc_iso()
        result = {"status": feedback.get("combined_state"), "combined_state": feedback.get("combined_state"), "reason": feedback.get("reason"), "trajectory_id": feedback.get("trajectory_id"), "duration_s": (feedback.get("common_timeline") or {}).get("duration_s"), "recording_started_at": self._manifest.get("recording_started_at"), "recording_ended_at": now, "raw_observation_count": len(self._raw["observations"]), "sample_counts": deepcopy(self._manifest["result"]["sample_counts"]), "torque_availability": {side: self._manifest["result"]["sample_counts"][f"torque_{side}"] > 0 for side in ("left", "right")}}
        self._manifest["state"] = "CLOSED"
        self._manifest["updated_at"] = now
        self._manifest["result"] = result
        self._raw["result"] = deepcopy(result)
        aligned = align_planned_to_actual(self._raw)
        # EXP-0 raw/aligned evidence is the authoritative recording product.
        # Persist it first so a derived EXP-1 analysis defect can never prevent
        # terminal evidence from being closed safely.
        self.store.update(
            self._manifest["run_id"], self._manifest, self._raw, aligned
        )
        exp1_analysis = analyze_trajectory_tracking(
            self._raw, aligned, self._manifest
        )
        self.store.write_exp1_analysis(
            self._manifest["run_id"], exp1_analysis
        )
        # EXP-2 is derived/offline evidence only. Never let an EXP-2 defect
        # prevent the already-persisted raw/aligned/EXP-1 terminal closure.
        try:
            exp2_analysis = analyze_rigid_grasp(
                self._raw, self._manifest, EXP2_URDF_PATH
            )
            self.store.write_exp2_analysis(
                self._manifest["run_id"], exp2_analysis
            )
        except Exception as error:  # defensive isolation from motion/recording path
            # Persist an explicit derived-analysis failure record without
            # poisoning recorder completion or motion authority.
            try:
                self.store.write_exp2_analysis(
                    self._manifest["run_id"],
                    {
                        "schema_version": EXP2_SCHEMA_VERSION,
                        "experiment": "EXP-2 Rigid-Grasp Preservation",
                        "run_id": self._manifest["run_id"],
                        "status": "UNAVAILABLE",
                        "semantic": EXP2_SEMANTIC,
                        "error": f"Derived EXP-2 analysis failed: {error}",
                        "coverage": {},
                        "samples": [],
                    },
                )
            except Exception:
                pass

    def state(self) -> dict[str, Any]:
        with self._lock:
            manifest = deepcopy(self._manifest)
            error = self._error
        counts = deepcopy((manifest or {}).get("result", {}).get("sample_counts", {}))
        return {
            "ok": error is None,
            "schema_version": SCHEMA_VERSION,
            "state": manifest.get("state") if manifest else "IDLE",
            "run_id": manifest.get("run_id") if manifest else None,
            "armed_artifact_fingerprint": manifest.get("artifact_fingerprint") if manifest else None,
            "active_trajectory_id": manifest.get("active_trajectory_id") if manifest else None,
            "trajectory": {
                "name": manifest.get("trajectory_name"),
                "artifact_fingerprint": manifest.get("artifact_fingerprint"),
                "plan_fingerprint": manifest.get("plan_fingerprint"),
            } if manifest else None,
            "sample_counts": counts,
            "torque_availability": {
                side: counts.get(f"torque_{side}", 0) > 0
                for side in ("left", "right")
            },
            "error": error,
            "semantic": "OBSERVATIONAL ONLY — NO MOTION AUTHORITY",
        }
