"""Persistent, motion-free storage for Digital Twin Center paths."""

from __future__ import annotations

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCHEMA = "jaka-dual-arm-center-path"
SCHEMA_VERSION = 1
MAX_NAME_LENGTH = 80


class CenterPathStoreError(ValueError):
    """Raised when a stored Center path request is invalid."""


def _utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def normalize_center_path_name(value: Any) -> str:
    name = str(value or "").strip()
    if not name:
        raise CenterPathStoreError("path name is empty")
    if len(name) > MAX_NAME_LENGTH:
        raise CenterPathStoreError(f"path name exceeds {MAX_NAME_LENGTH} characters")
    if name in {".", ".."} or ".." in name:
        raise CenterPathStoreError("path name must not contain '..'")
    if any(character in name for character in ("/", "\\", "\0")):
        raise CenterPathStoreError("path name must not contain path separators")
    if any(ord(character) < 32 for character in name):
        raise CenterPathStoreError("path name must not contain control characters")
    return name


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise CenterPathStoreError(f"{label} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise CenterPathStoreError(f"{label} must be a finite number") from error
    if not math.isfinite(number):
        raise CenterPathStoreError(f"{label} must be a finite number")
    return number


def _vector3(value: Any, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CenterPathStoreError(f"{label} must contain exactly 3 values")
    return [_finite_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def _percent(value: Any, label: str) -> float:
    number = 100.0 if value is None else _finite_number(value, label)
    if number <= 0.0 or number > 100.0:
        raise CenterPathStoreError(f"{label} must be within (0, 100]")
    return number


def normalize_center_path_payload(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise CenterPathStoreError("path payload must be an object")
    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise CenterPathStoreError("path must contain at least one Center waypoint")

    waypoints: list[dict[str, Any]] = []
    for index, waypoint in enumerate(raw_waypoints):
        if not isinstance(waypoint, dict):
            raise CenterPathStoreError(f"waypoints[{index}] must be an object")
        item = {
            "identifier": f"W{index}",
            "translation_m": _vector3(waypoint.get("translation_m"), f"waypoints[{index}].translation_m"),
            "rpy_rad": _vector3(waypoint.get("rpy_rad"), f"waypoints[{index}].rpy_rad"),
            "speed_percent": _percent(
                waypoint.get("speed_percent"),
                f"waypoints[{index}].speed_percent",
            ),
            "acceleration_percent": _percent(
                waypoint.get("acceleration_percent"),
                f"waypoints[{index}].acceleration_percent",
            ),
        }
        waypoints.append(item)

    fixed_orientation = _vector3(
        payload.get("fixed_orientation_rpy_rad", [0.0, 0.0, 0.0]),
        "fixed_orientation_rpy_rad",
    )
    segment_duration = _finite_number(payload.get("segment_duration_s", 2.0), "segment_duration_s")
    if segment_duration <= 0.0:
        raise CenterPathStoreError("segment_duration_s must be greater than zero")
    samples_per_segment = int(payload.get("samples_per_segment", 3))
    if samples_per_segment < 2:
        raise CenterPathStoreError("samples_per_segment must be at least 2")
    candidate_attempts = int(payload.get("candidate_attempts_per_arm", 3))
    if candidate_attempts < 1:
        raise CenterPathStoreError("candidate_attempts_per_arm must be at least 1")

    return {
        "waypoints": waypoints,
        "fixed_orientation_rpy_rad": fixed_orientation,
        "orientation_source": str(payload.get("orientation_source") or "PER-WAYPOINT 6D CENTER POSE"),
        "segment_duration_s": segment_duration,
        "samples_per_segment": samples_per_segment,
        "candidate_attempts_per_arm": candidate_attempts,
    }


class CenterPathStore:
    def __init__(self, directory: Path):
        self.directory = Path(directory)

    def _ensure_directory(self) -> Path:
        self.directory.mkdir(parents=True, exist_ok=True)
        return self.directory

    def _path_for_name(self, name: Any) -> tuple[str, Path]:
        normalized_name = normalize_center_path_name(name)
        directory = self._ensure_directory().resolve()
        path = (directory / f"{normalized_name}.json").resolve()
        if path.parent != directory:
            raise CenterPathStoreError("path name escapes the saved-path directory")
        return normalized_name, path

    def save(self, name: Any, payload: Any, *, overwrite: bool) -> dict[str, Any]:
        normalized_name, path = self._path_for_name(name)
        if path.exists() and not overwrite:
            raise CenterPathStoreError(f"saved path already exists: {normalized_name}")
        normalized_path = normalize_center_path_payload(payload)
        now = _utc_timestamp()
        document = {
            "schema": SCHEMA,
            "schema_version": SCHEMA_VERSION,
            "name": normalized_name,
            "saved_at": now,
            "path": normalized_path,
        }
        path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return {
            "ok": True,
            "message": "center path saved",
            "name": normalized_name,
            "file": path.name,
            "waypoint_count": len(normalized_path["waypoints"]),
            "document": document,
        }

    def load(self, name: Any) -> dict[str, Any]:
        normalized_name, path = self._path_for_name(name)
        if not path.exists():
            raise CenterPathStoreError(f"saved path not found: {normalized_name}")
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("schema") != SCHEMA or document.get("schema_version") != SCHEMA_VERSION:
            raise CenterPathStoreError("saved path schema is unsupported")
        normalized_path = normalize_center_path_payload(document.get("path"))
        return {
            "ok": True,
            "message": "center path loaded",
            "name": normalized_name,
            "file": path.name,
            "document": {
                "schema": SCHEMA,
                "schema_version": SCHEMA_VERSION,
                "name": normalized_name,
                "saved_at": document.get("saved_at"),
                "path": normalized_path,
            },
        }

    def list(self) -> dict[str, Any]:
        directory = self._ensure_directory()
        items: list[dict[str, Any]] = []
        warnings: list[str] = []
        for path in sorted(directory.glob("*.json"), key=lambda item: item.name.casefold()):
            try:
                document = json.loads(path.read_text(encoding="utf-8"))
                if document.get("schema") != SCHEMA or document.get("schema_version") != SCHEMA_VERSION:
                    raise CenterPathStoreError("unsupported schema")
                normalized_path = normalize_center_path_payload(document.get("path"))
                stat = path.stat()
                items.append({
                    "name": str(document.get("name") or path.stem),
                    "file": path.name,
                    "waypoint_count": len(normalized_path["waypoints"]),
                    "saved_at": document.get("saved_at"),
                    "modified_at": datetime.fromtimestamp(
                        stat.st_mtime, timezone.utc,
                    ).isoformat(timespec="seconds"),
                })
            except Exception as error:
                warnings.append(f"{path.name}: {error}")
        return {"ok": True, "paths": items, "warnings": warnings}

    def rename(self, old_name: Any, new_name: Any) -> dict[str, Any]:
        old_normalized, old_path = self._path_for_name(old_name)
        new_normalized, new_path = self._path_for_name(new_name)
        if not old_path.exists():
            raise CenterPathStoreError(f"saved path not found: {old_normalized}")
        if new_path.exists() and new_path != old_path:
            raise CenterPathStoreError(f"saved path already exists: {new_normalized}")
        document = self.load(old_normalized)["document"]
        document["name"] = new_normalized
        document["saved_at"] = _utc_timestamp()
        if new_path == old_path:
            old_path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        else:
            new_path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            old_path.unlink()
        return {
            "ok": True,
            "message": "center path renamed",
            "old_name": old_normalized,
            "name": new_normalized,
            "file": new_path.name,
        }

    def delete(self, name: Any) -> dict[str, Any]:
        normalized_name, path = self._path_for_name(name)
        if not path.exists():
            raise CenterPathStoreError(f"saved path not found: {normalized_name}")
        path.unlink()
        return {
            "ok": True,
            "message": "center path deleted",
            "name": normalized_name,
            "file": path.name,
        }
