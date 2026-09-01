"""Validated canonical World-to-base calibration/model alignment data.

This module is deliberately pure: it reads JSON, validates deterministic model
inputs, and never imports ROS or a robot driver.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
REVISION_ALGORITHM = "sha256"
CALIBRATION_STATE_MODEL_DEFAULT = "MODEL_DEFAULT"
CALIBRATION_STATE_PHYSICAL = "PHYSICAL_CALIBRATED"
PHYSICAL_CALIBRATION_NOT_CALIBRATED = "NOT_CALIBRATED"
PHYSICAL_CALIBRATION_CALIBRATED = "CALIBRATED"
TCP_CONTRACT_UNVERIFIED = "UNVERIFIED"
TCP_CONTRACT_TOOL0_J6_VERIFIED = "TOOL0_J6_KINEMATICALLY_VERIFIED"
TRANSLATION_UNIT = "meter"
ROTATION_UNIT = "radian"
ROTATION_CONVENTION = "RPY_EXTRINSIC_XYZ__R_EQ_RZ_RY_RX"
DEFAULT_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[1] / "config/world_frame_calibration.json"
)
DEFAULT_WEB_MODEL_METADATA_PATH = (
    Path(__file__).resolve().parents[1]
    / "web/assets/dual_jaka_a12_web.metadata.json"
)

_FRAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*$")


class WorldCalibrationError(ValueError):
    """Raised when canonical calibration data is missing or invalid."""


@dataclass(frozen=True)
class RpyTransform:
    translation_m: tuple[float, float, float]
    rotation_rpy_rad: tuple[float, float, float]

    def xacro_xyz(self) -> str:
        return " ".join(_format_number(value) for value in self.translation_m)

    def xacro_rpy(self) -> str:
        return " ".join(_format_number(value) for value in self.rotation_rpy_rad)

    def as_dict(self) -> dict[str, list[float]]:
        return {
            "translation_m": list(self.translation_m),
            "rotation_rpy_rad": list(self.rotation_rpy_rad),
        }


@dataclass(frozen=True)
class WorldCalibration:
    schema_version: int
    calibration_state: str
    physical_calibration: str
    world_frame: str
    left_base_frame: str
    right_base_frame: str
    world_to_left_base: RpyTransform
    world_to_right_base: RpyTransform
    source_method: str
    source_description: str
    provenance_status: str
    verification_status: str
    planning_tip_left: str
    planning_tip_right: str
    tcp_tool_contract_status: str
    revision: str

    @property
    def physically_calibrated(self) -> bool:
        return self.physical_calibration == "CALIBRATED"

    def xacro_mappings(self) -> Mapping[str, str]:
        return MappingProxyType(
            {
                "left_base_xyz": self.world_to_left_base.xacro_xyz(),
                "left_base_rpy": self.world_to_left_base.xacro_rpy(),
                "right_base_xyz": self.world_to_right_base.xacro_xyz(),
                "right_base_rpy": self.world_to_right_base.xacro_rpy(),
            }
        )

    def public_payload(self) -> dict[str, Any]:
        """Return a new JSON-safe copy for read-only inspection APIs."""
        return {
            "schema_version": self.schema_version,
            "calibration_state": self.calibration_state,
            "physical_calibration": self.physical_calibration,
            "physically_calibrated": self.physically_calibrated,
            "revision": self.revision,
            "frames": {
                "world": self.world_frame,
                "left_base": self.left_base_frame,
                "right_base": self.right_base_frame,
            },
            "units": {
                "translation": TRANSLATION_UNIT,
                "rotation": ROTATION_UNIT,
            },
            "rotation_convention": ROTATION_CONVENTION,
            "transforms": {
                "world_to_left_base": self.world_to_left_base.as_dict(),
                "world_to_right_base": self.world_to_right_base.as_dict(),
            },
            "source": {
                "method": self.source_method,
                "description": self.source_description,
            },
            "provenance": {
                "status": self.provenance_status,
                "verification_status": self.verification_status,
            },
            "tcp_tool_contract": {
                "status": self.tcp_tool_contract_status,
                "planning_tip_frames": {
                    "left": self.planning_tip_left,
                    "right": self.planning_tip_right,
                },
            },
        }


def _require_mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise WorldCalibrationError(f"{field} must be a JSON object")
    return value


def _require_exact_keys(value: Mapping[str, Any], expected: set[str], field: str) -> None:
    missing = sorted(expected - set(value))
    extra = sorted(set(value) - expected)
    if missing:
        raise WorldCalibrationError(f"{field} is missing required fields: {missing}")
    if extra:
        raise WorldCalibrationError(f"{field} contains unsupported fields: {extra}")


def _require_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise WorldCalibrationError(f"{field} must be a non-empty string")
    return value.strip()


def _require_enum(value: Any, expected: str, field: str) -> str:
    normalized = _require_string(value, field)
    if normalized != expected:
        raise WorldCalibrationError(f"{field} must be {expected!r}, found {normalized!r}")
    return normalized


def _require_one_of(value: Any, expected: set[str], field: str) -> str:
    normalized = _require_string(value, field)
    if normalized not in expected:
        raise WorldCalibrationError(
            f"{field} must be one of {sorted(expected)!r}, found {normalized!r}"
        )
    return normalized


def _require_frame(value: Any, field: str) -> str:
    frame = _require_string(value, field)
    if not _FRAME_PATTERN.fullmatch(frame):
        raise WorldCalibrationError(f"{field} is not a valid frame identifier: {frame!r}")
    return frame


def _require_vector3(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise WorldCalibrationError(f"{field} must contain exactly three numbers")
    result: list[float] = []
    for index, component in enumerate(value):
        if isinstance(component, bool) or not isinstance(component, (int, float)):
            raise WorldCalibrationError(f"{field}[{index}] must be a number, not boolean")
        number = float(component)
        if not math.isfinite(number):
            raise WorldCalibrationError(f"{field}[{index}] must be finite")
        result.append(number)
    return (result[0], result[1], result[2])


def _parse_transform(value: Any, field: str) -> RpyTransform:
    transform = _require_mapping(value, field)
    _require_exact_keys(transform, {"translation_m", "rotation_rpy_rad"}, field)
    translation = _require_vector3(transform["translation_m"], f"{field}.translation_m")
    rotation = _require_vector3(transform["rotation_rpy_rad"], f"{field}.rotation_rpy_rad")
    return RpyTransform(translation_m=translation, rotation_rpy_rad=rotation)


def _format_number(value: float) -> str:
    if value == 0.0:
        return "0.0"
    return format(value, ".15g")


def _critical_content(normalized: Mapping[str, Any]) -> dict[str, Any]:
    """Select all model-alignment and contract fields that define a revision."""
    return {
        "schema_version": normalized["schema_version"],
        "calibration_state": normalized["calibration_state"],
        "physical_calibration": normalized["physical_calibration"],
        "frames": normalized["frames"],
        "units": normalized["units"],
        "rotation_convention": normalized["rotation_convention"],
        "transforms": normalized["transforms"],
        "source": normalized["source"],
        "provenance": normalized["provenance"],
        "tcp_tool_contract": normalized["tcp_tool_contract"],
    }


def compute_calibration_revision(data: Mapping[str, Any]) -> str:
    """Compute the deterministic fingerprint from validated-normalized content."""
    normalized = normalize_calibration_data(data, verify_revision=False)
    encoded = json.dumps(
        _critical_content(normalized),
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return f"{REVISION_ALGORITHM}:{hashlib.sha256(encoded).hexdigest()}"


def normalize_calibration_data(
    data: Mapping[str, Any], *, verify_revision: bool = True
) -> dict[str, Any]:
    root = _require_mapping(data, "calibration")
    expected_root = {
        "schema_version",
        "calibration_state",
        "physical_calibration",
        "frames",
        "units",
        "rotation_convention",
        "transforms",
        "source",
        "provenance",
        "tcp_tool_contract",
        "revision",
    }
    _require_exact_keys(root, expected_root, "calibration")
    schema_version = root["schema_version"]
    if isinstance(schema_version, bool) or schema_version != SCHEMA_VERSION:
        raise WorldCalibrationError(
            f"unsupported calibration schema_version: {schema_version!r}"
        )

    frames = _require_mapping(root["frames"], "frames")
    _require_exact_keys(frames, {"world", "left_base", "right_base"}, "frames")
    units = _require_mapping(root["units"], "units")
    _require_exact_keys(units, {"translation", "rotation"}, "units")
    transforms = _require_mapping(root["transforms"], "transforms")
    _require_exact_keys(
        transforms,
        {"world_to_left_base", "world_to_right_base"},
        "transforms",
    )
    source = _require_mapping(root["source"], "source")
    _require_exact_keys(source, {"method", "description"}, "source")
    provenance = _require_mapping(root["provenance"], "provenance")
    _require_exact_keys(
        provenance, {"status", "verification_status"}, "provenance"
    )
    tcp = _require_mapping(root["tcp_tool_contract"], "tcp_tool_contract")
    _require_exact_keys(tcp, {"status", "planning_tip_frames"}, "tcp_tool_contract")
    tips = _require_mapping(tcp["planning_tip_frames"], "tcp_tool_contract.planning_tip_frames")
    _require_exact_keys(tips, {"left", "right"}, "tcp_tool_contract.planning_tip_frames")

    left_transform = _parse_transform(
        transforms["world_to_left_base"], "transforms.world_to_left_base"
    )
    right_transform = _parse_transform(
        transforms["world_to_right_base"], "transforms.world_to_right_base"
    )
    normalized = {
        "schema_version": SCHEMA_VERSION,
        "calibration_state": _require_one_of(
            root["calibration_state"],
            {CALIBRATION_STATE_MODEL_DEFAULT, CALIBRATION_STATE_PHYSICAL},
            "calibration_state",
        ),
        "physical_calibration": _require_one_of(
            root["physical_calibration"],
            {PHYSICAL_CALIBRATION_NOT_CALIBRATED, PHYSICAL_CALIBRATION_CALIBRATED},
            "physical_calibration",
        ),
        "frames": {
            "world": _require_frame(frames["world"], "frames.world"),
            "left_base": _require_frame(frames["left_base"], "frames.left_base"),
            "right_base": _require_frame(frames["right_base"], "frames.right_base"),
        },
        "units": {
            "translation": _require_enum(
                units["translation"], TRANSLATION_UNIT, "units.translation"
            ),
            "rotation": _require_enum(units["rotation"], ROTATION_UNIT, "units.rotation"),
        },
        "rotation_convention": _require_enum(
            root["rotation_convention"], ROTATION_CONVENTION, "rotation_convention"
        ),
        "transforms": {
            "world_to_left_base": left_transform.as_dict(),
            "world_to_right_base": right_transform.as_dict(),
        },
        "source": {
            "method": _require_one_of(
                source["method"],
                {"MODEL_DEFAULT_MIGRATION", "ONE_POINT_VIRTUAL_J6_TOUCH", "THREE_POINT_VIRTUAL_J6_TOUCH"},
                "source.method",
            ),
            "description": _require_string(source["description"], "source.description"),
        },
        "provenance": {
            "status": _require_one_of(
                provenance["status"],
                {"MIGRATED_MODEL_DEFAULT", "PHYSICAL_TOUCH_CALIBRATION"},
                "provenance.status",
            ),
            "verification_status": _require_one_of(
                provenance["verification_status"],
                {"NOT_PHYSICALLY_VERIFIED", "PHYSICALLY_VERIFIED_ONE_POINT_TRANSLATION", "PHYSICALLY_CALIBRATED_SINGLE_PASS_NOT_REPEATABILITY_VERIFIED"},
                "provenance.verification_status",
            ),
        },
        "tcp_tool_contract": {
            "status": _require_one_of(
                tcp["status"],
                {TCP_CONTRACT_UNVERIFIED, TCP_CONTRACT_TOOL0_J6_VERIFIED},
                "tcp_tool_contract.status",
            ),
            "planning_tip_frames": {
                "left": _require_frame(tips["left"], "tcp_tool_contract.planning_tip_frames.left"),
                "right": _require_frame(tips["right"], "tcp_tool_contract.planning_tip_frames.right"),
            },
        },
        "revision": _require_string(root["revision"], "revision"),
    }
    if verify_revision:
        expected_revision = compute_calibration_revision(normalized)
        if normalized["revision"] != expected_revision:
            raise WorldCalibrationError(
                "calibration revision does not match validated critical content: "
                f"expected {expected_revision}, found {normalized['revision']}"
            )
    return normalized


def load_world_calibration(
    path: str | Path = DEFAULT_CALIBRATION_PATH,
) -> WorldCalibration:
    calibration_path = Path(path)
    try:
        raw = json.loads(calibration_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise WorldCalibrationError(
            f"canonical world calibration artifact is missing: {calibration_path}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise WorldCalibrationError(
            f"canonical world calibration artifact is invalid JSON: {exc}"
        ) from exc
    normalized = normalize_calibration_data(raw)
    return WorldCalibration(
        schema_version=normalized["schema_version"],
        calibration_state=normalized["calibration_state"],
        physical_calibration=normalized["physical_calibration"],
        world_frame=normalized["frames"]["world"],
        left_base_frame=normalized["frames"]["left_base"],
        right_base_frame=normalized["frames"]["right_base"],
        world_to_left_base=_parse_transform(
            normalized["transforms"]["world_to_left_base"], "world_to_left_base"
        ),
        world_to_right_base=_parse_transform(
            normalized["transforms"]["world_to_right_base"], "world_to_right_base"
        ),
        source_method=normalized["source"]["method"],
        source_description=normalized["source"]["description"],
        provenance_status=normalized["provenance"]["status"],
        verification_status=normalized["provenance"]["verification_status"],
        planning_tip_left=normalized["tcp_tool_contract"]["planning_tip_frames"]["left"],
        planning_tip_right=normalized["tcp_tool_contract"]["planning_tip_frames"]["right"],
        tcp_tool_contract_status=normalized["tcp_tool_contract"]["status"],
        revision=normalized["revision"],
    )


def get_world_calibration_revision(
    path: str | Path = DEFAULT_CALIBRATION_PATH,
) -> str:
    """Stable public revision accessor for later plan/fingerprint integration."""
    return load_world_calibration(path).revision


def get_web_model_calibration_revision(
    path: str | Path = DEFAULT_WEB_MODEL_METADATA_PATH,
) -> str:
    """Read the checked-in Web URDF metadata revision for backend comparison."""
    metadata_path = Path(path)
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise WorldCalibrationError(
            f"Web model calibration metadata is unavailable or invalid: {error}"
        ) from error
    if not isinstance(payload, dict):
        raise WorldCalibrationError("Web model calibration metadata must be an object")
    revision = payload.get("calibration_revision")
    if not isinstance(revision, str) or not revision.strip():
        raise WorldCalibrationError(
            "Web model calibration metadata has no calibration_revision"
        )
    return revision.strip()


def world_calibration_public_payload(
    path: str | Path = DEFAULT_CALIBRATION_PATH,
) -> dict[str, Any]:
    return load_world_calibration(path).public_payload()
