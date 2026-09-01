"""Pure authoritative state for an operator-configurable rigid dual grasp.

This module performs validation and deterministic state transitions only.  It
has no ROS, MoveIt, JAKA, network, planning, or execution dependency.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

try:
    from dual_arm_app.backend.object_grasp_model import ObjectGraspModel, RigidTransform
except ImportError:
    from .object_grasp_model import ObjectGraspModel, RigidTransform


GRASP_UNLOCKED = "GRASP_UNLOCKED"
GRASP_LOCKED = "GRASP_LOCKED"
SYNTHETIC_DEFAULT_DRAFT = "SYNTHETIC_DEFAULT_DRAFT"
OPERATOR_CONFIGURED_RIGID_GRASP = "OPERATOR_CONFIGURED_RIGID_GRASP"
OPERATOR_CONFIGURED_UNLOCKED_DRAFT = "OPERATOR_CONFIGURED_UNLOCKED_DRAFT"
PLANNER_INTEGRATION_READY = "PRE_P5_C1_C2_INTEGRATED"
ROTATION_CONVENTION = "R = Rz(yaw) * Ry(pitch) * Rx(roll)"
TRANSLATION_UNIT = "meter"
ROTATION_UNIT = "radian"
MAX_TRANSLATION_M = 3.0
MAX_ABSOLUTE_RPY_RAD = 2.0 * math.pi


class RigidGraspConfigurationError(ValueError):
    """Raised when a draft or state transition is malformed."""


def _number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RigidGraspConfigurationError(f"{field_name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise RigidGraspConfigurationError(f"{field_name} must be a finite number")
    return 0.0 if result == 0.0 else result


def _vector3(value: Any, field_name: str) -> tuple[float, float, float]:
    if (
        isinstance(value, (str, bytes, bool))
        or not isinstance(value, Sequence)
        or len(value) != 3
    ):
        raise RigidGraspConfigurationError(
            f"{field_name} must contain exactly three finite numbers"
        )
    values = tuple(
        _number(component, f"{field_name}[{index}]")
        for index, component in enumerate(value)
    )
    return values  # type: ignore[return-value]


def _mapping(value: Any, field_name: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise RigidGraspConfigurationError(f"{field_name} must be an object")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], field_name: str) -> None:
    missing = sorted(expected - set(value))
    extra = sorted(set(value) - expected)
    if missing:
        raise RigidGraspConfigurationError(
            f"{field_name} is missing required fields: {missing}"
        )
    if extra:
        raise RigidGraspConfigurationError(
            f"{field_name} contains unsupported fields: {extra}"
        )


@dataclass(frozen=True)
class RigidGraspPose:
    """Canonical object-relative pose in meters and fixed-axis RPY radians."""

    translation_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]
    transform: RigidTransform = field(init=False, repr=False)

    def __post_init__(self) -> None:
        translation = _vector3(self.translation_m, "translation_m")
        rpy = _vector3(self.rpy_rad, "rpy_rad")
        if any(abs(component) > MAX_TRANSLATION_M for component in translation):
            raise RigidGraspConfigurationError(
                f"translation_m must be within +/-{MAX_TRANSLATION_M} m"
            )
        if any(abs(component) > MAX_ABSOLUTE_RPY_RAD for component in rpy):
            raise RigidGraspConfigurationError(
                f"rpy_rad must be within +/-{MAX_ABSOLUTE_RPY_RAD} rad"
            )
        object.__setattr__(self, "translation_m", translation)
        object.__setattr__(self, "rpy_rad", rpy)
        object.__setattr__(
            self,
            "transform",
            RigidTransform.from_translation_rpy(translation, rpy),
        )

    @classmethod
    def from_payload(cls, value: Any, field_name: str) -> "RigidGraspPose":
        payload = _mapping(value, field_name)
        _exact_keys(payload, {"translation_m", "rpy_rad"}, field_name)
        return cls(
            translation_m=payload["translation_m"],
            rpy_rad=payload["rpy_rad"],
        )

    def as_payload(self) -> dict[str, list[float]]:
        return {
            "translation_m": list(self.translation_m),
            "rpy_rad": list(self.rpy_rad),
        }


@dataclass(frozen=True)
class RigidGraspContent:
    left: RigidGraspPose
    right: RigidGraspPose

    def __post_init__(self) -> None:
        if not isinstance(self.left, RigidGraspPose):
            raise TypeError("left must be RigidGraspPose")
        if not isinstance(self.right, RigidGraspPose):
            raise TypeError("right must be RigidGraspPose")

    @classmethod
    def from_payload(cls, value: Any) -> "RigidGraspContent":
        payload = _mapping(value, "grasp configuration")
        _exact_keys(payload, {"left", "right"}, "grasp configuration")
        return cls(
            left=RigidGraspPose.from_payload(payload["left"], "left"),
            right=RigidGraspPose.from_payload(payload["right"], "right"),
        )

    @property
    def model(self) -> ObjectGraspModel:
        return ObjectGraspModel(self.left.transform, self.right.transform)

    def as_payload(self) -> dict[str, dict[str, list[float]]]:
        return {"left": self.left.as_payload(), "right": self.right.as_payload()}


def compute_grasp_content_revision(content: RigidGraspContent) -> str:
    if not isinstance(content, RigidGraspContent):
        raise TypeError("content must be RigidGraspContent")
    canonical = {
        "left": content.left.as_payload(),
        "right": content.right.as_payload(),
        "units": {"translation": TRANSLATION_UNIT, "rotation": ROTATION_UNIT},
        "rotation_convention": ROTATION_CONVENTION,
        "semantic": OPERATOR_CONFIGURED_RIGID_GRASP,
    }
    encoded = json.dumps(
        canonical,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return f"sha256:{hashlib.sha256(encoded).hexdigest()}"


def compute_lock_revision(content_revision: str, lock_generation: int) -> str:
    encoded = json.dumps(
        {
            "content_revision": content_revision,
            "lock_generation": lock_generation,
            "state": GRASP_LOCKED,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return f"sha256:{hashlib.sha256(encoded).hexdigest()}"


@dataclass(frozen=True)
class LockedRigidGraspSnapshot:
    content: RigidGraspContent
    content_revision: str
    lock_generation: int
    lock_revision: str

    def as_payload(self) -> dict[str, Any]:
        return {
            **self.content.as_payload(),
            "content_revision": self.content_revision,
            "lock_generation": self.lock_generation,
            "lock_revision": self.lock_revision,
        }


def synthetic_default_content() -> RigidGraspContent:
    return RigidGraspContent(
        left=RigidGraspPose((0.0, 0.25, 0.0), (0.0, 0.0, 0.0)),
        right=RigidGraspPose((0.0, -0.25, 0.0), (0.0, 0.0, 0.0)),
    )


class AuthoritativeRigidGraspState:
    """Thread-safe draft/locked lifecycle with copy-safe JSON snapshots."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._draft = synthetic_default_content()
        self._draft_source = SYNTHETIC_DEFAULT_DRAFT
        self._locked: LockedRigidGraspSnapshot | None = None
        self._lock_generation = 0

    def _snapshot_unlocked(self) -> dict[str, Any]:
        content_revision = compute_grasp_content_revision(self._draft)
        return {
            "state": GRASP_LOCKED if self._locked is not None else GRASP_UNLOCKED,
            "draft": self._draft.as_payload(),
            "draft_source": self._draft_source,
            "draft_content_revision": content_revision,
            "locked_snapshot": (
                self._locked.as_payload() if self._locked is not None else None
            ),
            "authoritative_revision": (
                self._locked.lock_revision if self._locked is not None else None
            ),
            "lock_generation": self._lock_generation,
            "units": {"translation": TRANSLATION_UNIT, "rotation": ROTATION_UNIT},
            "rotation_convention": ROTATION_CONVENTION,
            "semantic": OPERATOR_CONFIGURED_RIGID_GRASP,
            "physically_calibrated": False,
            "planner_integration": PLANNER_INTEGRATION_READY,
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_unlocked()

    def locked_snapshot(self) -> LockedRigidGraspSnapshot | None:
        """Return the immutable locked authority object, never the editable draft."""
        with self._lock:
            return self._locked

    def lock(self, payload: Any) -> dict[str, Any]:
        content = RigidGraspContent.from_payload(payload)
        content_revision = compute_grasp_content_revision(content)
        with self._lock:
            self._lock_generation += 1
            self._draft = content
            self._draft_source = OPERATOR_CONFIGURED_RIGID_GRASP
            self._locked = LockedRigidGraspSnapshot(
                content=content,
                content_revision=content_revision,
                lock_generation=self._lock_generation,
                lock_revision=compute_lock_revision(
                    content_revision, self._lock_generation
                ),
            )
            return self._snapshot_unlocked()

    def unlock(self) -> dict[str, Any]:
        with self._lock:
            if self._locked is not None:
                self._draft = self._locked.content
            self._draft_source = OPERATOR_CONFIGURED_UNLOCKED_DRAFT
            self._locked = None
            self._lock_generation += 1
            return self._snapshot_unlocked()
