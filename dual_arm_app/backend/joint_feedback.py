"""Pure helpers for read-only Digital Twin joint feedback.

This module intentionally has no ROS, FastAPI, or JAKA dependencies so its
normalization rules can be validated entirely offline.
"""

from __future__ import annotations

import math
import time
from collections.abc import Iterable, Sequence
from typing import Any


JOINT_COUNT = 6


def expected_joint_name_aliases(side: str) -> tuple[tuple[str, ...], ...]:
    """Return the explicit, conservative aliases supported for one arm."""
    if side not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    return tuple(
        (
            f"{side}_joint_{index}",
            f"joint_{index}",
            f"joint{index}",
        )
        for index in range(1, JOINT_COUNT + 1)
    )


def _as_list(value: Any, label: str) -> list[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Iterable):
        raise ValueError(f"{label} must be an iterable")
    return list(value)


def _finite_joint(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be a finite number")
    return converted


def normalize_joint_state_message(
    names: Any,
    positions: Any,
    expected_joint_names: Sequence[Sequence[str]],
) -> dict[str, Any]:
    """Normalize JointState-like arrays without mutating caller-owned data.

    Non-empty names select strict alias-based mapping. Positional fallback is
    permitted only for an empty name array and exactly six finite positions.
    """
    try:
        name_values = _as_list(names, "names")
        position_values = _as_list(positions, "positions")
        alias_groups = [tuple(group) for group in expected_joint_names]

        if len(alias_groups) != JOINT_COUNT or any(not group for group in alias_groups):
            raise ValueError("expected_joint_names must define six alias groups")

        if not name_values:
            if len(position_values) != JOINT_COUNT:
                raise ValueError(
                    "unnamed JointState requires exactly six positions"
                )
            joints = [
                _finite_joint(value, f"positions[{index}]")
                for index, value in enumerate(position_values)
            ]
            return {
                "valid": True,
                "joint": joints,
                "mapping": "position_fallback",
                "error": None,
                "names": [],
            }

        if len(name_values) != len(position_values):
            raise ValueError("names and positions must have the same length")
        if any(not isinstance(name, str) or not name for name in name_values):
            raise ValueError("all supplied joint names must be non-empty strings")
        if len(set(name_values)) != len(name_values):
            raise ValueError("duplicate joint names are not allowed")

        name_to_index = {name: index for index, name in enumerate(name_values)}
        joints: list[float] = []
        for joint_index, aliases in enumerate(alias_groups, start=1):
            matches = [name for name in aliases if name in name_to_index]
            if not matches:
                raise ValueError(f"missing expected joint {joint_index}")
            if len(matches) > 1:
                raise ValueError(f"multiple aliases supplied for joint {joint_index}")
            position_index = name_to_index[matches[0]]
            joints.append(
                _finite_joint(
                    position_values[position_index],
                    f"position for {matches[0]}",
                )
            )

        return {
            "valid": True,
            "joint": joints,
            "mapping": "name",
            "error": None,
            "names": list(name_values),
        }
    except (TypeError, ValueError) as error:
        safe_names = list(names) if isinstance(names, (list, tuple)) else []
        return {
            "valid": False,
            "joint": None,
            "mapping": None,
            "error": str(error),
            "names": safe_names,
        }


def wall_clock_ms() -> int:
    """Return integer Unix epoch milliseconds suitable for JSON age reports."""
    return time.time_ns() // 1_000_000


def build_digital_twin_joint_status(
    cache: dict[str, dict[str, Any]],
    server_time_ms: int | None = None,
) -> dict[str, Any]:
    """Build a copied, JSON-safe response from a two-arm feedback cache."""
    now_ms = wall_clock_ms() if server_time_ms is None else int(server_time_ms)
    response: dict[str, Any] = {
        "ok": False,
        "source": "ros_joint_state_cache",
        "unit": "radian",
        "server_time_ms": now_ms,
    }

    both_valid = True
    for side in ("left", "right"):
        side_cache = cache.get(side, {})
        joint = side_cache.get("joint")
        received_at_ms = side_cache.get("received_at_ms")
        valid = (
            isinstance(joint, (list, tuple))
            and len(joint) == JOINT_COUNT
            and all(
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(float(value))
                for value in joint
            )
            and isinstance(received_at_ms, (int, float))
            and not isinstance(received_at_ms, bool)
            and math.isfinite(float(received_at_ms))
        )
        both_valid = both_valid and valid
        response[side] = {
            "joint": [float(value) for value in joint] if valid else None,
            "received_at_ms": int(received_at_ms) if valid else None,
            "age_ms": max(0, now_ms - int(received_at_ms)) if valid else None,
            "valid": valid,
            "mapping": side_cache.get("mapping"),
            "error": side_cache.get("error"),
            "names": list(side_cache.get("names") or []),
        }

    response["ok"] = both_valid
    return response
