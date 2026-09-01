"""Developer-edited Planning Initial Joint Configuration.

OFFLINE / PLAN ONLY. This module contains no ROS, JAKA driver, endpoint, or
motion dependency. Changing these values changes only the joint configuration
used to seed planning and initialize the Planned Ghost; it is not World/Base
calibration, a Home pose, a live robot state, or an execution target.
"""

from __future__ import annotations

import math
from types import MappingProxyType
from typing import Any, Mapping, Sequence


PLANNING_START_STATE_SOURCE = "CODE CONFIG — planning_start_state_config.py"
PLANNING_START_STATE_UNIT = "radian"
PLANNING_START_JOINT_NAMES = tuple(
    [f"left_joint_{index}" for index in range(1, 7)]
    + [f"right_joint_{index}" for index in range(1, 7)]
)

# DEVELOPER CONFIGURATION: manually edit ONLY these 12 radian values when the
# desired planning initial joint configuration changes. Keep exactly six Left
# values followed by exactly six Right values in the documented joint order.
PLANNING_START_STATE_RAD = MappingProxyType({
    "left": (
        3.1399999999999997,   # left_joint_1
        0.5187280000000003,   # left_joint_2
        -0.8000720000000001,  # left_joint_3
        0.0,                  # left_joint_4
        0.7988159999999995,   # left_joint_5
        0.0,                  # left_joint_6
    ),
    "right": (
        0.0,                 # right_joint_1
        2.6162480000000015,  # right_joint_2
        0.7988159999999995,  # right_joint_3
        0.0,                 # right_joint_4
        2.3399279999999996,  # right_joint_5
        0.0,                 # right_joint_6
    ),
})


class PlanningStartStateConfigError(ValueError):
    """Raised when the developer-defined planning start state is invalid."""


def _arm_values(value: Any, label: str) -> tuple[float, ...]:
    if (
        isinstance(value, (str, bytes, bool))
        or not isinstance(value, Sequence)
        or len(value) != 6
    ):
        raise PlanningStartStateConfigError(
            f"{label} must contain exactly 6 finite radian values"
        )
    normalized: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise PlanningStartStateConfigError(
                f"{label}[{index}] must be a finite radian number"
            )
        converted = float(item)
        if not math.isfinite(converted):
            raise PlanningStartStateConfigError(
                f"{label}[{index}] must be a finite radian number"
            )
        normalized.append(converted)
    return tuple(normalized)


def normalize_planning_start_state_rad(
    value: Any,
    joint_limits: Any = None,
) -> tuple[float, ...]:
    """Validate and flatten Left then Right, optionally against model limits."""
    if not isinstance(value, Mapping):
        raise PlanningStartStateConfigError(
            "Planning Start State must contain left and right joint arrays"
        )
    combined = (
        *_arm_values(value.get("left"), "left"),
        *_arm_values(value.get("right"), "right"),
    )
    if joint_limits is not None:
        lower = getattr(joint_limits, "lower_rad", None)
        upper = getattr(joint_limits, "upper_rad", None)
        if not isinstance(lower, Sequence) or not isinstance(upper, Sequence):
            raise PlanningStartStateConfigError("canonical joint limits are invalid")
        if len(lower) != 12 or len(upper) != 12:
            raise PlanningStartStateConfigError(
                "canonical joint limits must contain exactly 12 values"
            )
        for index, (position, minimum, maximum) in enumerate(
            zip(combined, lower, upper)
        ):
            if position < minimum or position > maximum:
                raise PlanningStartStateConfigError(
                    f"{PLANNING_START_JOINT_NAMES[index]} is outside canonical "
                    f"position limits [{minimum}, {maximum}] rad"
                )
    return combined


def configured_planning_start_state_rad(joint_limits: Any = None) -> tuple[float, ...]:
    """Return the one validated code-defined default in canonical joint order."""
    return normalize_planning_start_state_rad(PLANNING_START_STATE_RAD, joint_limits)


def planning_start_state_payload(joint_limits: Any = None) -> dict[str, Any]:
    """Build a defensive, read-only transport payload with no side effects."""
    combined = configured_planning_start_state_rad(joint_limits)
    return {
        "ok": True,
        "source": PLANNING_START_STATE_SOURCE,
        "units": PLANNING_START_STATE_UNIT,
        "joint_names": list(PLANNING_START_JOINT_NAMES),
        "left": list(combined[:6]),
        "right": list(combined[6:]),
    }


# Fail fast during backend import if the manually edited source configuration is
# malformed. Canonical position limits are also checked where the model limits
# are available in the planning node.
PLANNING_START_STATE_COMBINED_RAD = configured_planning_start_state_rad()
