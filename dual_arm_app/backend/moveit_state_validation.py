"""Validation-only bridge for MoveIt's GetStateValidity service.

The pure normalization and response parsing paths are intentionally testable
without a running ROS graph. This module creates no publishers or actions.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any

try:
    from dual_arm_app.backend.sampled_path_validation import (
        generate_trajectory_interior_samples,
        normalize_sampled_path_options,
    )
except ImportError:
    try:
        from .sampled_path_validation import (
            generate_trajectory_interior_samples,
            normalize_sampled_path_options,
        )
    except ImportError:
        from sampled_path_validation import (
            generate_trajectory_interior_samples,
            normalize_sampled_path_options,
        )


SERVICE_NAME = "/check_state_validity"
PLANNING_GROUP = "dual_arm"
EXPECTED_JOINTS = tuple(
    f"{side}_joint_{index}"
    for side in ("left", "right")
    for index in range(1, 7)
)


class TrajectoryValidationInputError(ValueError):
    """Raised when a validation request is structurally unsafe or ambiguous."""


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TrajectoryValidationInputError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise TrajectoryValidationInputError(f"{label} must be a finite number")
    return converted


def _joint_array(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise TrajectoryValidationInputError(
            f"{label} must contain exactly six joint values"
        )
    return [
        _finite_number(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    ]


def normalize_trajectory(trajectory: Any) -> dict[str, Any]:
    """Validate and copy one browser trajectory without trusting its caller."""
    if not isinstance(trajectory, dict):
        raise TrajectoryValidationInputError("trajectory must be an object")
    points = trajectory.get("points")
    if not isinstance(points, list) or not points:
        raise TrajectoryValidationInputError(
            "trajectory.points must contain at least one point"
        )

    normalized_points: list[dict[str, Any]] = []
    previous_time: float | None = None
    for point_index, point in enumerate(points):
        if not isinstance(point, dict):
            raise TrajectoryValidationInputError(
                f"trajectory.points[{point_index}] must be an object"
            )
        timestamp = _finite_number(
            point.get("time_from_start_s"),
            f"trajectory.points[{point_index}].time_from_start_s",
        )
        if timestamp < 0:
            raise TrajectoryValidationInputError(
                f"trajectory.points[{point_index}].time_from_start_s must be >= 0"
            )
        if previous_time is not None and timestamp <= previous_time:
            raise TrajectoryValidationInputError(
                "trajectory timestamps must be strictly increasing"
            )
        previous_time = timestamp
        normalized_points.append(
            {
                "time_from_start_s": timestamp,
                "left": _joint_array(
                    point.get("left"), f"trajectory.points[{point_index}].left"
                ),
                "right": _joint_array(
                    point.get("right"), f"trajectory.points[{point_index}].right"
                ),
            }
        )

    name = trajectory.get("name")
    normalized_name = name.strip() if isinstance(name, str) and name.strip() else "Unnamed"
    return {"name": normalized_name, "points": normalized_points}


def joint_positions(point: dict[str, Any]) -> list[float]:
    """Flatten one normalized point in the canonical left-then-right order."""
    return [*point["left"], *point["right"]]


def _contact_value(contact: Any, *names: str, default: Any = None) -> Any:
    for name in names:
        if isinstance(contact, dict) and name in contact:
            return contact[name]
        if hasattr(contact, name):
            return getattr(contact, name)
    return default


def parse_contacts(contacts: Any) -> list[dict[str, Any]]:
    """Copy MoveIt contact details while preserving penetration depth in meters."""
    parsed: list[dict[str, Any]] = []
    for index, contact in enumerate(list(contacts or [])):
        body_1 = _contact_value(contact, "contact_body_1", "body_1")
        body_2 = _contact_value(contact, "contact_body_2", "body_2")
        if not isinstance(body_1, str) or not isinstance(body_2, str):
            raise RuntimeError(f"MoveIt contact {index} has invalid body names")
        depth = _finite_number(
            _contact_value(contact, "depth", "depth_m"),
            f"MoveIt contact {index} depth",
        )
        parsed.append({"body_1": body_1, "body_2": body_2, "depth_m": depth})
    return parsed


def summarize_contacts(contacts: list[dict[str, Any]]) -> dict[str, Any]:
    """Deduplicate unordered body pairs and compute maximum penetration."""
    pairs: dict[tuple[str, str], dict[str, str]] = {}
    for contact in contacts:
        pair_key = tuple(sorted((contact["body_1"], contact["body_2"])))
        if pair_key not in pairs:
            pairs[pair_key] = {"body_1": pair_key[0], "body_2": pair_key[1]}
    normalized_pairs = list(pairs.values())
    return {
        "collision_pair_count": len(normalized_pairs),
        "collision_pairs": normalized_pairs,
        "first_collision_pair": dict(normalized_pairs[0]) if normalized_pairs else None,
        "max_penetration_depth_m": max(
            (contact["depth_m"] for contact in contacts),
            default=None,
        ),
    }


def parse_point_response(
    response: Any,
    point_index: int,
    time_from_start_s: float,
) -> dict[str, Any]:
    """Convert one GetStateValidity response into JSON-safe diagnostics."""
    valid = bool(getattr(response, "valid", False))
    contacts = parse_contacts(getattr(response, "contacts", []))
    collision = not valid and bool(contacts)
    result = {
        "point_index": point_index,
        "time_from_start_s": time_from_start_s,
        "valid": valid,
        "collision": collision,
        "contacts": contacts,
        "reason": None,
    }
    if not valid and not contacts:
        result["reason"] = "STATE INVALID — NO COLLISION CONTACT RETURNED"
    return result


def aggregate_point_results(points: list[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate ordered point results without assuming contact ordering."""
    failed = [point for point in points if not point["valid"]]
    contacts = [contact for point in points for contact in point["contacts"]]
    contact_summary = summarize_contacts(contacts)
    all_valid = not failed
    if all_valid:
        collision_free: bool | None = True
    elif contacts:
        collision_free = False
    else:
        collision_free = None
    return {
        "checked_point_count": len(points),
        "all_states_valid": all_valid,
        "collision_free": collision_free,
        "failed_point_count": len(failed),
        "first_failed_point": dict(failed[0]) if failed else None,
        **contact_summary,
    }


def empty_validation_result(
    status: str,
    error: str | None = None,
    *,
    service_name: str = SERVICE_NAME,
    group_name: str = PLANNING_GROUP,
) -> dict[str, Any]:
    """Build a stable response for unavailable, timeout, or error states."""
    return {
        "source": "moveit_check_state_validity",
        "service": service_name,
        "group": group_name,
        "status": status,
        "checked_point_count": 0,
        "all_states_valid": None,
        "collision_free": None,
        "failed_point_count": 0,
        "first_failed_point": None,
        "points": [],
        "collision_pair_count": 0,
        "collision_pairs": [],
        "first_collision_pair": None,
        "max_penetration_depth_m": None,
        "error": error,
    }


class MoveItStateValidationBridge:
    """Sequential, bounded, single-flight GetStateValidity bridge."""

    def __init__(
        self,
        client: Any,
        request_factory: Callable[[], Any],
        robot_state_factory: Callable[[], Any],
        joint_state_factory: Callable[[], Any],
        *,
        service_name: str = SERVICE_NAME,
        group_name: str = PLANNING_GROUP,
        timeout_s: float = 2.0,
    ) -> None:
        self.client = client
        self.request_factory = request_factory
        self.robot_state_factory = robot_state_factory
        self.joint_state_factory = joint_state_factory
        self.service_name = service_name
        self.group_name = group_name
        self.timeout_s = float(timeout_s)
        self._validation_lock = threading.Lock()

    @classmethod
    def from_node(
        cls,
        node: Any,
        *,
        service_name: str = SERVICE_NAME,
        group_name: str = PLANNING_GROUP,
        timeout_s: float = 2.0,
    ) -> "MoveItStateValidationBridge":
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        client = node.create_client(GetStateValidity, service_name)
        return cls(
            client,
            GetStateValidity.Request,
            RobotState,
            JointState,
            service_name=service_name,
            group_name=group_name,
            timeout_s=timeout_s,
        )

    def _request_for_point(self, point: dict[str, Any]) -> Any:
        request = self.request_factory()
        robot_state = self.robot_state_factory()
        joint_state = self.joint_state_factory()
        joint_state.name = list(EXPECTED_JOINTS)
        joint_state.position = joint_positions(point)
        robot_state.joint_state = joint_state
        request.robot_state = robot_state
        request.group_name = self.group_name
        return request

    def _wait_for_future(self, future: Any) -> tuple[str, Any]:
        deadline = time.monotonic() + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            cancel = getattr(future, "cancel", None)
            if callable(cancel):
                cancel()
            return "TIMEOUT", None
        try:
            response = future.result()
        except Exception as error:  # the ROS future forwards service exceptions
            return "ERROR", str(error)
        if response is None:
            return "ERROR", "MoveIt returned no response"
        return "OK", response

    def _base_result(self, status: str, error: str | None = None) -> dict[str, Any]:
        return empty_validation_result(
            status,
            error,
            service_name=self.service_name,
            group_name=self.group_name,
        )

    @staticmethod
    def _sampled_base_result(
        plan: dict[str, Any],
        status: str,
        error: str | None = None,
    ) -> dict[str, Any]:
        segments = [
            {
                **segment,
                "checked_interior_sample_count": 0,
                "failed_interior_sample_count": 0,
            }
            for segment in plan["segments"]
        ]
        return {
            "status": status,
            "max_joint_step_rad": plan["max_joint_step_rad"],
            "segment_count": plan["segment_count"],
            "generated_interior_sample_count": plan[
                "generated_interior_sample_count"
            ],
            "checked_interior_sample_count": 0,
            "failed_interior_sample_count": 0,
            "first_failed_sample": None,
            "collision_pair_count": 0,
            "collision_pairs": [],
            "first_collision_pair": None,
            "max_penetration_depth_m": None,
            "segments": segments,
            "samples": [],
            "error": error,
        }

    def _validate_sampled_path(self, plan: dict[str, Any]) -> dict[str, Any]:
        result = self._sampled_base_result(plan, "CHECKING")
        sample_results: list[dict[str, Any]] = []
        for sample in plan["samples"]:
            try:
                request = self._request_for_point(sample)
                wait_status, response = self._wait_for_future(
                    self.client.call_async(request)
                )
            except Exception as error:
                result["status"] = "ERROR"
                result["error"] = str(error)
                break
            if wait_status != "OK":
                if wait_status == "ERROR":
                    error = str(response)
                else:
                    error = (
                        "MoveIt sampled-path request timed out at segment "
                        f"{sample['segment_index']}, sample {sample['sample_index']} "
                        f"after {self.timeout_s:.3f} seconds"
                    )
                result["status"] = wait_status
                result["error"] = error
                break

            try:
                parsed = parse_point_response(
                    response,
                    sample["sample_index"],
                    sample["time_from_start_s"],
                )
            except Exception as error:
                result["status"] = "ERROR"
                result["error"] = str(error)
                break
            parsed.pop("point_index", None)
            sample_result = {
                key: sample[key]
                for key in (
                    "segment_index",
                    "start_point_index",
                    "end_point_index",
                    "sample_index",
                    "subdivision_count",
                    "alpha",
                    "time_from_start_s",
                    "sample_time_from_start_s",
                )
            }
            sample_result.update(parsed)
            sample_results.append(sample_result)
            segment = result["segments"][sample["segment_index"]]
            segment["checked_interior_sample_count"] += 1
            if not sample_result["valid"]:
                segment["failed_interior_sample_count"] += 1

        failed = [sample for sample in sample_results if not sample["valid"]]
        contacts = [
            contact
            for sample in sample_results
            for contact in sample["contacts"]
        ]
        result.update(summarize_contacts(contacts))
        result["samples"] = sample_results
        result["checked_interior_sample_count"] = len(sample_results)
        result["failed_interior_sample_count"] = len(failed)
        result["first_failed_sample"] = dict(failed[0]) if failed else None
        if result["status"] == "CHECKING":
            result["status"] = "PASS" if not failed else "FAIL"
            result["error"] = None
        return result

    def validate_trajectory(
        self,
        trajectory: Any,
        sampled_path: Any = None,
    ) -> dict[str, Any]:
        normalized = normalize_trajectory(trajectory)
        sampled_options = normalize_sampled_path_options(sampled_path)
        sampled_plan = None
        if sampled_options["enabled"]:
            sampled_plan = generate_trajectory_interior_samples(
                normalized,
                sampled_options["max_joint_step_rad"],
            )

        def with_sampled_skip(result: dict[str, Any], reason: str) -> dict[str, Any]:
            if sampled_plan is not None:
                result["sampled_path"] = self._sampled_base_result(
                    sampled_plan, "SKIPPED", reason
                )
            return result

        if not self._validation_lock.acquire(blocking=False):
            return with_sampled_skip(
                self._base_result("ERROR", "MoveIt validation is already running"),
                "Stored-point validation did not pass",
            )
        try:
            if not self.client.wait_for_service(timeout_sec=min(self.timeout_s, 0.25)):
                return with_sampled_skip(
                    self._base_result(
                        "UNAVAILABLE",
                        f"MoveIt service unavailable: {self.service_name}",
                    ),
                    "Stored-point MoveIt service is unavailable",
                )

            point_results: list[dict[str, Any]] = []
            for point_index, point in enumerate(normalized["points"]):
                request = self._request_for_point(point)
                wait_status, response = self._wait_for_future(
                    self.client.call_async(request)
                )
                if wait_status != "OK":
                    error = str(response) if wait_status == "ERROR" else (
                        f"MoveIt state-validity request timed out at point "
                        f"{point_index} after {self.timeout_s:.3f} seconds"
                    )
                    result = self._base_result(wait_status, error)
                    result["checked_point_count"] = len(point_results)
                    result["points"] = [dict(item) for item in point_results]
                    return with_sampled_skip(
                        result,
                        f"Stored-point validation ended with {wait_status}",
                    )
                point_results.append(
                    parse_point_response(
                        response,
                        point_index,
                        point["time_from_start_s"],
                    )
                )

            summary = aggregate_point_results(point_results)
            result = self._base_result("PASS" if summary["all_states_valid"] else "FAIL")
            result.update(summary)
            result["points"] = point_results
            result["error"] = None
            if sampled_plan is not None:
                if summary["all_states_valid"]:
                    result["sampled_path"] = self._validate_sampled_path(sampled_plan)
                else:
                    result["sampled_path"] = self._sampled_base_result(
                        sampled_plan,
                        "SKIPPED",
                        "Stored-point MoveIt validation failed",
                    )
            return result
        except Exception as error:
            return with_sampled_skip(
                self._base_result("ERROR", str(error)),
                "Stored-point validation did not pass",
            )
        finally:
            self._validation_lock.release()
