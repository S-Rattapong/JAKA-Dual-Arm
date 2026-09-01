"""Pure Phase-4B collision classification and scene-sample orchestration.

OFFLINE MOVEIT VALIDATION ONLY. This module reuses results produced by the
existing state-validity bridge and the existing sampled-path generator. It has
no ROS, driver, publisher, action, controller, or robot execution dependency.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence
import xml.etree.ElementTree as ET

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory import _slerp_rotation
from dual_arm_app.backend.sampled_path_validation import (
    DEFAULT_MAX_JOINT_STEP_RAD,
    generate_trajectory_interior_samples,
)


SELF_LEFT = "SELF_LEFT"
SELF_RIGHT = "SELF_RIGHT"
INTER_ARM = "INTER_ARM"
OBJECT_ROBOT = "OBJECT_ROBOT"
ENVIRONMENT_ROBOT = "ENVIRONMENT_ROBOT"
OBJECT_ENVIRONMENT = "OBJECT_ENVIRONMENT"
OTHER_UNKNOWN = "OTHER / UNKNOWN"
STATE_INVALID_NO_CONTACT = "STATE_INVALID_NO_CONTACT"
COLLISION_CATEGORIES = (
    SELF_LEFT,
    SELF_RIGHT,
    INTER_ARM,
    OBJECT_ROBOT,
    ENVIRONMENT_ROBOT,
    OBJECT_ENVIRONMENT,
    OTHER_UNKNOWN,
)
OFFLINE_FIXTURE_LABEL = "OFFLINE TEST FIXTURE — NOT PHYSICAL CELL GEOMETRY"
OBJECT_GEOMETRY_AUDIT_SOURCE = (
    "digital_twin_object_grasp_preview.js:SYNTHETIC_OBJECT_DIMENSIONS_M — "
    "VISUALIZATION-ONLY SYNTHETIC METADATA; NOT COLLISION GEOMETRY"
)
ENVIRONMENT_GEOMETRY_AUDIT_SOURCE = (
    "REPOSITORY AUDIT — NO CANONICAL PHYSICAL CELL COLLISION GEOMETRY"
)
ROBOT_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "web/assets/dual_jaka_a12_web.urdf"
)


class Phase4CollisionInputError(ValueError):
    """Raised for malformed Phase-3 or collision-scene validation input."""


class SceneSampleValidator(Protocol):
    def validate_scene_samples(
        self,
        samples: Sequence[Mapping[str, Any]],
        *,
        object_scene: Mapping[str, Any] | None,
        environment_scene: Sequence[Mapping[str, Any]],
    ) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class CollisionIdentityRegistry:
    left_robot_links: frozenset[str]
    right_robot_links: frozenset[str]
    object_ids: frozenset[str] = frozenset()
    environment_ids: frozenset[str] = frozenset()
    allowed_grasp_contact_links: frozenset[str] = frozenset()
    identity_source: str = "COMBINED_ROBOT_URDF_AND_EXPLICIT_SCENE_IDS"

    def __post_init__(self) -> None:
        if not self.left_robot_links or not self.right_robot_links:
            raise ValueError("both canonical robot-link identity sets are required")
        if self.left_robot_links & self.right_robot_links:
            raise ValueError("left and right robot-link identities must be disjoint")
        robot_links = self.left_robot_links | self.right_robot_links
        if not self.allowed_grasp_contact_links <= robot_links:
            raise ValueError("allowed grasp contacts must name canonical robot links")
        scene_ids = self.object_ids | self.environment_ids
        if scene_ids & robot_links or self.object_ids & self.environment_ids:
            raise ValueError("robot, Object, and environment identities must be disjoint")


def load_robot_collision_identities(
    path: Path = ROBOT_MODEL_PATH,
) -> tuple[frozenset[str], frozenset[str]]:
    """Load explicit left/right link identities from the combined robot URDF."""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"Unable to load combined robot identities: {error}") from error
    names = {
        element.attrib.get("name")
        for element in root.findall("link")
        if isinstance(element.attrib.get("name"), str)
    }
    left = frozenset(name for name in names if name.startswith("left_"))
    right = frozenset(name for name in names if name.startswith("right_"))
    if not left or not right:
        raise RuntimeError("Combined robot URDF has incomplete arm link identities")
    return left, right


def canonical_collision_registry(
    *,
    object_ids: Sequence[str] = (),
    environment_ids: Sequence[str] = (),
    allowed_grasp_contact_links: Sequence[str] = (),
) -> CollisionIdentityRegistry:
    left, right = load_robot_collision_identities()
    return CollisionIdentityRegistry(
        left_robot_links=left,
        right_robot_links=right,
        object_ids=frozenset(object_ids),
        environment_ids=frozenset(environment_ids),
        allowed_grasp_contact_links=frozenset(allowed_grasp_contact_links),
    )


def classify_collision_pair(
    body_1: str,
    body_2: str,
    registry: CollisionIdentityRegistry,
) -> dict[str, Any]:
    """Classify unordered explicit identities without relying on contact order."""
    if not isinstance(body_1, str) or not isinstance(body_2, str):
        raise Phase4CollisionInputError("collision bodies must be strings")
    if not isinstance(registry, CollisionIdentityRegistry):
        raise TypeError("registry must be a CollisionIdentityRegistry")

    def identity(body: str) -> str:
        if body in registry.left_robot_links:
            return "LEFT_ROBOT"
        if body in registry.right_robot_links:
            return "RIGHT_ROBOT"
        if body in registry.object_ids:
            return "OBJECT"
        if body in registry.environment_ids:
            return "ENVIRONMENT"
        return "UNKNOWN"

    identities = frozenset((identity(body_1), identity(body_2)))
    if identities == {"LEFT_ROBOT"}:
        category = SELF_LEFT
    elif identities == {"RIGHT_ROBOT"}:
        category = SELF_RIGHT
    elif identities == {"LEFT_ROBOT", "RIGHT_ROBOT"}:
        category = INTER_ARM
    elif "OBJECT" in identities and identities & {"LEFT_ROBOT", "RIGHT_ROBOT"}:
        category = OBJECT_ROBOT
    elif "ENVIRONMENT" in identities and identities & {"LEFT_ROBOT", "RIGHT_ROBOT"}:
        category = ENVIRONMENT_ROBOT
    elif identities == {"OBJECT", "ENVIRONMENT"}:
        category = OBJECT_ENVIRONMENT
    else:
        category = OTHER_UNKNOWN
    robot_body = body_1 if identity(body_1) in {"LEFT_ROBOT", "RIGHT_ROBOT"} else body_2
    object_in_pair = "OBJECT" in identities
    allowed = bool(
        category == OBJECT_ROBOT
        and object_in_pair
        and robot_body in registry.allowed_grasp_contact_links
    )
    return {
        "body_1": body_1,
        "body_2": body_2,
        "body_1_identity": identity(body_1),
        "body_2_identity": identity(body_2),
        "category": category,
        "allowed_grasp_contact": allowed,
    }


def _classified_points(
    points: Sequence[Mapping[str, Any]],
    registry: CollisionIdentityRegistry,
    location_kind: str,
) -> list[dict[str, Any]]:
    output = []
    for point in points:
        copied = deepcopy(dict(point))
        contacts = []
        for contact in point.get("contacts", ()):
            classified = classify_collision_pair(
                contact.get("body_1"), contact.get("body_2"), registry
            )
            classified["depth_m"] = contact.get("depth_m")
            contacts.append(classified)
        copied["contacts"] = contacts
        copied["collision_categories"] = sorted({
            contact["category"]
            for contact in contacts
            if not contact["allowed_grasp_contact"]
        })
        copied["allowed_grasp_contacts"] = [
            contact for contact in contacts if contact["allowed_grasp_contact"]
        ]
        if point.get("valid") is False and not contacts:
            copied["state_classification"] = STATE_INVALID_NO_CONTACT
        else:
            copied["state_classification"] = (
                "COLLISION" if copied["collision_categories"] else "VALID_OR_ALLOWED_CONTACT"
            )
        copied["location_kind"] = location_kind
        output.append(copied)
    return output


def _category_summary(
    points: Sequence[Mapping[str, Any]],
    categories: frozenset[str],
    evaluated: bool,
) -> dict[str, Any]:
    matches = []
    for point in points:
        for contact in point.get("contacts", ()):
            if contact["category"] in categories and not contact["allowed_grasp_contact"]:
                matches.append({
                    "location_kind": point.get("location_kind"),
                    "point_index": point.get("point_index"),
                    "segment_index": point.get("segment_index"),
                    "sample_index": point.get("sample_index"),
                    "alpha": point.get("alpha"),
                    "time_from_start_s": point.get("time_from_start_s"),
                    **contact,
                })
    pairs = {}
    for match in matches:
        key = tuple(sorted((match["body_1"], match["body_2"])))
        pairs.setdefault(key, {"body_1": key[0], "body_2": key[1], "category": match["category"]})
    return {
        "status": "NOT_EVALUATED" if not evaluated else ("FAIL" if matches else "PASS"),
        "first_failure": deepcopy(matches[0]) if matches else None,
        "collision_pairs": list(pairs.values()),
        "collision_count": len(matches),
        "max_penetration_depth_m": max(
            (item["depth_m"] for item in matches if isinstance(item.get("depth_m"), (int, float))),
            default=None,
        ),
    }


def phase3_plan_to_robot_trajectory(plan: Mapping[str, Any]) -> dict[str, Any]:
    path = plan.get("combined_path") if isinstance(plan.get("combined_path"), list) else plan.get("global_path")
    if plan.get("ok") is not True or plan.get("planner_status") != "READY":
        raise Phase4CollisionInputError("plan must be a successful Phase-3 Global Plan")
    if not isinstance(path, list) or not path:
        raise Phase4CollisionInputError("selected motion path must be non-empty")
    points = []
    for index, point in enumerate(path):
        if not isinstance(point, Mapping):
            raise Phase4CollisionInputError(f"motion_path[{index}] must be an object")
        left, right = point.get("left"), point.get("right")
        if not isinstance(left, list) or len(left) != 6 or not isinstance(right, list) or len(right) != 6:
            raise Phase4CollisionInputError(f"motion_path[{index}] must contain Left/Right 6 joints")
        values = [*left, *right, point.get("time_from_start_s")]
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in values):
            raise Phase4CollisionInputError(f"motion_path[{index}] contains non-finite values")
        points.append({
            "time_from_start_s": float(point["time_from_start_s"]),
            "left": [float(value) for value in left],
            "right": [float(value) for value in right],
        })
    return {"name": str(plan.get("trajectory_name") or "Phase3_Global_Path"), "points": points}


def _object_transform(sample: Mapping[str, Any], label: str) -> RigidTransform:
    pose = sample.get("object_pose")
    matrix = pose.get("matrix") if isinstance(pose, Mapping) else None
    if matrix is None:
        raise Phase4CollisionInputError(f"{label}.object_pose.matrix is required")
    try:
        return RigidTransform.from_matrix(matrix)
    except (TypeError, ValueError) as error:
        raise Phase4CollisionInputError(f"{label}.object_pose.matrix: {error}") from error


def build_synchronized_scene_samples(
    plan: Mapping[str, Any],
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> dict[str, Any]:
    """Reuse joint subdivision and attach the matching interpolated Object pose."""
    trajectory = phase3_plan_to_robot_trajectory(plan)
    object_samples = (
        plan.get("combined_object_samples")
        if isinstance(plan.get("combined_object_samples"), list)
        else plan.get("object_samples")
    )
    if not isinstance(object_samples, list) or len(object_samples) != len(trajectory["points"]):
        raise Phase4CollisionInputError("Object and selected joint sample counts must match")
    transforms = []
    for index, sample in enumerate(object_samples):
        if not isinstance(sample, Mapping):
            raise Phase4CollisionInputError(f"object_samples[{index}] must be an object")
        sample_time = sample.get("time_from_start_s")
        if (
            isinstance(sample_time, bool)
            or not isinstance(sample_time, (int, float))
            or not math.isfinite(float(sample_time))
        ):
            raise Phase4CollisionInputError(
                f"object_samples[{index}].time_from_start_s must be finite"
            )
        if float(sample_time) != trajectory["points"][index]["time_from_start_s"]:
            raise Phase4CollisionInputError("Object and joint timestamps must match exactly")
        transforms.append(_object_transform(sample, f"object_samples[{index}]"))
    sampled = generate_trajectory_interior_samples(trajectory, max_joint_step_rad)
    stored = []
    for index, point in enumerate(trajectory["points"]):
        stored.append({
            **deepcopy(point),
            "location_kind": "STORED_POINT",
            "point_index": index,
            "object_pose_matrix": [list(row) for row in transforms[index].matrix],
        })
    interior = []
    for sample in sampled["samples"]:
        segment = sample["segment_index"]
        start, end = transforms[segment], transforms[segment + 1]
        alpha = sample["alpha"]
        translation = tuple(
            (1.0 - alpha) * start.translation_m[axis] + alpha * end.translation_m[axis]
            for axis in range(3)
        )
        interpolated = _slerp_rotation(
            start, end, alpha, translation  # type: ignore[arg-type]
        )
        interior.append({
            **deepcopy(sample),
            "location_kind": "SAMPLED_PATH",
            "object_pose_matrix": [list(row) for row in interpolated.matrix],
        })
    return {
        "max_joint_step_rad": sampled["max_joint_step_rad"],
        "stored_samples": stored,
        "sampled_path": {**sampled, "samples": interior},
    }


def synthetic_object_collision_fixture() -> dict[str, Any]:
    """Explicit test-only geometry matching the synthetic Web visual fixture."""
    return {
        "id": "phase4b_offline_test_workpiece",
        "shape": "BOX",
        "dimensions_m": [0.50, 0.15, 0.10],
        "source": OFFLINE_FIXTURE_LABEL,
        "allowed_grasp_contact_links": ["left_J6", "right_J6"],
        "allowed_contact_policy": "EXPLICIT TEST-ONLY TOUCH LINKS",
    }


def synthetic_environment_collision_fixture() -> list[dict[str, Any]]:
    """Return one deterministic test-only box, never normal cell geometry."""
    return [{
        "id": "phase4b_offline_test_environment_box",
        "shape": "BOX",
        "dimensions_m": [0.20, 0.20, 0.20],
        "pose_matrix": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "source": OFFLINE_FIXTURE_LABEL,
    }]


def _scene_status(configured: bool, source: str, content: Any, key: str) -> dict[str, Any]:
    return {
        "status": "CONFIGURED" if configured else "NOT_CONFIGURED",
        key: deepcopy(content) if configured else ([] if key == "objects" else None),
        "source": source,
    }


def build_phase4b_collision_result(
    plan: Mapping[str, Any],
    robot_validation: Mapping[str, Any],
    *,
    scene_validator: SceneSampleValidator | None = None,
    object_geometry: Mapping[str, Any] | None = None,
    environment_objects: Sequence[Mapping[str, Any]] = (),
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> dict[str, Any]:
    """Build Phase-4B result without confusing an empty scene with PASS."""
    synchronized = build_synchronized_scene_samples(plan, max_joint_step_rad)
    object_configured = object_geometry is not None
    environment_configured = bool(environment_objects)
    object_ids = [str(object_geometry["id"])] if object_configured else []
    environment_ids = [str(item["id"]) for item in environment_objects]
    allowed_links = object_geometry.get("allowed_grasp_contact_links", ()) if object_configured else ()
    registry = canonical_collision_registry(
        object_ids=object_ids,
        environment_ids=environment_ids,
        allowed_grasp_contact_links=allowed_links,
    )

    stored = _classified_points(robot_validation.get("points", ()), registry, "STORED_POINT")
    sampled_raw = robot_validation.get("sampled_path") or {}
    sampled = _classified_points(sampled_raw.get("samples", ()), registry, "SAMPLED_PATH")
    robot_evaluated = robot_validation.get("status") in {"PASS", "FAIL"}
    sampled_evaluated = sampled_raw.get("status") in {"PASS", "FAIL"}

    scene_error = None
    scene_points: list[dict[str, Any]] = []
    if object_configured or environment_configured:
        if scene_validator is None:
            scene_error = "MoveIt Planning Scene validator is unavailable"
        else:
            try:
                scene_result = scene_validator.validate_scene_samples(
                    [*synchronized["stored_samples"], *synchronized["sampled_path"]["samples"]],
                    object_scene=object_geometry,
                    environment_scene=environment_objects,
                )
                scene_points = _classified_points(
                    scene_result.get("samples", ()), registry, "PLANNING_SCENE"
                )
                scene_error = scene_result.get("error")
            except Exception as error:
                scene_error = str(error)
    all_robot_points = [*stored, *sampled]
    all_scene_points = scene_points
    self_collision = _category_summary(
        all_robot_points, frozenset((SELF_LEFT, SELF_RIGHT)), robot_evaluated
    )
    inter_arm = _category_summary(
        all_robot_points, frozenset((INTER_ARM,)), robot_evaluated
    )
    object_collision = _category_summary(
        all_scene_points,
        frozenset((OBJECT_ROBOT, OBJECT_ENVIRONMENT)),
        object_configured and scene_validator is not None and scene_error is None,
    )
    environment_collision = _category_summary(
        all_scene_points,
        frozenset((ENVIRONMENT_ROBOT, OBJECT_ENVIRONMENT)),
        environment_configured and scene_validator is not None and scene_error is None,
    )
    all_points = [*all_robot_points, *all_scene_points]
    failures = []
    category_counts = {category: 0 for category in COLLISION_CATEGORIES}
    pairs = {}
    for point in all_points:
        for contact in point.get("contacts", ()):
            if contact["allowed_grasp_contact"]:
                continue
            category_counts[contact["category"]] += 1
            failure = {
                "location_kind": point.get("location_kind"),
                "point_index": point.get("point_index"),
                "segment_index": point.get("segment_index"),
                "sample_index": point.get("sample_index"),
                "alpha": point.get("alpha"),
                "time_from_start_s": point.get("time_from_start_s"),
                **contact,
            }
            failures.append(failure)
            key = tuple(sorted((contact["body_1"], contact["body_2"])))
            pairs.setdefault(key, {"body_1": key[0], "body_2": key[1], "category": contact["category"]})
    required_incomplete = not object_configured or not environment_configured
    robot_failed = (
        robot_validation.get("status") == "FAIL"
        or sampled_raw.get("status") == "FAIL"
    )
    status = "FAIL" if failures or robot_failed else (
        "INCOMPLETE" if required_incomplete or scene_error else "PASS"
    )
    return {
        "status": status,
        "plan_only": True,
        "execution_ready": None,
        "robot_collision": {
            "stored_points": {"status": robot_validation.get("status"), "points": stored},
            "sampled_path": {"status": sampled_raw.get("status", "NOT_EVALUATED"), "samples": sampled},
            "self_collision": self_collision,
            "inter_arm_collision": inter_arm,
        },
        "object_scene": {
            **_scene_status(object_configured, object_geometry.get("source") if object_configured else OBJECT_GEOMETRY_AUDIT_SOURCE, object_geometry, "geometry"),
            "allowed_grasp_contact_links": list(allowed_links),
            "allowed_contact_policy": object_geometry.get("allowed_contact_policy") if object_configured else "NOT_CONFIGURED",
            "physical_gripper_calibration_claim": False,
        },
        "object_collision": object_collision,
        "environment_scene": _scene_status(environment_configured, environment_objects[0].get("source") if environment_configured else ENVIRONMENT_GEOMETRY_AUDIT_SOURCE, environment_objects, "objects"),
        "environment_collision": environment_collision,
        "synchronized_sampling": synchronized,
        "collision_summary": {
            "first_failure": deepcopy(failures[0]) if failures else None,
            "collision_categories": category_counts,
            "collision_pairs": list(pairs.values()),
        },
        "scene_error": scene_error,
        "warnings": [
            "DISCRETE SAMPLED CHECK — NOT CONTINUOUS COLLISION GUARANTEE",
            "NOT EXECUTION READY",
        ] + (["OBJECT COLLISION GEOMETRY NOT CONFIGURED"] if not object_configured else [])
          + (["ENVIRONMENT COLLISION SCENE NOT CONFIGURED"] if not environment_configured else []),
    }
