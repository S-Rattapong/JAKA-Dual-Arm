"""Reversible MoveIt Planning Scene adapter for Phase-4B validation.

The adapter is hosted by the existing backend node/executor. It creates only
Planning Scene service clients and delegates state checking to the existing
MoveItStateValidationBridge. Validation-owned IDs are rejected if preexisting,
and all inserted objects plus the temporary collision matrix are restored.
"""

from __future__ import annotations

from copy import deepcopy
import math
import threading
import time
from typing import Any, Mapping, Sequence

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory_ik import rotation_matrix_to_quaternion_xyzw


GET_PLANNING_SCENE_SERVICE = "/get_planning_scene"
APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
WORLD_FRAME = "world"
VALIDATION_ID_PREFIX = "phase4b_"


class MoveItPlanningSceneValidationAdapter:
    """Controlled, single-flight scene mutation around existing state checks."""

    def __init__(
        self,
        get_scene_client: Any,
        apply_scene_client: Any,
        state_validation_bridge: Any,
        call_and_wait: Any,
        ros_types: Mapping[str, Any],
        *,
        timeout_s: float = 2.0,
    ) -> None:
        self.get_scene_client = get_scene_client
        self.apply_scene_client = apply_scene_client
        self.state_validation_bridge = state_validation_bridge
        self.call_and_wait = call_and_wait
        self.ros_types = dict(ros_types)
        self.timeout_s = float(timeout_s)
        self._lock = threading.Lock()

    @classmethod
    def from_node(
        cls,
        node: Any,
        state_validation_bridge: Any,
        *,
        timeout_s: float = 2.0,
    ) -> "MoveItPlanningSceneValidationAdapter":
        from moveit_msgs.msg import (
            AllowedCollisionEntry,
            CollisionObject,
            PlanningScene,
            PlanningSceneComponents,
        )
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        from shape_msgs.msg import SolidPrimitive

        get_client = node.create_client(GetPlanningScene, GET_PLANNING_SCENE_SERVICE)
        apply_client = node.create_client(ApplyPlanningScene, APPLY_PLANNING_SCENE_SERVICE)

        def wait_on_existing_executor(client: Any, request: Any, wait_s: float) -> Any:
            future = client.call_async(request)
            deadline = time.monotonic() + wait_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.005)
            if not future.done():
                cancel = getattr(future, "cancel", None)
                if callable(cancel):
                    cancel()
                raise TimeoutError("MoveIt Planning Scene service response timed out")
            exception = future.exception()
            if exception is not None:
                raise RuntimeError(f"MoveIt Planning Scene service failed: {exception}")
            response = future.result()
            if response is None:
                raise RuntimeError("MoveIt Planning Scene service returned no response")
            return response

        return cls(
            get_client,
            apply_client,
            state_validation_bridge,
            wait_on_existing_executor,
            {
                "AllowedCollisionEntry": AllowedCollisionEntry,
                "CollisionObject": CollisionObject,
                "PlanningScene": PlanningScene,
                "PlanningSceneComponents": PlanningSceneComponents,
                "ApplyRequest": ApplyPlanningScene.Request,
                "GetRequest": GetPlanningScene.Request,
                "SolidPrimitive": SolidPrimitive,
            },
            timeout_s=timeout_s,
        )

    @staticmethod
    def _ready(client: Any) -> bool:
        ready = getattr(client, "service_is_ready", None)
        return bool(ready()) if callable(ready) else bool(
            client.wait_for_service(timeout_sec=0.0)
        )

    def unavailable_services(self) -> tuple[str, ...]:
        return tuple(
            name for name, client in (
                (GET_PLANNING_SCENE_SERVICE, self.get_scene_client),
                (APPLY_PLANNING_SCENE_SERVICE, self.apply_scene_client),
            ) if not self._ready(client)
        )

    def _get_scene(self) -> Any:
        request = self.ros_types["GetRequest"]()
        components = self.ros_types["PlanningSceneComponents"]
        request.components.components = (
            components.WORLD_OBJECT_NAMES | components.ALLOWED_COLLISION_MATRIX
        )
        return self.call_and_wait(
            self.get_scene_client, request, self.timeout_s
        ).scene

    def _apply(self, scene: Any) -> None:
        request = self.ros_types["ApplyRequest"]()
        request.scene = scene
        response = self.call_and_wait(self.apply_scene_client, request, self.timeout_s)
        if getattr(response, "success", False) is not True:
            raise RuntimeError("MoveIt rejected the Planning Scene diff")

    @staticmethod
    def _pose_from_matrix(matrix: Any, pose: Any) -> None:
        transform = RigidTransform.from_matrix(matrix)
        quaternion = rotation_matrix_to_quaternion_xyzw(transform)
        pose.position.x, pose.position.y, pose.position.z = transform.translation_m
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

    def _collision_object(self, config: Mapping[str, Any], pose_matrix: Any) -> Any:
        if config.get("shape") != "BOX":
            raise ValueError("Phase-4B currently supports configured BOX geometry only")
        dimensions = config.get("dimensions_m")
        if not isinstance(dimensions, Sequence) or len(dimensions) != 3:
            raise ValueError("configured BOX dimensions must contain three values")
        dimensions = [float(value) for value in dimensions]
        if any(not math.isfinite(value) or value <= 0.0 for value in dimensions):
            raise ValueError("configured BOX dimensions must be finite and positive")
        collision = self.ros_types["CollisionObject"]()
        collision.header.frame_id = WORLD_FRAME
        collision.id = str(config["id"])
        primitive = self.ros_types["SolidPrimitive"]()
        primitive.type = primitive.BOX
        primitive.dimensions = dimensions
        pose = collision.pose.__class__() if hasattr(collision, "pose") else None
        if pose is None:
            from geometry_msgs.msg import Pose
            pose = Pose()
        self._pose_from_matrix(pose_matrix, pose)
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = collision.ADD
        return collision

    def _scene_diff(self, collision_objects: Sequence[Any] = ()) -> Any:
        scene = self.ros_types["PlanningScene"]()
        scene.is_diff = True
        scene.world.collision_objects = list(collision_objects)
        return scene

    def _remove_scene(self, ids: Sequence[str], original_matrix: Any) -> None:
        objects = []
        for identifier in ids:
            collision = self.ros_types["CollisionObject"]()
            collision.header.frame_id = WORLD_FRAME
            collision.id = identifier
            collision.operation = collision.REMOVE
            objects.append(collision)
        scene = self._scene_diff(objects)
        scene.allowed_collision_matrix = deepcopy(original_matrix)
        self._apply(scene)

    def _allowed_matrix(self, original: Any, object_scene: Mapping[str, Any] | None) -> Any:
        matrix = deepcopy(original)
        if object_scene is None:
            return matrix
        object_id = str(object_scene["id"])
        allowed_links = tuple(object_scene.get("allowed_grasp_contact_links", ()))
        names = list(matrix.entry_names)
        enabled = [list(entry.enabled) for entry in matrix.entry_values]
        for name in (object_id, *allowed_links):
            if name not in names:
                names.append(name)
                for row in enabled:
                    row.append(False)
                enabled.append([False] * len(names))
        object_index = names.index(object_id)
        for link in allowed_links:
            link_index = names.index(link)
            enabled[object_index][link_index] = True
            enabled[link_index][object_index] = True
        entry_type = self.ros_types["AllowedCollisionEntry"]
        matrix.entry_names = names
        matrix.entry_values = []
        for row in enabled:
            entry = entry_type()
            entry.enabled = row
            matrix.entry_values.append(entry)
        return matrix

    def validate_scene_samples(
        self,
        samples: Sequence[Mapping[str, Any]],
        *,
        object_scene: Mapping[str, Any] | None,
        environment_scene: Sequence[Mapping[str, Any]],
    ) -> dict[str, Any]:
        if not self._lock.acquire(blocking=False):
            return {"status": "ERROR", "samples": [], "error": "Planning Scene validation is already running"}
        inserted_ids = tuple(
            ([str(object_scene["id"])] if object_scene else [])
            + [str(item["id"]) for item in environment_scene]
        )
        if any(not identifier.startswith(VALIDATION_ID_PREFIX) for identifier in inserted_ids):
            self._lock.release()
            raise ValueError(f"validation scene IDs must start with {VALIDATION_ID_PREFIX}")
        original_matrix = None
        inserted = False
        results = []
        outcome = {"status": "ERROR", "samples": results, "error": None}
        try:
            unavailable = self.unavailable_services()
            if unavailable:
                outcome.update(
                    status="UNAVAILABLE",
                    error="MoveIt Planning Scene service(s) unavailable: "
                    + ", ".join(unavailable),
                )
                return outcome
            original_scene = self._get_scene()
            existing = {item.id for item in original_scene.world.collision_objects}
            conflict = sorted(existing.intersection(inserted_ids))
            if conflict:
                outcome["error"] = (
                    "Validation scene ID already exists: " + ", ".join(conflict)
                )
                return outcome
            original_matrix = deepcopy(original_scene.allowed_collision_matrix)
            environments = [
                self._collision_object(item, item["pose_matrix"])
                for item in environment_scene
            ]
            base_scene = self._scene_diff(environments)
            base_scene.allowed_collision_matrix = self._allowed_matrix(
                original_matrix, object_scene
            )
            self._apply(base_scene)
            inserted = True
            for sample in samples:
                if object_scene is not None:
                    scene = self._scene_diff([
                        self._collision_object(
                            object_scene, sample["object_pose_matrix"]
                        )
                    ])
                    self._apply(scene)
                validation = self.state_validation_bridge.validate_trajectory({
                    "name": "Phase4B_Planning_Scene_Sample",
                    "points": [{
                        "time_from_start_s": float(sample["time_from_start_s"]),
                        "left": list(sample["left"]),
                        "right": list(sample["right"]),
                    }],
                })
                point = (validation.get("points") or [{}])[0]
                results.append({**deepcopy(dict(sample)), **deepcopy(point)})
                if validation.get("status") not in {"PASS", "FAIL"}:
                    outcome.update(
                        status=validation.get("status", "ERROR"),
                        error=validation.get("error"),
                    )
                    return outcome
            outcome.update(
                status="PASS" if all(item.get("valid") for item in results) else "FAIL",
                error=None,
            )
            return outcome
        except TimeoutError as error:
            outcome.update(status="TIMEOUT", error=str(error))
            return outcome
        except Exception as error:
            outcome.update(status="ERROR", error=str(error))
            return outcome
        finally:
            if inserted and original_matrix is not None:
                try:
                    self._remove_scene(inserted_ids, original_matrix)
                except Exception as cleanup_error:
                    outcome["status"] = "ERROR"
                    outcome["error"] = (
                        f"Planning Scene cleanup failed: {cleanup_error}"
                    )
            self._lock.release()
