"""Reusable MoveIt /compute_fk adapter for Phase-4A planned validation.

MOVEIT MODEL VALIDATION ONLY. NO JAKA GetFK, driver, publisher, controller,
action, or motion command is created here.
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Sequence

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory_ik import (
    DUAL_ARM_JOINT_ORDER,
    LEFT_IK_LINK_NAME,
    RIGHT_IK_LINK_NAME,
)


COMPUTE_FK_SERVICE = "/compute_fk"
WORLD_FRAME = "world"
FK_LINK_NAMES = (LEFT_IK_LINK_NAME, RIGHT_IK_LINK_NAME)


def _quaternion_transform(pose: Any) -> RigidTransform:
    position = pose.position
    orientation = pose.orientation
    values = tuple(float(value) for value in (
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("FK pose contains non-finite values")
    x, y, z, qx, qy, qz, qw = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("FK pose quaternion has zero norm")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    return RigidTransform.from_matrix((
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw), x),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), y),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy), z),
        (0.0, 0.0, 0.0, 1.0),
    ))


class MoveItFkValidationAdapter:
    """One existing-node client; one request returns both planning tip poses."""

    def __init__(
        self,
        compute_fk_client: Any,
        clock: Any,
        call_and_wait: Callable[[Any, Any, float], Any],
        request_type: Any,
    ) -> None:
        self.compute_fk_client = compute_fk_client
        self.clock = clock
        self.call_and_wait = call_and_wait
        self.request_type = request_type

    @classmethod
    def from_node(cls, node: Any) -> "MoveItFkValidationAdapter":
        # Lazy ROS import keeps the pure Phase-4 core importable in offline tests.
        from moveit_msgs.srv import GetPositionFK

        client = node.create_client(GetPositionFK, COMPUTE_FK_SERVICE)

        def wait_on_existing_executor(client: Any, request: Any, timeout_s: float) -> Any:
            future = client.call_async(request)
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.005)
            if not future.done():
                cancel = getattr(future, "cancel", None)
                if callable(cancel):
                    cancel()
                raise TimeoutError("MoveIt /compute_fk response timed out")
            exception = future.exception()
            if exception is not None:
                raise RuntimeError(f"MoveIt /compute_fk failed: {exception}")
            response = future.result()
            if response is None:
                raise RuntimeError("MoveIt /compute_fk returned no response")
            return response

        return cls(client, node.get_clock(), wait_on_existing_executor, GetPositionFK.Request)

    def service_is_ready(self) -> bool:
        ready = getattr(self.compute_fk_client, "service_is_ready", None)
        if callable(ready):
            return bool(ready())
        return bool(self.compute_fk_client.wait_for_service(timeout_sec=0.0))

    def compute_combined_fk(
        self,
        joint_positions_rad: Sequence[float],
        timeout_s: float = 2.0,
    ) -> dict[str, Any]:
        if len(joint_positions_rad) != 12:
            raise ValueError("joint_positions_rad must contain exactly 12 values")
        values = tuple(float(value) for value in joint_positions_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("joint_positions_rad must contain finite values")
        if not self.service_is_ready():
            return {"status": "UNAVAILABLE", "error": "/compute_fk is unavailable"}
        request = self.request_type()
        request.header.frame_id = WORLD_FRAME
        request.header.stamp = self.clock.now().to_msg()
        request.fk_link_names = list(FK_LINK_NAMES)
        request.robot_state.joint_state.name = list(DUAL_ARM_JOINT_ORDER)
        request.robot_state.joint_state.position = list(values)
        request.robot_state.is_diff = False
        response = self.call_and_wait(self.compute_fk_client, request, timeout_s)
        error_code = getattr(getattr(response, "error_code", None), "val", 1)
        if error_code != 1:
            return {
                "status": "ERROR",
                "error": f"MoveIt /compute_fk error code: {error_code}",
            }
        names = list(getattr(response, "fk_link_names", ()))
        poses = list(getattr(response, "pose_stamped", ()))
        if len(names) != len(poses):
            return {"status": "ERROR", "error": "MoveIt FK link/pose counts differ"}
        by_name = {name: stamped.pose for name, stamped in zip(names, poses)}
        missing = [name for name in FK_LINK_NAMES if name not in by_name]
        if missing:
            return {"status": "ERROR", "error": "MoveIt FK response missing links: " + ", ".join(missing)}
        try:
            return {
                "status": "PASS",
                "left": _quaternion_transform(by_name[LEFT_IK_LINK_NAME]),
                "right": _quaternion_transform(by_name[RIGHT_IK_LINK_NAME]),
                "frame_id": WORLD_FRAME,
                "link_names": list(FK_LINK_NAMES),
            }
        except (AttributeError, TypeError, ValueError) as error:
            return {"status": "ERROR", "error": f"Malformed MoveIt FK pose: {error}"}
