from dataclasses import dataclass

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState

from jaka_a12_dual_planner.object_frame import transform_to_pose


@dataclass
class MoveItValidationResult:
    ok: bool
    error_code: int
    message: str
    response: object = None


def duration_msg(seconds):
    seconds = max(0.0, float(seconds))
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1e9)
    msg = DurationMsg()
    msg.sec = whole_seconds
    msg.nanosec = nanoseconds
    return msg


def pose_stamped_from_transform(T, frame_id, stamp):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose = transform_to_pose(T)
    return msg


def robot_state_from_joint_state(joint_state):
    state = RobotState()
    if joint_state is not None:
        state.joint_state = joint_state
    return state


def merge_joint_states(*joint_states):
    merged = JointState()
    positions = {}
    velocities = {}
    efforts = {}
    stamp = None
    frame_id = ""

    for joint_state in joint_states:
        if joint_state is None:
            continue
        if stamp is None:
            stamp = joint_state.header.stamp
            frame_id = joint_state.header.frame_id
        for index, name in enumerate(joint_state.name):
            if index < len(joint_state.position):
                positions[name] = joint_state.position[index]
            if index < len(joint_state.velocity):
                velocities[name] = joint_state.velocity[index]
            if index < len(joint_state.effort):
                efforts[name] = joint_state.effort[index]

    names = sorted(positions.keys())
    if stamp is not None:
        merged.header.stamp = stamp
    merged.header.frame_id = frame_id
    merged.name = names
    merged.position = [float(positions[name]) for name in names]
    if velocities:
        merged.velocity = [float(velocities.get(name, 0.0)) for name in names]
    if efforts:
        merged.effort = [float(efforts.get(name, 0.0)) for name in names]
    return merged


def moveit_error_success(error_code):
    return int(error_code.val) == int(MoveItErrorCodes.SUCCESS)


def moveit_error_text(error_code):
    value = int(error_code.val)
    known = {
        MoveItErrorCodes.SUCCESS: "SUCCESS",
        MoveItErrorCodes.FAILURE: "FAILURE",
        MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: (
            "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"
        ),
        MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
        MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: (
            "UNABLE_TO_AQUIRE_SENSOR_DATA"
        ),
        MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
        MoveItErrorCodes.PREEMPTED: "PREEMPTED",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
        MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: (
            "START_STATE_VIOLATES_PATH_CONSTRAINTS"
        ),
        MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: (
            "GOAL_VIOLATES_PATH_CONSTRAINTS"
        ),
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: (
            "GOAL_CONSTRAINTS_VIOLATED"
        ),
        MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
        MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
        MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
        MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
        MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
        MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
        MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: (
            "COLLISION_CHECKING_UNAVAILABLE"
        ),
        MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
        MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
        MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
    }
    return known.get(value, f"MoveItErrorCode({value})")


def joint_constraints_from_state(joint_state, joint_names):
    positions = dict(zip(joint_state.name, joint_state.position))
    constraints = Constraints()
    constraints.name = "ik_solution_joint_goal"
    for joint_name in joint_names:
        if joint_name not in positions:
            continue
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = float(positions[joint_name])
        constraint.tolerance_above = 1e-4
        constraint.tolerance_below = 1e-4
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)
    return constraints


class MoveItValidationClient:
    def __init__(
        self,
        node,
        compute_ik_service="/compute_ik",
        plan_service="/plan_kinematic_path",
        state_validity_service="/check_state_validity",
    ):
        self.node = node
        self.ik_client = node.create_client(GetPositionIK, compute_ik_service)
        self.plan_client = node.create_client(GetMotionPlan, plan_service)
        self.state_validity_client = node.create_client(
            GetStateValidity,
            state_validity_service,
        )

    def services_ready(self):
        return (
            self.ik_client.service_is_ready()
            and self.plan_client.service_is_ready()
        )

    def ik_ready(self):
        return self.ik_client.service_is_ready()

    def wait_for_ik_once(self, timeout_sec=0.0):
        return self.ik_client.wait_for_service(timeout_sec=timeout_sec)

    def wait_for_services_once(self, timeout_sec=0.0):
        ik_ready = self.ik_client.wait_for_service(timeout_sec=timeout_sec)
        plan_ready = self.plan_client.wait_for_service(timeout_sec=timeout_sec)
        return ik_ready and plan_ready

    def state_validity_ready(self):
        return self.state_validity_client.service_is_ready()

    def send_ik_request(
        self,
        group_name,
        ik_link_name,
        target_pose_stamped,
        seed_joint_state,
        timeout_sec,
        avoid_collisions=True,
    ):
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.pose_stamped = target_pose_stamped
        request.ik_request.robot_state = robot_state_from_joint_state(
            seed_joint_state
        )
        request.ik_request.avoid_collisions = bool(avoid_collisions)
        request.ik_request.timeout = duration_msg(timeout_sec)
        return self.ik_client.call_async(request)

    def parse_ik_response(self, response):
        ok = moveit_error_success(response.error_code)
        return MoveItValidationResult(
            ok=ok,
            error_code=int(response.error_code.val),
            message=moveit_error_text(response.error_code),
            response=response,
        )

    def send_motion_plan_request(
        self,
        group_name,
        start_joint_state,
        goal_joint_state,
        goal_joint_names,
        planning_time,
        plan_attempts,
    ):
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = group_name
        motion_request.num_planning_attempts = int(plan_attempts)
        motion_request.allowed_planning_time = float(planning_time)
        motion_request.start_state = robot_state_from_joint_state(start_joint_state)
        motion_request.goal_constraints.append(
            joint_constraints_from_state(goal_joint_state, goal_joint_names)
        )
        motion_request.max_velocity_scaling_factor = 0.2
        motion_request.max_acceleration_scaling_factor = 0.2
        return self.plan_client.call_async(request)

    def parse_plan_response(self, response):
        code = response.motion_plan_response.error_code
        ok = moveit_error_success(code)
        return MoveItValidationResult(
            ok=ok,
            error_code=int(code.val),
            message=moveit_error_text(code),
            response=response,
        )

    def send_state_validity_request(self, joint_state, group_name="dual_arm"):
        request = GetStateValidity.Request()
        request.robot_state = robot_state_from_joint_state(joint_state)
        request.group_name = group_name
        return self.state_validity_client.call_async(request)

    def parse_state_validity_response(self, response):
        ok = bool(response.valid)
        return MoveItValidationResult(
            ok=ok,
            error_code=(
                int(MoveItErrorCodes.SUCCESS)
                if ok
                else int(MoveItErrorCodes.FAILURE)
            ),
            message="VALID" if ok else "INVALID",
            response=response,
        )
