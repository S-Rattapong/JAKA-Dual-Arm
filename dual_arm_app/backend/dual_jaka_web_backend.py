from typing import List, Literal, Optional
#!/usr/bin/env python3
import json
import re
import threading
import time
from pathlib import Path
from typing import Optional, Dict, Any, Literal

import yaml
import rclpy
from rclpy.node import Node

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

try:
    from dual_arm_app.backend.center_path_store import CenterPathStore, CenterPathStoreError
except ImportError:
    try:
        from .center_path_store import CenterPathStore, CenterPathStoreError
    except ImportError:
        from center_path_store import CenterPathStore, CenterPathStoreError

from std_srvs.srv import Empty, SetBool
from dual_arm_app.backend.operator_runtime import (
    OperatorRuntime, RuntimeRejected, DRIVERS, TERMINAL, control_robot, shutdown_decision,
)
from dual_arm_app.backend.operator_api import system_control_router
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import (
    Move,
    GetFK,
    GetIK,
    GetFrameState,
    ExecuteJointTrajectory,
    GetExecutionStatus,
)

try:
    from dual_arm_app.backend.joint_feedback import (
        build_digital_twin_joint_status,
        expected_joint_name_aliases,
        normalize_joint_state_message,
        wall_clock_ms,
    )
except ImportError:
    try:
        from .joint_feedback import (
            build_digital_twin_joint_status,
            expected_joint_name_aliases,
            normalize_joint_state_message,
            wall_clock_ms,
        )
    except ImportError:
        from joint_feedback import (
            build_digital_twin_joint_status,
            expected_joint_name_aliases,
            normalize_joint_state_message,
            wall_clock_ms,
        )

try:
    from dual_arm_app.backend.digital_twin_live_state import (
        build_digital_twin_robot_status,
        build_digital_twin_tcp_status,
        normalize_robot_state_message,
    )
except ImportError:
    try:
        from .digital_twin_live_state import (
            build_digital_twin_robot_status,
            build_digital_twin_tcp_status,
            normalize_robot_state_message,
        )
    except ImportError:
        from digital_twin_live_state import (
            build_digital_twin_robot_status,
            build_digital_twin_tcp_status,
            normalize_robot_state_message,
        )

try:
    from dual_arm_app.backend.port10000_actual_feedback import (
        DEFAULT_FRESHNESS_THRESHOLD_MS,
        DEFAULT_EXPECTED_PERIOD_MS,
        DEFAULT_MAX_PACKET_BYTES,
        DEFAULT_PORT,
        Port10000ActualFeedback,
        select_visualization_joint_status,
    )
except ImportError:
    try:
        from .port10000_actual_feedback import (
            DEFAULT_FRESHNESS_THRESHOLD_MS,
            DEFAULT_EXPECTED_PERIOD_MS,
            DEFAULT_MAX_PACKET_BYTES,
            DEFAULT_PORT,
            Port10000ActualFeedback,
            select_visualization_joint_status,
        )
    except ImportError:
        from port10000_actual_feedback import (
            DEFAULT_FRESHNESS_THRESHOLD_MS,
            DEFAULT_EXPECTED_PERIOD_MS,
            DEFAULT_MAX_PACKET_BYTES,
            DEFAULT_PORT,
            Port10000ActualFeedback,
            select_visualization_joint_status,
        )

try:
    from dual_arm_app.backend.sampled_path_validation import (
        SampledPathValidationInputError,
        normalize_sampled_path_options,
    )
except ImportError:
    try:
        from .sampled_path_validation import (
            SampledPathValidationInputError,
            normalize_sampled_path_options,
        )
    except ImportError:
        from sampled_path_validation import (
            SampledPathValidationInputError,
            normalize_sampled_path_options,
        )

try:
    from dual_arm_app.backend.moveit_state_validation import (
        MoveItStateValidationBridge,
        TrajectoryValidationInputError,
        empty_validation_result,
    )
except ImportError:
    try:
        from .moveit_state_validation import (
            MoveItStateValidationBridge,
            TrajectoryValidationInputError,
            empty_validation_result,
        )
    except ImportError:
        from moveit_state_validation import (
            MoveItStateValidationBridge,
            TrajectoryValidationInputError,
            empty_validation_result,
        )

try:
    from dual_arm_app.backend.object_global_planning import (
        ObjectGlobalPlanInputError,
        normalize_object_global_plan_request,
        plan_object_global,
        planning_authority_rejected_result,
        planning_unavailable_result,
    )
except ImportError:
    try:
        from .object_global_planning import (
            ObjectGlobalPlanInputError,
            normalize_object_global_plan_request,
            plan_object_global,
            planning_authority_rejected_result,
            planning_unavailable_result,
        )
    except ImportError:
        from object_global_planning import (
            ObjectGlobalPlanInputError,
            normalize_object_global_plan_request,
            plan_object_global,
            planning_authority_rejected_result,
            planning_unavailable_result,
        )

try:
    from dual_arm_app.backend.planning_start_state_config import (
        planning_start_state_payload,
    )
except ImportError:
    try:
        from .planning_start_state_config import planning_start_state_payload
    except ImportError:
        from planning_start_state_config import planning_start_state_payload

try:
    from dual_arm_app.backend.world_frame_calibration import (
        get_world_calibration_revision,
        get_web_model_calibration_revision,
        world_calibration_public_payload,
    )
except ImportError:
    try:
        from .world_frame_calibration import (
            get_world_calibration_revision,
            get_web_model_calibration_revision,
            world_calibration_public_payload,
        )
    except ImportError:
        from world_frame_calibration import (
            get_world_calibration_revision,
            get_web_model_calibration_revision,
            world_calibration_public_payload,
        )

try:
    from dual_arm_app.backend.rigid_grasp_configuration import (
        AuthoritativeRigidGraspState,
        GRASP_LOCKED,
        RigidGraspConfigurationError,
    )
except ImportError:
    try:
        from .rigid_grasp_configuration import (
            AuthoritativeRigidGraspState,
            GRASP_LOCKED,
            RigidGraspConfigurationError,
        )
    except ImportError:
        from rigid_grasp_configuration import (
            AuthoritativeRigidGraspState,
            GRASP_LOCKED,
            RigidGraspConfigurationError,
        )

try:
    from dual_arm_app.backend.phase4_trajectory_validation import (
        DEFAULT_ORIENTATION_TOLERANCE_RAD,
        DEFAULT_POSITION_TOLERANCE_M,
        Phase4ValidationInputError,
        validate_phase4_trajectory,
    )
except ImportError:
    try:
        from .phase4_trajectory_validation import (
            DEFAULT_ORIENTATION_TOLERANCE_RAD,
            DEFAULT_POSITION_TOLERANCE_M,
            Phase4ValidationInputError,
            validate_phase4_trajectory,
        )
    except ImportError:
        from phase4_trajectory_validation import (
            DEFAULT_ORIENTATION_TOLERANCE_RAD,
            DEFAULT_POSITION_TOLERANCE_M,
            Phase4ValidationInputError,
            validate_phase4_trajectory,
        )

try:
    from dual_arm_app.backend.phase4_collision_validation import (
        Phase4CollisionInputError,
        build_phase4b_collision_result,
        phase3_plan_to_robot_trajectory,
    )
except ImportError:
    try:
        from .phase4_collision_validation import (
            Phase4CollisionInputError,
            build_phase4b_collision_result,
            phase3_plan_to_robot_trajectory,
        )
    except ImportError:
        from phase4_collision_validation import (
            Phase4CollisionInputError,
            build_phase4b_collision_result,
            phase3_plan_to_robot_trajectory,
        )

try:
    from dual_arm_app.backend.phase4_unified_validation import (
        Phase4UnifiedValidationInputError,
        build_phase4_unified_report,
        evaluate_execution_gate,
    )
except ImportError:
    try:
        from .phase4_unified_validation import (
            Phase4UnifiedValidationInputError,
            build_phase4_unified_report,
            evaluate_execution_gate,
        )
    except ImportError:
        from phase4_unified_validation import (
            Phase4UnifiedValidationInputError,
            build_phase4_unified_report,
            evaluate_execution_gate,
        )

try:
    from dual_arm_app.backend.phase5_execution_artifact import (
        Phase5ExecutionArtifactError,
        freeze_validated_execution_artifact,
    )
except ImportError:
    try:
        from .phase5_execution_artifact import (
            Phase5ExecutionArtifactError,
            freeze_validated_execution_artifact,
        )
    except ImportError:
        from phase5_execution_artifact import (
            Phase5ExecutionArtifactError,
            freeze_validated_execution_artifact,
        )

try:
    from dual_arm_app.backend.phase5_execution_transport import (
        PHASE5_DRIVER_CONTRACT_MARKER,
        Phase5AbsoluteStartTransport,
        Phase5ExecutionTransportError,
    )
except ImportError:
    try:
        from .phase5_execution_transport import (
            PHASE5_DRIVER_CONTRACT_MARKER,
            Phase5AbsoluteStartTransport,
            Phase5ExecutionTransportError,
        )
    except ImportError:
        from phase5_execution_transport import (
            PHASE5_DRIVER_CONTRACT_MARKER,
            Phase5AbsoluteStartTransport,
            Phase5ExecutionTransportError,
        )

try:
    from dual_arm_app.backend.phase5_execution_coordinator import (
        ACTIVE_STATES as PHASE5_ACTIVE_STATES,
        Phase5ExecutionCoordinator,
    )
except ImportError:
    try:
        from .phase5_execution_coordinator import (
            ACTIVE_STATES as PHASE5_ACTIVE_STATES,
            Phase5ExecutionCoordinator,
        )
    except ImportError:
        from phase5_execution_coordinator import (
            ACTIVE_STATES as PHASE5_ACTIVE_STATES,
            Phase5ExecutionCoordinator,
        )

try:
    from dual_arm_app.backend.phase5_execution_feedback import (
        build_phase5_execution_feedback,
    )
except ImportError:
    try:
        from .phase5_execution_feedback import (
            build_phase5_execution_feedback,
        )
    except ImportError:
        from phase5_execution_feedback import (
            build_phase5_execution_feedback,
        )

try:
    from dual_arm_app.backend.phase6_joint_tracking import (
        compute_joint_tracking_error,
    )
except ImportError:
    try:
        from .phase6_joint_tracking import compute_joint_tracking_error
    except ImportError:
        from phase6_joint_tracking import compute_joint_tracking_error

try:
    from dual_arm_app.backend.phase5_motion_quality import (
        REPLAN_FROM_CURRENT_SOURCE,
        evaluate_actual_start_match,
        fresh_actual_joint_state,
        normalize_phase5_motion_settings,
    )
except ImportError:
    try:
        from .phase5_motion_quality import (
            REPLAN_FROM_CURRENT_SOURCE,
            evaluate_actual_start_match,
            fresh_actual_joint_state,
            normalize_phase5_motion_settings,
            )
    except ImportError:
        from phase5_motion_quality import (
            REPLAN_FROM_CURRENT_SOURCE,
            evaluate_actual_start_match,
            fresh_actual_joint_state,
            normalize_phase5_motion_settings,
            )

try:
    from dual_arm_app.backend.experimental_infrastructure import (
        ExperimentError,
        ExperimentRecorder,
        ExperimentRunStore,
        TorqueCache,
    )
except ImportError:
    try:
        from .experimental_infrastructure import (
            ExperimentError,
            ExperimentRecorder,
            ExperimentRunStore,
            TorqueCache,
        )
    except ImportError:
        from experimental_infrastructure import (
            ExperimentError,
            ExperimentRecorder,
            ExperimentRunStore,
            TorqueCache,
        )


COORD_MODE = {
    "joint": 0,
    "base": 1,
    "tool": 2,
}

AXIS_INDEX = {
    "j1": 0, "joint1": 0, "1": 0,
    "j2": 1, "joint2": 1, "2": 1,
    "j3": 2, "joint3": 2, "3": 2,
    "j4": 3, "joint4": 3, "4": 3,
    "j5": 4, "joint5": 4, "5": 4,
    "j6": 5, "joint6": 5, "6": 5,

    "x": 0,
    "y": 1,
    "z": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
}

class JogStartRequest(BaseModel):
    side: str
    coord: str
    axis: str
    direction: str
    speed: float
    sync_mode: str = "same"



class ProgramStepRequest(BaseModel):
    name: str
    side: Optional[str] = None
    vel: Optional[float] = None
    acc: Optional[float] = None
    delay: float = 0.0


class ProgramRunRequest(BaseModel):
    steps: list[ProgramStepRequest]
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None
    loop_mode: Literal["once", "count", "forever"] = "once"
    loop_count: int = Field(default=1, ge=1)


class ProgramSaveRequest(BaseModel):
    name: str
    steps: list[ProgramStepRequest]


class ProgramNameRequest(BaseModel):
    name: str


class DigitalTwinTrajectoryValidationRequest(BaseModel):
    trajectory: Dict[str, Any]
    sampled_path: Optional[Dict[str, Any]] = None


class DigitalTwinObjectGlobalPlanRequest(BaseModel):
    name: Any
    waypoints: Any
    approach_waypoints: Any = None
    fixed_orientation_rpy_rad: Any
    segment_duration_s: Any
    samples_per_segment: Any
    candidate_attempts_per_arm: Any = None
    initial_joint_state_rad: Any = None
    planning_start_state_source: Any = None
    expected_grasp_content_revision: Any = None
    expected_lock_generation: Any = None
    expected_lock_revision: Any = None
    expected_calibration_revision: Any = None
    expected_model_calibration_revision: Any = None


class DigitalTwinCenterPathSaveRequest(BaseModel):
    name: str
    path: Dict[str, Any]
    overwrite: bool = False


class DigitalTwinCenterPathNameRequest(BaseModel):
    name: str


class DigitalTwinCenterPathRenameRequest(BaseModel):
    name: str
    new_name: str


class DigitalTwinGraspLockRequest(BaseModel):
    left: Any
    right: Any


class DigitalTwinPhase4TrajectoryValidationRequest(BaseModel):
    plan: Any
    start_state: Any = None
    position_tolerance_m: Any = DEFAULT_POSITION_TOLERANCE_M
    orientation_tolerance_rad: Any = DEFAULT_ORIENTATION_TOLERANCE_RAD


class DigitalTwinPhase4CollisionValidationRequest(BaseModel):
    plan: Any
    max_joint_step_rad: Any = 0.05


class DigitalTwinPhase4UnifiedValidationRequest(BaseModel):
    plan: Any
    start_state: Any = None
    position_tolerance_m: Any = DEFAULT_POSITION_TOLERANCE_M
    orientation_tolerance_rad: Any = DEFAULT_ORIENTATION_TOLERANCE_RAD
    max_joint_step_rad: Any = 0.05


class DigitalTwinPhase5ExecuteRequest(BaseModel):
    operator_confirmed: bool = Field(..., strict=True)


class ExperimentArmRequest(BaseModel):
    label: str = ""
    path_type: Literal["LINEAR", "CURVED", "COMPLEX", "CUSTOM"] = "CUSTOM"
    fixture_condition: Literal["NONE", "CLEARANCE", "RIGID", "OTHER"] = "NONE"
    notes: str = ""
    plan_snapshot: Any = None


class ExperimentRelabelRequest(BaseModel):
    label: str


class StopRequest(BaseModel):
    side: str = "both"


class HomeRequest(BaseModel):
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None
    vel: Optional[float] = None
    acc: Optional[float] = None


class WaypointSaveRequest(BaseModel):
    name: str
    side: str = "both"


class WaypointRunRequest(BaseModel):
    name: str
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None


class SequenceRunRequest(BaseModel):
    names: list[str]
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None


class DualJakaWebNode(Node):
    def __init__(self, cfg):
        super().__init__("dual_jaka_web_backend")
        self.cfg = cfg
        try:
            self.phase5_motion_settings = normalize_phase5_motion_settings(
                cfg.get("motion", {})
            )
            self.phase5_motion_configuration_error = None
        except Exception as error:
            self.phase5_motion_settings = None
            self.phase5_motion_configuration_error = str(error)

        self.left_joint = None
        self.right_joint = None
        self.left_state = None
        self.right_state = None

        # Read-only Digital Twin feedback is kept separate from other robot
        # state so the endpoint never needs FK, services, or publishers.
        self.digital_twin_joint_cache_lock = threading.Lock()
        self.digital_twin_expected_joint_names = {
            side: expected_joint_name_aliases(side)
            for side in ("left", "right")
        }
        self.digital_twin_joint_cache = {
            side: {
                "joint": None,
                "received_at_ms": None,
                "mapping": None,
                "status": "MISSING",
                "error": "No valid JointState received",
                "names": [],
            }
            for side in ("left", "right")
        }
        # Independent visualization-only actual feedback. These daemon threads
        # connect to receive-only port 10000 and never use the JAKA SDK. Phase-5
        # start matching and safety continue to call the ROS-only cache method.
        port10000_cfg = cfg.get("actual_feedback", {}).get("port10000", {})
        self.port10000_freshness_threshold_ms = int(
            port10000_cfg.get(
                "freshness_threshold_ms", DEFAULT_FRESHNESS_THRESHOLD_MS
            )
        )
        self.port10000_expected_period_ms = int(
            port10000_cfg.get("expected_period_ms", DEFAULT_EXPECTED_PERIOD_MS)
        )
        self.port10000_actual_feedback = None
        if port10000_cfg.get("enabled", True):
            self.port10000_actual_feedback = Port10000ActualFeedback(
                {
                    "left": (
                        str(port10000_cfg.get("left_ip", "192.168.0.1")),
                        int(port10000_cfg.get("port", DEFAULT_PORT)),
                    ),
                    "right": (
                        str(port10000_cfg.get("right_ip", "192.168.0.2")),
                        int(port10000_cfg.get("port", DEFAULT_PORT)),
                    ),
                },
                max_packet_bytes=int(
                    port10000_cfg.get("max_packet_bytes", DEFAULT_MAX_PACKET_BYTES)
                ),
                connect_timeout_s=float(port10000_cfg.get("connect_timeout_s", 0.5)),
                recv_timeout_s=float(port10000_cfg.get("recv_timeout_s", 0.5)),
                reconnect_backoff_s=float(
                    port10000_cfg.get("reconnect_backoff_s", 1.0)
                ),
                stale_reconnect_timeout_s=float(
                    port10000_cfg.get("stale_reconnect_timeout_s", 2.0)
                ),
                log_warning=lambda message: self.get_logger().warning(message),
            )
            self.port10000_actual_feedback.start()
        self.digital_twin_robot_state_cache_lock = threading.Lock()
        self.digital_twin_robot_state_cache = {
            side: {
                "state": None,
                "received_at_ms": None,
                "valid": False,
                "status": "MISSING",
                "error": "No RobotMsg received",
            }
            for side in ("left", "right")
        }
        self.moveit_state_validation_bridge = None
        self.moveit_state_validation_error = None
        try:
            self.moveit_state_validation_bridge = MoveItStateValidationBridge.from_node(
                self,
                service_name="/check_state_validity",
                group_name="dual_arm",
                timeout_s=2.0,
            )
        except Exception as error:
            self.moveit_state_validation_error = str(error)
        self.object_global_planning_adapter = None
        self.object_global_planning_joint_limits = None
        self.object_global_planning_error = None
        self.object_global_planning_lock = threading.Lock()
        try:
            from dual_arm_app.backend.moveit_object_trajectory_ik import (
                MoveItObjectTrajectoryPlanningAdapter,
                load_canonical_joint_limits,
            )
            self.object_global_planning_adapter = (
                MoveItObjectTrajectoryPlanningAdapter.from_node(self)
            )
            self.object_global_planning_joint_limits = load_canonical_joint_limits()
        except Exception as error:
            self.object_global_planning_error = str(error)
        self.phase4_fk_validation_adapter = None
        self.phase4_fk_validation_error = None
        self.phase4_validation_lock = threading.Lock()
        try:
            from dual_arm_app.backend.moveit_fk_validation import (
                MoveItFkValidationAdapter,
            )
            self.phase4_fk_validation_adapter = MoveItFkValidationAdapter.from_node(self)
        except Exception as error:
            self.phase4_fk_validation_error = str(error)
        self.phase4_planning_scene_adapter = None
        self.phase4_planning_scene_error = None
        self.phase4_collision_validation_lock = threading.Lock()
        try:
            from dual_arm_app.backend.moveit_planning_scene_validation import (
                MoveItPlanningSceneValidationAdapter,
            )
            if self.moveit_state_validation_bridge is None:
                raise RuntimeError("Existing MoveIt state-validation bridge is unavailable")
            self.phase4_planning_scene_adapter = (
                MoveItPlanningSceneValidationAdapter.from_node(
                    self, self.moveit_state_validation_bridge, timeout_s=2.0
                )
            )
        except Exception as error:
            self.phase4_planning_scene_error = str(error)
        self.phase4_unified_validation_lock = threading.Lock()
        self.phase4_unified_validation_report = None
        self.phase4_unified_validation_running = False
        self.phase4_unified_validation_invalidated_reason = "NO REPORT"
        self.phase4_unified_validation_generation = 0
        self.phase5_execution_artifact_lock = threading.Lock()
        self.phase5_execution_artifact = None
        self.phase5_execution_artifact_invalidated_reason = "NO VALIDATED ARTIFACT"
        self.phase5_execution_artifact_generation = 0
        self.phase5_recovery_initial_lock = threading.Lock()
        self.phase5_recovery_initial = None
        self.rigid_grasp_configuration = AuthoritativeRigidGraspState()
        self.experiment_store = ExperimentRunStore(
            Path(__file__).resolve().parents[1] / "experiment_runs"
        )
        actual_stale_ms = (
            self.phase5_motion_settings.get("actual_feedback_max_age_ms", 500)
            if self.phase5_motion_settings is not None else 500
        )
        # Existing driver torque telemetry is lower-rate than Port10000 joint
        # feedback; keep a separate freshness window and report staleness
        # explicitly instead of fabricating high-rate torque during Phase5.
        self.experiment_torque_cache = TorqueCache(stale_after_ms=1500)
        self.experiment_recorder = ExperimentRecorder(
            self.experiment_store,
            actual_stale_after_ms=int(actual_stale_ms),
            expected_packet_period_ms=self.port10000_expected_period_ms,
        )

        if self.port10000_actual_feedback is not None:
            self.port10000_actual_feedback.subscribe_packets(self.experiment_recorder.ingest_actual_packet)

        self.active_jog: Optional[Dict[str, Any]] = None
        self.active_sequence: Optional[Dict[str, Any]] = None
        self.sequence_cancel_requested = False
        self.stop_generation = 0
        self.active_motion: Optional[Dict[str, Any]] = None
        self.active_lock = threading.Lock()
        self.last_jog_time = 0.0
        self.deadman_timeout = 0.45
        self.repeat_period = 0.12

        self.waypoint_path = Path(__file__).resolve().parents[1] / "tasks/waypoints.json"
        self.waypoint_path.parent.mkdir(parents=True, exist_ok=True)
        self.center_path_store = CenterPathStore(
            Path(__file__).resolve().parents[1] / "saved_paths"
        )

        self.create_subscription(JointState, f"{cfg['left']['prefix']}/joint_position", self.left_joint_cb, 10)
        self.create_subscription(JointState, f"{cfg['right']['prefix']}/joint_position", self.right_joint_cb, 10)
        self.create_subscription(RobotMsg, f"{cfg['left']['prefix']}/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, f"{cfg['right']['prefix']}/robot_states", self.right_state_cb, 10)
        self.create_subscription(
            Float64MultiArray,
            f"{cfg['left']['prefix']}/joint_torque_raw",
            self.left_torque_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            f"{cfg['right']['prefix']}/joint_torque_raw",
            self.right_torque_cb,
            10,
        )

        self.operator_runtime = OperatorRuntime()
        self.operator_control_lock = threading.Lock()
        self.operator_state_clients = {
            (side, control): self.create_client(SetBool, f"{cfg[side]['prefix']}/robot_{control}")
            for side in ("left", "right") for control in ("power", "enable")
        }
        self.left_jog = self.create_client(Move, f"{cfg['left']['prefix']}/jog")
        self.right_jog = self.create_client(Move, f"{cfg['right']['prefix']}/jog")
        self.left_move = self.create_client(Move, f"{cfg['left']['prefix']}/joint_move")
        self.right_move = self.create_client(Move, f"{cfg['right']['prefix']}/joint_move")
        self.left_linear_move = self.create_client(Move, f"{cfg['left']['prefix']}/linear_move")
        self.right_linear_move = self.create_client(Move, f"{cfg['right']['prefix']}/linear_move")
        self.left_get_fk = self.create_client(GetFK, f"{cfg['left']['prefix']}/get_fk")
        self.right_get_fk = self.create_client(GetFK, f"{cfg['right']['prefix']}/get_fk")
        self.left_get_ik = self.create_client(GetIK, f"{cfg['left']['prefix']}/get_ik")
        self.right_get_ik = self.create_client(GetIK, f"{cfg['right']['prefix']}/get_ik")
        self.left_get_frame_state = self.create_client(GetFrameState, f"{cfg['left']['prefix']}/get_frame_state")
        self.right_get_frame_state = self.create_client(GetFrameState, f"{cfg['right']['prefix']}/get_frame_state")

        self.left_stop = self.create_client(Empty, f"{cfg['left']['prefix']}/stop_move")
        self.right_stop = self.create_client(Empty, f"{cfg['right']['prefix']}/stop_move")
        self.left_phase5_execute = self.create_client(
            ExecuteJointTrajectory,
            f"{cfg['left']['prefix']}/execute_joint_trajectory",
        )
        self.right_phase5_execute = self.create_client(
            ExecuteJointTrajectory,
            f"{cfg['right']['prefix']}/execute_joint_trajectory",
        )
        self.left_phase5_status = self.create_client(
            GetExecutionStatus,
            f"{cfg['left']['prefix']}/get_execution_status",
        )
        self.right_phase5_status = self.create_client(
            GetExecutionStatus,
            f"{cfg['right']['prefix']}/get_execution_status",
        )
        self.phase5_execution_transport = Phase5AbsoluteStartTransport(
            left_client=self.left_phase5_execute,
            right_client=self.right_phase5_execute,
            request_factory=self.make_execute_joint_trajectory_request,
            wait_future_result=self.wait_future_result,
            stop_generation_getter=lambda: self.stop_generation,
            driver_contract_checker=self.phase5_driver_contract_preflight,
            left_service_name=f"{cfg['left']['prefix']}/execute_joint_trajectory",
            right_service_name=f"{cfg['right']['prefix']}/execute_joint_trajectory",
            servo_filter_config=(
                self.phase5_motion_settings["servo_filter"]
                if self.phase5_motion_settings is not None
                else {
                    "mode": "LEGACY_FORESIGHT",
                    "legacy_max_buf": 15,
                    "legacy_kp": 0.03,
                }
            ),
            servo_step_num=(
                self.phase5_motion_settings["servo_step_num"]
                if self.phase5_motion_settings is not None
                else 1
            ),
            configuration_error=self.phase5_motion_configuration_error,
        )
        self.phase5_execution_coordinator = Phase5ExecutionCoordinator(
            artifact_snapshot_getter=self._phase5_artifact_snapshot,
            phase4_gate_getter=self.phase4_execution_gate_state,
            transport=self.phase5_execution_transport,
            safe_state_checker=self.safe_state_ok,
            start_match_checker=self.phase5_start_match_state,
            legacy_conflict_getter=self._phase5_legacy_conflicts,
            stop_generation_getter=lambda: self.stop_generation,
            abort_callback=self._phase5_abort_existing_stop_path,
            feedback_getter=self.phase5_driver_execution_feedback,
        )

        self.jog_thread = threading.Thread(target=self.jog_loop, daemon=True)
        self.jog_thread.start()

    def left_joint_cb(self, msg):
        self._update_digital_twin_joint_cache("left", msg)

    def right_joint_cb(self, msg):
        self._update_digital_twin_joint_cache("right", msg)

    def left_torque_cb(self, msg):
        self.experiment_torque_cache.update("left", getattr(msg, "data", None))

    def right_torque_cb(self, msg):
        self.experiment_torque_cache.update("right", getattr(msg, "data", None))

    def _update_digital_twin_joint_cache(self, side, msg):
        received_at_ms = wall_clock_ms()
        result = normalize_joint_state_message(
            getattr(msg, "name", []),
            getattr(msg, "position", None),
            self.digital_twin_expected_joint_names[side],
        )
        with self.digital_twin_joint_cache_lock:
            cache = self.digital_twin_joint_cache[side]
            cache["names"] = list(result["names"])
            cache["error"] = result["error"]
            cache["status"] = "VALID" if result["valid"] else "INVALID"
            if result["valid"]:
                normalized_joint = list(result["joint"])
                cache["joint"] = normalized_joint
                cache["received_at_ms"] = received_at_ms
                cache["mapping"] = result["mapping"]
                if side == "left":
                    self.left_joint = list(normalized_joint)
                else:
                    self.right_joint = list(normalized_joint)

    def left_state_cb(self, msg):
        self.left_state = msg
        self._update_digital_twin_robot_state_cache("left", msg)

    def right_state_cb(self, msg):
        self.right_state = msg
        self._update_digital_twin_robot_state_cache("right", msg)

    def _update_digital_twin_robot_state_cache(self, side, msg):
        normalized = normalize_robot_state_message(msg)
        with self.digital_twin_robot_state_cache_lock:
            cache = self.digital_twin_robot_state_cache[side]
            cache["state"] = (
                dict(normalized["state"]) if normalized["valid"] else None
            )
            cache["received_at_ms"] = wall_clock_ms() if normalized["valid"] else None
            cache["valid"] = normalized["valid"]
            cache["status"] = "VALID" if normalized["valid"] else "INVALID"
            cache["error"] = normalized["error"]

    def robot_state_dict(self, state):
        if state is None:
            return None
        return {
            "power_state": int(state.power_state),
            "servo_state": int(state.servo_state),
            "motion_state": int(state.motion_state),
            "collision_state": int(state.collision_state),
        }

    def status(self):
        def to_builtin(obj):
            if obj is None:
                return None

            if isinstance(obj, (str, int, float, bool)):
                return obj

            if isinstance(obj, (list, tuple)):
                return [to_builtin(x) for x in obj]

            if isinstance(obj, dict):
                return {str(k): to_builtin(v) for k, v in obj.items()}

            # ROS2 message: use get_fields_and_field_types() if available
            if hasattr(obj, "get_fields_and_field_types"):
                out = {}
                try:
                    for name in obj.get_fields_and_field_types().keys():
                        out[name] = to_builtin(getattr(obj, name))
                    return out
                except Exception:
                    pass

            # ROS2 message fallback: use __slots__
            slots = getattr(obj, "__slots__", None)
            if slots:
                out = {}
                for slot in slots:
                    clean = slot[1:] if str(slot).startswith("_") else slot

                    if hasattr(obj, clean):
                        out[clean] = to_builtin(getattr(obj, clean))
                    elif hasattr(obj, slot):
                        out[clean] = to_builtin(getattr(obj, slot))

                if out:
                    return out

            # Last fallback: avoid FastAPI JSON encoder crash
            return str(obj)

        left_tcp = self.get_fk_pose("left")
        right_tcp = self.get_fk_pose("right")

        return {
            "left": {
                "joint": to_builtin(self.left_joint),
                "tcp": to_builtin(left_tcp),
                "state": to_builtin(self.left_state),
            },
            "right": {
                "joint": to_builtin(self.right_joint),
                "tcp": to_builtin(right_tcp),
                "state": to_builtin(self.right_state),
            },
            "active_jog": to_builtin(self.active_jog),
            "active_motion": to_builtin(self.active_motion),
            "active_sequence": to_builtin(self.active_sequence),
            "waypoints": to_builtin(self.load_waypoints()),
        }

    def operator_cache(self):
        with self.digital_twin_robot_state_cache_lock:
            return {side: {**value, "state": dict(value["state"]) if value["state"] else None}
                    for side, value in self.digital_twin_robot_state_cache.items()}

    def operator_live_sides(self):
        # ROS graph discovery only, no SDK calls and no additional robot sessions.
        try:
            names = {name for name, _namespace in self.get_node_names_and_namespaces()}
            services = {name for name, _types in self.get_service_names_and_types()}
            return {side for side, spec in DRIVERS.items()
                    if spec["node"] in names or any(
                        name.startswith(spec["prefix"] + "/") for name in services)}
        except Exception as error:
            raise RuntimeRejected(f"ROS graph unavailable; duplicate prevention cannot be verified: {error}") from error

    def operator_execution_snapshots(self):
        # Never call coordinator inspection/is_active here: those can request abort
        # on peer feedback errors. Browser cleanup must be strictly observational.
        futures, result = {}, {}
        for side in ("left", "right"):
            client = getattr(self, f"{side}_phase5_status")
            try:
                if client.service_is_ready():
                    futures[side] = client.call_async(GetExecutionStatus.Request())
            except Exception:
                pass
        deadline = time.monotonic() + 0.3
        for side, future in futures.items():
            response = self.wait_future_result(future, timeout=max(0.0, deadline - time.monotonic()))
            if response is not None:
                result[side] = {"valid": response.valid and response.ret == 1,
                                "state": response.state, "active": response.active,
                                "trajectory_id": response.trajectory_id}
        return result

    def operator_status(self):
        status = self.operator_runtime.status(self.operator_cache(), self.operator_live_sides())
        executions = self.operator_execution_snapshots()
        local = self.phase5_execution_coordinator.passive_execution_snapshot()
        local_active = local.get("state") == "PREPARING"
        if local.get("state") in {"ARMED", "RUNNING", "ABORT_REQUESTED"}:
            local_active = not all(
                executions.get(side, {}).get("valid") is True
                and executions[side].get("active") is False
                and executions[side].get("state") in TERMINAL
                and executions[side].get("trajectory_id") == local.get("trajectory_id")
                for side in ("left", "right"))
        with self.active_lock:
            local_active = local_active or self.active_jog is not None or (
                isinstance(self.active_sequence, dict)
                and self.active_sequence.get("status") in {"running", "stop_requested"})
            if isinstance(self.active_motion, dict):
                sent_at = self.active_motion.get("sent_at_unix_s")
                local_active = local_active or (isinstance(sent_at, (float, int)) and time.time() - sent_at < 2)
        local_active = local_active or _d33_activity_snapshot().get("active", False)
        status["execution"] = executions
        status["shutdown"] = shutdown_decision(status, executions, local_active)
        return status

    def operator_connect(self):
        return self.operator_runtime.connect(self.operator_cache, self.operator_live_sides)

    def operator_control(self, side, control, desired):
        with self.operator_control_lock:
            status = self.operator_status()
            if not status["shutdown"]["safe"]:
                raise RuntimeRejected(
                    "Robot state control requires confirmed idle/terminal state: "
                    + str(status["shutdown"]["reason"])
                )
            return control_robot(side, control, desired, status["drivers"][side],
                                 self.operator_state_clients[(side, control)], SetBool.Request,
                                 self.wait_future_result)

    def operator_reset_moveit(self):
        if not self.operator_status()["shutdown"]["safe"]:
            raise RuntimeRejected("MoveIt reset requires confirmed idle/terminal robot execution")
        return self.operator_runtime.reset_moveit()

    def digital_twin_ros_joint_status(self):
        """Return only cached ROS joint feedback; retain Phase-5 authority."""
        with self.digital_twin_joint_cache_lock:
            cache_snapshot = {
                side: {
                    "joint": (
                        list(values["joint"])
                        if values["joint"] is not None
                        else None
                    ),
                    "received_at_ms": values["received_at_ms"],
                    "mapping": values["mapping"],
                    "error": values["error"],
                    "names": list(values["names"]),
                }
                for side, values in self.digital_twin_joint_cache.items()
            }
        return build_digital_twin_joint_status(cache_snapshot)

    def digital_twin_joint_status(self):
        """Select actual visualization feedback without changing safety authority."""
        ros_status = self.digital_twin_ros_joint_status()
        port10000_cache = (
            self.port10000_actual_feedback.snapshot()
            if self.port10000_actual_feedback is not None
            else {}
        )
        return select_visualization_joint_status(
            port10000_cache,
            ros_status,
            server_time_ms=wall_clock_ms(),
            expected_period_ms=self.port10000_expected_period_ms,
            freshness_threshold_ms=self.port10000_freshness_threshold_ms,
        )

    def digital_twin_robot_status(self):
        """Return RobotMsg and JointState freshness from caches only."""
        with self.digital_twin_robot_state_cache_lock:
            state_snapshot = {
                side: {
                    "state": dict(values["state"]) if values["state"] else None,
                    "received_at_ms": values["received_at_ms"],
                    "valid": values["valid"],
                    "status": values["status"],
                    "error": values["error"],
                }
                for side, values in self.digital_twin_robot_state_cache.items()
            }
        with self.digital_twin_joint_cache_lock:
            joint_snapshot = {
                side: {
                    "joint": list(values["joint"]) if values["joint"] else None,
                    "received_at_ms": values["received_at_ms"],
                    "status": values["status"],
                    "error": values["error"],
                }
                for side, values in self.digital_twin_joint_cache.items()
            }
        return build_digital_twin_robot_status(
            state_snapshot,
            joint_snapshot,
            server_time_ms=wall_clock_ms(),
        )

    def digital_twin_tcp_status(self):
        """Call only existing read-only FK, with failures isolated per arm."""
        poses = {}
        errors = {}
        for side in ("left", "right"):
            try:
                poses[side] = self.get_fk_pose(side)
                if poses[side] is None:
                    errors[side] = (
                        "FK unavailable: joint feedback missing, service unavailable, "
                        "timeout, or invalid service response"
                    )
            except Exception as error:
                poses[side] = None
                errors[side] = f"FK failed: {error}"
        return build_digital_twin_tcp_status(
            poses,
            errors=errors,
            server_time_ms=wall_clock_ms(),
        )

    def validate_digital_twin_trajectory(self, trajectory, sampled_path=None):
        """Check stored and optional sampled states without invoking motion APIs."""
        normalize_sampled_path_options(sampled_path)
        if self.moveit_state_validation_bridge is None:
            return empty_validation_result(
                "UNAVAILABLE",
                self.moveit_state_validation_error
                or "MoveIt state-validity client is unavailable",
            )
        return self.moveit_state_validation_bridge.validate_trajectory(
            trajectory, sampled_path
        )

    def _phase3_planning_authority(self, normalized_request):
        """Resolve and verify one immutable grasp/calibration input before IK."""
        locked = self.rigid_grasp_configuration.locked_snapshot()
        if locked is None:
            return None, planning_authority_rejected_result(
                "RIGID_GRASP_UNLOCKED_OR_MISSING",
                "Phase-3 planning requires a current immutable GRASP_LOCKED snapshot",
                normalized_request,
            )
        comparisons = (
            (
                "expected_grasp_content_revision",
                locked.content_revision,
                "GRASP_CONTENT_REVISION_MISMATCH",
            ),
            (
                "expected_lock_generation",
                locked.lock_generation,
                "LOCK_GENERATION_MISMATCH",
            ),
            (
                "expected_lock_revision",
                locked.lock_revision,
                "LOCK_REVISION_MISMATCH",
            ),
        )
        for field, actual, reason_code in comparisons:
            expected = normalized_request[field]
            if expected != actual:
                return None, planning_authority_rejected_result(
                    reason_code,
                    f"{field} does not match current authoritative locked grasp",
                    normalized_request,
                    diagnostics={"expected": expected, "current": actual},
                )
        try:
            calibration = world_calibration_public_payload()
            web_model_revision = get_web_model_calibration_revision()
        except Exception as error:
            return None, planning_authority_rejected_result(
                "CALIBRATION_STATE_UNAVAILABLE_OR_INVALID",
                f"Canonical World calibration is unavailable or invalid: {error}",
                normalized_request,
            )
        revision = calibration.get("revision")
        expected_calibration = normalized_request["expected_calibration_revision"]
        expected_model = normalized_request["expected_model_calibration_revision"]
        if expected_calibration != revision:
            return None, planning_authority_rejected_result(
                "CALIBRATION_REVISION_MISMATCH",
                "Expected calibration revision does not match backend authority",
                normalized_request,
                diagnostics={"expected": expected_calibration, "current": revision},
            )
        if web_model_revision != revision:
            return None, planning_authority_rejected_result(
                "MODEL_CALIBRATION_REVISION_MISMATCH",
                "Checked-in Web model revision does not match backend calibration",
                normalized_request,
                diagnostics={"model": web_model_revision, "backend": revision},
            )
        if expected_model != web_model_revision:
            return None, planning_authority_rejected_result(
                "MODEL_CALIBRATION_REVISION_MISMATCH",
                "Client-expected Web/model revision does not match checked-in model metadata",
                normalized_request,
                diagnostics={
                    "expected_model": expected_model,
                    "current_model": web_model_revision,
                },
            )
        content = locked.content
        authority = {
            "grasp_status": GRASP_LOCKED,
            "grasp_source": "LOCKED OPERATOR GRASP",
            "grasp_content_revision": locked.content_revision,
            "lock_generation": locked.lock_generation,
            "lock_revision": locked.lock_revision,
            "left": content.left.as_payload(),
            "right": content.right.as_payload(),
            "calibration_revision": revision,
            "model_calibration_revision": web_model_revision,
            "calibration_revision_status": "MATCH",
            "calibration_state": calibration["calibration_state"],
            "physical_calibration": calibration["physical_calibration"],
            "physically_calibrated": calibration["physically_calibrated"],
            "tcp_tool_contract_status": calibration["tcp_tool_contract"]["status"],
        }
        return (locked, authority), None

    def _phase3_authority_still_current(self, locked, calibration_revision):
        current = self.rigid_grasp_configuration.locked_snapshot()
        if current is None:
            return False
        if (
            current.content_revision != locked.content_revision
            or current.lock_generation != locked.lock_generation
            or current.lock_revision != locked.lock_revision
        ):
            return False
        try:
            return (
                get_world_calibration_revision() == calibration_revision
                and get_web_model_calibration_revision() == calibration_revision
            )
        except Exception:
            return False

    def plan_digital_twin_object_global(self, request_payload):
        """Run MoveIt-backed Phase-3 planning without invoking JAKA motion APIs."""
        self.invalidate_phase4_unified_validation("NEW GLOBAL PLAN REQUEST")
        normalized_request = normalize_object_global_plan_request(request_payload)
        candidate_attempts_per_arm = normalized_request[
            "candidate_attempts_per_arm"
        ]
        resolved_authority, rejection = self._phase3_planning_authority(
            normalized_request
        )
        if rejection is not None:
            return rejection
        locked, planning_authority = resolved_authority
        if (
            self.object_global_planning_adapter is None
            or self.object_global_planning_joint_limits is None
        ):
            return planning_unavailable_result(
                self.object_global_planning_error
                or "MoveIt object-planning adapter is unavailable",
                candidate_attempts_per_arm,
                normalized_request["initial_joint_state_rad"],
                normalized_request["planning_start_state_source"],
            )
        unavailable = self.object_global_planning_adapter.unavailable_services()
        if unavailable:
            return planning_unavailable_result(
                "MoveIt planning service(s) unavailable: " + ", ".join(unavailable),
                candidate_attempts_per_arm,
                normalized_request["initial_joint_state_rad"],
                normalized_request["planning_start_state_source"],
            )
        with self.object_global_planning_lock:
            try:
                result = plan_object_global(
                    request_payload,
                    self.object_global_planning_adapter,
                    self.object_global_planning_joint_limits,
                    grasp_model=locked.content.model,
                    planning_authority=planning_authority,
                )
                if not self._phase3_authority_still_current(
                    locked, planning_authority["calibration_revision"]
                ):
                    return planning_authority_rejected_result(
                        "PLANNING_AUTHORITY_CHANGED_IN_FLIGHT",
                        "Locked grasp or calibration changed while planning was in flight",
                        normalized_request,
                    )
                return result
            except ObjectGlobalPlanInputError:
                raise
            except Exception as error:
                return planning_unavailable_result(
                    f"Object planning failed: {error}",
                    candidate_attempts_per_arm,
                    normalized_request["initial_joint_state_rad"],
                    normalized_request["planning_start_state_source"],
                )

    def digital_twin_planning_start_state(self):
        """Expose code-defined planning joints without robot or motion access."""
        return planning_start_state_payload(
            self.object_global_planning_joint_limits
        )

    def digital_twin_grasp_configuration_state(self):
        """Return copy-safe software grasp state without robot access."""
        state = self.rigid_grasp_configuration.snapshot()
        state["world_calibration_revision"] = get_world_calibration_revision()
        return state

    def lock_digital_twin_grasp_configuration(self, request_payload):
        """Validate and lock one rigid grasp, then invalidate old readiness."""
        state = self.rigid_grasp_configuration.lock(request_payload)
        self.invalidate_phase4_unified_validation(
            "RIGID GRASP LOCK/REVISION CHANGED"
        )
        state["world_calibration_revision"] = get_world_calibration_revision()
        return state

    def unlock_digital_twin_grasp_configuration(self):
        """Unlock to an editable draft and invalidate old execution artifacts."""
        state = self.rigid_grasp_configuration.unlock()
        self.invalidate_phase4_unified_validation("RIGID GRASP UNLOCKED")
        state["world_calibration_revision"] = get_world_calibration_revision()
        return state

    def validate_digital_twin_phase4_trajectory(self, request_payload):
        """Validate a Phase-3 plan through MoveIt model FK; never execute it."""
        adapter = self.phase4_fk_validation_adapter
        unavailable_error = self.phase4_fk_validation_error
        if adapter is not None:
            try:
                if not adapter.service_is_ready():
                    adapter = None
                    unavailable_error = "MoveIt /compute_fk service is unavailable"
            except Exception as error:
                adapter = None
                unavailable_error = f"MoveIt /compute_fk readiness check failed: {error}"
        with self.phase4_validation_lock:
            return validate_phase4_trajectory(
                request_payload.get("plan"),
                fk_adapter=adapter,
                joint_position_limits=self.object_global_planning_joint_limits,
                start_state=request_payload.get("start_state"),
                position_tolerance_m=request_payload.get(
                    "position_tolerance_m", DEFAULT_POSITION_TOLERANCE_M
                ),
                orientation_tolerance_rad=request_payload.get(
                    "orientation_tolerance_rad", DEFAULT_ORIENTATION_TOLERANCE_RAD
                ),
                fk_unavailable_error=unavailable_error,
            )

    def validate_digital_twin_phase4_collision(self, request_payload):
        """Classify robot/scene contacts without invoking any execution API."""
        plan = request_payload.get("plan")
        trajectory = phase3_plan_to_robot_trajectory(plan)
        max_joint_step_rad = request_payload.get("max_joint_step_rad", 0.05)
        if self.moveit_state_validation_bridge is None:
            robot_validation = empty_validation_result(
                "UNAVAILABLE",
                self.moveit_state_validation_error
                or "MoveIt state-validity bridge is unavailable",
            )
        else:
            robot_validation = self.moveit_state_validation_bridge.validate_trajectory(
                trajectory,
                {"enabled": True, "max_joint_step_rad": max_joint_step_rad},
            )
        # Repository audit found no canonical physical Object or environment
        # collision geometry. Empty scenes remain explicitly NOT_EVALUATED.
        with self.phase4_collision_validation_lock:
            return build_phase4b_collision_result(
                plan,
                robot_validation,
                scene_validator=self.phase4_planning_scene_adapter,
                object_geometry=None,
                environment_objects=(),
                max_joint_step_rad=max_joint_step_rad,
            )

    def invalidate_phase4_unified_validation(self, reason):
        """Fail closed whenever the plan artifact or validation input changes."""
        if hasattr(self, "phase5_execution_coordinator"):
            self.phase5_execution_coordinator.invalidate_authority(str(reason))
        with self.phase4_unified_validation_lock:
            self.phase4_unified_validation_generation += 1
            self.phase4_unified_validation_report = None
            self.phase4_unified_validation_running = False
            self.phase4_unified_validation_invalidated_reason = str(reason)
            with self.phase5_execution_artifact_lock:
                self.phase5_execution_artifact = None
                self.phase5_execution_artifact_invalidated_reason = str(reason)
                self.phase5_execution_artifact_generation = (
                    self.phase4_unified_validation_generation
                )

    def _phase5_artifact_snapshot(self):
        with self.phase5_execution_artifact_lock:
            return (
                self.phase5_execution_artifact,
                int(self.phase5_execution_artifact_generation),
                self.phase5_execution_artifact_invalidated_reason,
            )

    def _phase5_legacy_conflicts(self):
        conflicts = []
        with self.active_lock:
            if self.active_jog is not None:
                conflicts.append("LEGACY_JOG_ACTIVE")
            if (
                isinstance(self.active_sequence, dict)
                and self.active_sequence.get("status") in {"running", "stop_requested"}
            ):
                conflicts.append("LEGACY_SEQUENCE_OR_PROGRAM_ACTIVE")
            recent_motion = False
            if isinstance(self.active_motion, dict):
                sent_at = self.active_motion.get("sent_at_unix_s")
                recent_motion = isinstance(sent_at, (int, float)) and (
                    time.time() - float(sent_at) < 2.0
                )
            robot_reports_motion = any(
                state is not None and int(getattr(state, "motion_state", 0)) != 0
                for state in (self.left_state, self.right_state)
            )
            if recent_motion or robot_reports_motion:
                conflicts.append("LEGACY_OR_ROBOT_MOTION_ACTIVE")
        return conflicts

    def _phase5_abort_existing_stop_path(self):
        return self.request_stop_all("both")

    def phase5_start_match_state(self, artifact=None):
        """Evaluate cached Actual joints against frozen sample zero only."""
        if self.phase5_motion_settings is None:
            return {
                "match": False,
                "state": "BLOCKED",
                "reason": (
                    "PHASE5_MOTION_CONFIGURATION_INVALID: "
                    + str(self.phase5_motion_configuration_error)
                ),
                "max_delta_rad": None,
                "threshold_rad": None,
            }
        if artifact is None:
            artifact, _generation, _reason = self._phase5_artifact_snapshot()
        settings = self.phase5_motion_settings
        return evaluate_actual_start_match(
            self.digital_twin_ros_joint_status(),
            artifact,
            threshold_rad=settings["start_match_threshold_rad"],
            max_age_ms=settings["actual_feedback_max_age_ms"],
        )

    def phase5_replan_from_current(self):
        """Capture a fresh no-motion planning override for the next UI plan."""
        if self.phase5_motion_settings is None:
            return {
                "ok": False,
                "error": "PHASE5_MOTION_CONFIGURATION_INVALID",
                "detail": self.phase5_motion_configuration_error,
                "motion_dispatched": False,
            }
        snapshot = fresh_actual_joint_state(
            self.digital_twin_ros_joint_status(),
            max_age_ms=self.phase5_motion_settings["actual_feedback_max_age_ms"],
        )
        if snapshot["ok"] is not True:
            return {
                "ok": False,
                "error": "REPLAN_FROM_CURRENT_FEEDBACK_BLOCKED",
                "detail": snapshot["reason"],
                "motion_dispatched": False,
            }
        return {
            "ok": True,
            "motion_dispatched": False,
            "initial_joint_state_rad": snapshot["initial_joint_state_rad"],
            "planning_start_state_source": REPLAN_FROM_CURRENT_SOURCE,
            "actual_feedback_max_age_ms": snapshot["max_age_ms"],
            "semantic": (
                "NO MOTION — ONE-SHOT OVERRIDE FOR THE NEXT GLOBAL PLAN; "
                "NORMAL PHASE4/FREEZE/GHOST/CONFIRMATION STILL REQUIRED"
            ),
        }

    def phase5_move_to_initial(self):
        """Explicit recovery motion to the remembered last-executed sample zero."""
        settings = self.phase5_motion_settings
        if settings is None:
            return {
                "ok": False,
                "error": "PHASE5_MOTION_CONFIGURATION_INVALID",
                "detail": self.phase5_motion_configuration_error,
            }
        if self.phase5_execution_coordinator.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ACTIVE"}
        safe, safe_reason = self.safe_state_ok("both")
        if not safe:
            return {
                "ok": False,
                "error": "ROBOT_SAFE_STATE_BLOCKED",
                "detail": safe_reason,
            }
        conflicts = self._phase5_legacy_conflicts()
        if conflicts:
            return {
                "ok": False,
                "error": "LEGACY_CONFLICT_BLOCKED",
                "blocking_reasons": conflicts,
            }
        fresh = fresh_actual_joint_state(
            self.digital_twin_ros_joint_status(),
            max_age_ms=settings["actual_feedback_max_age_ms"],
        )
        if fresh["ok"] is not True:
            return {
                "ok": False,
                "error": "ACTUAL_FEEDBACK_BLOCKED",
                "detail": fresh["reason"],
            }

        with self.phase5_recovery_initial_lock:
            remembered = (
                dict(self.phase5_recovery_initial)
                if isinstance(self.phase5_recovery_initial, dict)
                else None
            )
        if remembered is not None:
            left_initial = list(remembered["left"])
            right_initial = list(remembered["right"])
            target_source = remembered["source"]
            target_fingerprint = remembered["artifact_fingerprint"]
            target_generation = remembered["artifact_generation"]
            target_trajectory_name = remembered["trajectory_name"]
        else:
            artifact, generation, artifact_reason = self._phase5_artifact_snapshot()
            if artifact is None:
                return {
                    "ok": False,
                    "error": "INITIAL_RECOVERY_TARGET_UNAVAILABLE",
                    "detail": artifact_reason,
                }
            try:
                left_initial = [float(value) for value in artifact.left_positions_rad[0]]
                right_initial = [float(value) for value in artifact.right_positions_rad[0]]
                if len(left_initial) != 6 or len(right_initial) != 6:
                    raise ValueError("sample zero must contain Left6 + Right6")
            except (AttributeError, IndexError, TypeError, ValueError) as error:
                return {
                    "ok": False,
                    "error": "FROZEN_ARTIFACT_SAMPLE_0_INVALID",
                    "detail": str(error),
                }
            target_source = "CURRENT_FROZEN_ARTIFACT_SAMPLE_0_FALLBACK"
            target_fingerprint = artifact.artifact_fingerprint
            target_generation = int(generation)
            target_trajectory_name = str(artifact.trajectory_name)

        # Recheck the two conflict authorities immediately before the existing
        # move path performs its own Phase-5/safe-state checks.
        if self.phase5_execution_coordinator.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ACTIVE"}
        conflicts = self._phase5_legacy_conflicts()
        if conflicts:
            return {
                "ok": False,
                "error": "LEGACY_CONFLICT_BLOCKED",
                "blocking_reasons": conflicts,
            }
        result = self.move_both_joint(
            left_target=left_initial,
            right_target=right_initial,
            side="both",
            vel=settings["recovery_joint_vel_rad_s"],
            acc=settings["recovery_joint_acc_rad_s2"],
        )
        result["action"] = "MOVE_TO_REMEMBERED_PHASE5_INITIAL_REAL_MOTION"
        result["initial_target_source"] = target_source
        result["artifact_fingerprint"] = target_fingerprint
        result["artifact_generation"] = target_generation
        result["trajectory_name"] = target_trajectory_name
        result["recovery_joint_vel_rad_s"] = settings[
            "recovery_joint_vel_rad_s"
        ]
        result["recovery_joint_acc_rad_s2"] = settings[
            "recovery_joint_acc_rad_s2"
        ]
        return result

    def phase5_execution_state(self):
        """Read authoritative execution feedback plus existing joint caches."""
        result = self.phase5_execution_coordinator.inspection_state()
        actual_joints = self.digital_twin_ros_joint_status()
        result["actual_joints"] = actual_joints
        monitoring_actual_joints = self.digital_twin_joint_status()
        result["phase6_joint_tracking"] = compute_joint_tracking_error(
            result.get("driver_feedback"), monitoring_actual_joints
        )
        result["actual_joint_semantic"] = (
            "P5.11 CACHED ROS JOINT FEEDBACK ONLY — NO NEW ROBOT CALL"
        )
        result["motion_quality_configuration"] = (
            dict(self.phase5_motion_settings)
            if self.phase5_motion_settings is not None
            else {
                "valid": False,
                "error": self.phase5_motion_configuration_error,
            }
        )
        fresh = (
            fresh_actual_joint_state(
                actual_joints,
                max_age_ms=self.phase5_motion_settings[
                    "actual_feedback_max_age_ms"
                ],
            )
            if self.phase5_motion_settings is not None
            else {"ok": False, "reason": "PHASE5_MOTION_CONFIGURATION_INVALID"}
        )
        execution_state = (result.get("execution") or {}).get("state")
        with self.phase5_recovery_initial_lock:
            remembered_initial = (
                dict(self.phase5_recovery_initial)
                if isinstance(self.phase5_recovery_initial, dict)
                else None
            )
        current_artifact_available = (result.get("artifact") or {}).get("available") is True
        result["initial_recovery_target"] = {
            "available": remembered_initial is not None or current_artifact_available,
            "remembered_from_last_execution": remembered_initial is not None,
            "source": (
                remembered_initial.get("source")
                if remembered_initial is not None
                else (
                    "CURRENT_FROZEN_ARTIFACT_SAMPLE_0_FALLBACK"
                    if current_artifact_available
                    else None
                )
            ),
            "artifact_fingerprint": (
                remembered_initial.get("artifact_fingerprint")
                if remembered_initial is not None
                else (result.get("artifact") or {}).get("artifact_fingerprint")
            ),
            "artifact_generation": (
                remembered_initial.get("artifact_generation")
                if remembered_initial is not None
                else (result.get("artifact") or {}).get("generation")
            ),
            "trajectory_name": (
                remembered_initial.get("trajectory_name")
                if remembered_initial is not None
                else None
            ),
        }
        result["ready_to_replan_from_current"] = fresh.get("ok") is True
        result["ready_to_move_to_initial"] = bool(
            fresh.get("ok") is True
            and result["initial_recovery_target"]["available"] is True
            and (result.get("safe_state") or {}).get("ok") is True
            and not result.get("legacy_conflicts")
            and execution_state not in {"PREPARING", "ARMED", "RUNNING", "ABORT_REQUESTED"}
        )
        try:
            self.experiment_recorder.observe(
                result,
                monitoring_actual_joints,
                self.experiment_torque_cache.snapshot(),
            )
        except Exception as error:
            # Experiment persistence is observational: it must never change
            # Phase5 status, terminal authority, or accepted robot execution.
            self.experiment_recorder.record_error(error)
        result["experiment_recorder"] = self.experiment_recorder.state()
        return result

    def phase5_driver_contract_preflight(self):
        """Read-only proof that both running drivers own the Linear-v2 contract."""
        clients = {
            "left": self.left_phase5_status,
            "right": self.right_phase5_status,
        }
        futures = {}
        sides = {}
        for side, client in clients.items():
            try:
                if not bool(client.service_is_ready()):
                    sides[side] = {
                        "ok": False,
                        "error": f"{side} /get_execution_status service unavailable",
                    }
                    continue
                futures[side] = client.call_async(GetExecutionStatus.Request())
            except Exception as error:
                sides[side] = {"ok": False, "error": str(error)}

        # Dispatch both read-only snapshots before waiting for either response.
        deadline = time.monotonic() + 0.12
        for side, future in futures.items():
            remaining_s = max(0.0, deadline - time.monotonic())
            response = self.wait_future_result(future, timeout=remaining_s)
            if response is None:
                sides[side] = {
                    "ok": False,
                    "error": f"{side} /get_execution_status timeout/no response",
                }
                continue
            message = str(getattr(response, "message", "") or "")
            valid = getattr(response, "valid", False) is True
            marker_present = PHASE5_DRIVER_CONTRACT_MARKER in message
            sides[side] = {
                "ok": bool(valid and marker_present),
                "valid": valid,
                "marker_present": marker_present,
                "message": message,
            }
        return {
            "ok": all(sides.get(side, {}).get("ok") is True for side in ("left", "right")),
            "expected_marker": PHASE5_DRIVER_CONTRACT_MARKER,
            "sides": sides,
        }

    def phase5_driver_execution_feedback(self, trajectory_id):
        """Read both driver memory snapshots and aggregate them fail-closed."""
        clients = {
            "left": self.left_phase5_status,
            "right": self.right_phase5_status,
        }
        futures = {}
        responses = {"left": None, "right": None}
        errors = {}
        for side, client in clients.items():
            try:
                ready = bool(client.service_is_ready())
            except Exception as error:
                errors[side] = str(error)
                continue
            if not ready:
                errors[side] = f"{side} /get_execution_status service unavailable"
                continue
            try:
                futures[side] = client.call_async(GetExecutionStatus.Request())
            except Exception as error:
                errors[side] = str(error)

        # Both read-only requests are dispatched before either wait.
        deadline = time.monotonic() + 0.12
        for side, future in futures.items():
            remaining_s = max(0.0, deadline - time.monotonic())
            responses[side] = self.wait_future_result(future, timeout=remaining_s)
            if responses[side] is None:
                errors[side] = f"{side} /get_execution_status timeout/no response"
        return build_phase5_execution_feedback(
            expected_trajectory_id=trajectory_id,
            left_response=responses["left"],
            right_response=responses["right"],
            left_error=errors.get("left"),
            right_error=errors.get("right"),
            actual_joint_feedback=self.digital_twin_ros_joint_status(),
        )

    def prepare_phase5_execution(self):
        """Capture no-expiry pending operator authority without dispatching motion."""
        return self.phase5_execution_coordinator.prepare()

    def execute_phase5_execution(self, operator_confirmed):
        """Consume explicit operator authority and remember validated sample zero."""
        artifact, generation, _reason = self._phase5_artifact_snapshot()
        result = self.phase5_execution_coordinator.execute(operator_confirmed)
        execution = result.get("execution") if isinstance(result, dict) else None
        if (
            isinstance(result, dict)
            and result.get("ok") is True
            and result.get("accepted") is True
            and artifact is not None
            and isinstance(execution, dict)
            and execution.get("artifact_fingerprint") == artifact.artifact_fingerprint
            and int(execution.get("artifact_generation")) == int(generation)
        ):
            try:
                remembered = {
                    "left": tuple(float(value) for value in artifact.left_positions_rad[0]),
                    "right": tuple(float(value) for value in artifact.right_positions_rad[0]),
                    "artifact_fingerprint": artifact.artifact_fingerprint,
                    "artifact_generation": int(generation),
                    "trajectory_name": str(artifact.trajectory_name),
                    "source": "LAST_EXECUTED_FROZEN_ARTIFACT_SAMPLE_0",
                }
                if len(remembered["left"]) != 6 or len(remembered["right"]) != 6:
                    raise ValueError("sample zero must contain Left6 + Right6")
                with self.phase5_recovery_initial_lock:
                    self.phase5_recovery_initial = remembered
            except (AttributeError, IndexError, TypeError, ValueError):
                # Execution authority already passed; recovery-memory failure must
                # never mutate or retroactively cancel that accepted execution.
                with self.phase5_recovery_initial_lock:
                    self.phase5_recovery_initial = None
            try:
                self.experiment_recorder.bind_execution(execution)
            except Exception as error:
                # The paired driver submission is already accepted. Recorder
                # failure is reported separately and can never alter result.
                self.experiment_recorder.record_error(error)
        return result

    def arm_experiment_recorder(self, request_payload):
        """Arm observational storage against the current artifact/report identity."""
        artifact, artifact_generation, artifact_reason = self._phase5_artifact_snapshot()
        if artifact is None:
            raise ExperimentError(
                f"Current frozen artifact required: {artifact_reason}"
            )
        gate = self.phase4_execution_gate_state(artifact.plan_fingerprint)
        if gate.get("execution_ready") is not True:
            raise ExperimentError(
                "Current matching Phase4 execution gate must be PASS before arming"
            )
        with self.phase4_unified_validation_lock:
            report = (
                json.loads(json.dumps(self.phase4_unified_validation_report))
                if self.phase4_unified_validation_report is not None else None
            )
            validation_generation = int(self.phase4_unified_validation_generation)
        current_artifact, current_generation, _reason = self._phase5_artifact_snapshot()
        if (
            current_artifact is not artifact
            or int(current_generation) != int(artifact_generation)
            or validation_generation != int(artifact_generation)
        ):
            raise ExperimentError(
                "Current artifact or Phase4 PASS changed while arming; refresh and retry"
            )
        locked = self.rigid_grasp_configuration.locked_snapshot()
        if (
            locked is None
            or locked.content_revision != artifact.grasp_content_revision
            or locked.lock_generation != artifact.grasp_lock_generation
            or locked.lock_revision != artifact.grasp_lock_revision
        ):
            raise ExperimentError(
                "Current locked grasp does not match the frozen artifact"
            )
        return self.experiment_recorder.arm(
            artifact=artifact,
            artifact_generation=artifact_generation,
            phase4_report=report,
            phase4_generation=validation_generation,
            metadata=request_payload,
            plan_snapshot=request_payload.get("plan_snapshot"),
            motion_configuration=self.phase5_motion_settings or {
                "valid": False,
                "error": self.phase5_motion_configuration_error,
            },
            robot_configuration=self.cfg,
            grasp_snapshot=locked.as_payload() if locked is not None else None,
        )

    def experiment_recorder_state(self):
        return self.experiment_recorder.state()

    def disarm_experiment_recorder(self):
        return self.experiment_recorder.disarm()

    def list_experiment_runs(self):
        return {
            "ok": True,
            "runs": self.experiment_store.list_runs(),
            "recorder": self.experiment_recorder.state(),
        }

    def load_experiment_run(self, run_id):
        return {"ok": True, "run": self.experiment_store.load(run_id)}

    def analyze_experiment_run_exp1(self, run_id):
        """Analyze persisted EXP-0 evidence only; never contact either robot."""
        return {
            "ok": True,
            "run_id": run_id,
            "exp1_analysis": self.experiment_store.analyze_exp1(run_id),
        }

    def analyze_experiment_run_exp2(self, run_id):
        """Compute model-based rigid-grasp metrics from persisted evidence only."""
        return {
            "ok": True,
            "run_id": run_id,
            "exp2_analysis": self.experiment_store.analyze_exp2(run_id),
        }

    def relabel_experiment_run(self, run_id, label):
        return {
            "ok": True,
            "manifest": self.experiment_store.relabel(run_id, label),
        }

    def delete_experiment_run(self, run_id):
        active_run_id = self.experiment_recorder.state().get("run_id")
        active_state = self.experiment_recorder.state().get("state")
        if run_id == active_run_id and active_state in {"ARMED", "RECORDING"}:
            raise ExperimentError("Cannot delete an armed or recording run")
        self.experiment_store.delete(run_id)
        return {"ok": True, "deleted_run_id": run_id}

    def phase5_execution_artifact_state(self):
        """Return a copy-safe P5.1-P5.3 artifact snapshot; never execute it."""
        with self.phase5_execution_artifact_lock:
            artifact = self.phase5_execution_artifact
            reason = self.phase5_execution_artifact_invalidated_reason
            generation = self.phase5_execution_artifact_generation
        if artifact is None:
            return {
                "ok": True,
                "available": False,
                "status": "NOT_FROZEN",
                "reason": reason,
                "generation": generation,
                "artifact": None,
            }
        return {
            "ok": True,
            "available": True,
            "status": "FROZEN",
            "reason": None,
            "generation": generation,
            "artifact": artifact.public_payload(),
        }

    def phase5_execution_transport_state(self):
        """Inspect the P5.10 whole-trajectory transport without dispatch."""
        transport = self.phase5_execution_transport.inspection_state()
        artifact_state = self.phase5_execution_artifact_state()
        return {
            "ok": True,
            "transport": transport,
            "artifact_available": artifact_state["available"],
            "artifact_status": artifact_state["status"],
            "stop_generation": int(self.stop_generation),
            "execution_endpoint_available": True,
            "motion_dispatched": False,
            "semantic": (
                "READ-ONLY P5.10 TRANSPORT INSPECTION — HOST-TIMED COMMON "
                "START IS NOT HARD REAL-TIME"
            ),
        }

    def _phase4_authoritative_input_validation(self, plan):
        """Compare one plan artifact with current backend grasp/calibration."""
        if not isinstance(plan, dict):
            return {
                "status": "FAIL",
                "reason_code": "MALFORMED_PLAN_AUTHORITY",
                "reason": "Plan must be an object with grasp/calibration metadata",
            }
        locked = self.rigid_grasp_configuration.locked_snapshot()
        grasp = plan.get("grasp")
        calibration = plan.get("calibration")
        if locked is None:
            return {
                "status": "FAIL",
                "reason_code": "CURRENT_GRASP_UNLOCKED",
                "reason": "Current authoritative rigid grasp is not locked",
            }
        if not isinstance(grasp, dict):
            return {
                "status": "FAIL",
                "reason_code": "PLAN_GRASP_METADATA_MISSING",
                "reason": "Plan has no locked-grasp identity metadata",
            }
        if not isinstance(calibration, dict):
            return {
                "status": "FAIL",
                "reason_code": "PLAN_CALIBRATION_METADATA_MISSING",
                "reason": "Plan has no calibration/model identity metadata",
            }
        try:
            authoritative_calibration = world_calibration_public_payload()
            web_model_revision = get_web_model_calibration_revision()
        except Exception as error:
            return {
                "status": "FAIL",
                "reason_code": "CALIBRATION_STATE_UNAVAILABLE_OR_INVALID",
                "reason": str(error),
            }
        current_revision = authoritative_calibration["revision"]
        checks = (
            ("status", grasp.get("status"), GRASP_LOCKED, "PLAN_GRASP_NOT_LOCKED"),
            (
                "grasp_content_revision",
                grasp.get("content_revision"),
                locked.content_revision,
                "PLAN_GRASP_REVISION_STALE",
            ),
            (
                "lock_generation",
                grasp.get("lock_generation"),
                locked.lock_generation,
                "PLAN_LOCK_GENERATION_STALE",
            ),
            (
                "lock_revision",
                grasp.get("lock_revision"),
                locked.lock_revision,
                "PLAN_LOCK_REVISION_STALE",
            ),
            (
                "calibration_revision",
                calibration.get("revision"),
                current_revision,
                "PLAN_CALIBRATION_REVISION_STALE",
            ),
            (
                "model_calibration_revision",
                calibration.get("model_revision"),
                web_model_revision,
                "MODEL_CALIBRATION_REVISION_MISMATCH",
            ),
            (
                "calibration_revision_status",
                calibration.get("revision_status"),
                "MATCH",
                "MODEL_CALIBRATION_REVISION_MISMATCH",
            ),
        )
        if web_model_revision != current_revision:
            return {
                "status": "FAIL",
                "reason_code": "MODEL_CALIBRATION_REVISION_MISMATCH",
                "reason": "Checked-in Web model revision does not match backend calibration",
                "plan_value": web_model_revision,
                "current_value": current_revision,
            }
        for field, actual, expected, reason_code in checks:
            if actual != expected:
                return {
                    "status": "FAIL",
                    "reason_code": reason_code,
                    "reason": f"Plan {field} does not match current authority",
                    "field": field,
                    "plan_value": actual,
                    "current_value": expected,
                }
        return {
            "status": "PASS",
            "reason_code": None,
            "reason": "Plan grasp and calibration match current backend authority",
            "grasp_status": GRASP_LOCKED,
            "grasp_content_revision": locked.content_revision,
            "lock_generation": locked.lock_generation,
            "lock_revision": locked.lock_revision,
            "calibration_revision": current_revision,
            "model_calibration_revision": web_model_revision,
            "model_calibration_status": "MATCH",
            "calibration_state": authoritative_calibration["calibration_state"],
            "physical_calibration": authoritative_calibration["physical_calibration"],
            "physically_calibrated": authoritative_calibration[
                "physically_calibrated"
            ],
            "tcp_tool_contract_status": authoritative_calibration[
                "tcp_tool_contract"
            ]["status"],
        }

    def validate_digital_twin_phase4_unified(self, request_payload):
        """Orchestrate existing Phase-4 validators for one immutable plan."""
        if hasattr(self, "phase5_execution_coordinator"):
            self.phase5_execution_coordinator.invalidate_authority(
                "PHASE4 VALIDATION RUNNING"
            )
        with self.phase4_unified_validation_lock:
            self.phase4_unified_validation_generation += 1
            validation_generation = self.phase4_unified_validation_generation
            self.phase4_unified_validation_report = None
            self.phase4_unified_validation_running = True
            self.phase4_unified_validation_invalidated_reason = "VALIDATION RUNNING"
            with self.phase5_execution_artifact_lock:
                self.phase5_execution_artifact = None
                self.phase5_execution_artifact_invalidated_reason = (
                    "PHASE4 VALIDATION RUNNING"
                )
                self.phase5_execution_artifact_generation = validation_generation
        try:
            authority_validation = self._phase4_authoritative_input_validation(
                request_payload.get("plan")
            )
            if authority_validation["status"] != "PASS":
                raise Phase4UnifiedValidationInputError(
                    f"{authority_validation['reason_code']}: "
                    f"{authority_validation['reason']}"
                )
            phase4a = self.validate_digital_twin_phase4_trajectory(request_payload)
            phase4b = self.validate_digital_twin_phase4_collision(request_payload)
            limits = self.object_global_planning_joint_limits
            validation_context = {
                "joint_position_limits": {
                    "source": "CANONICAL_MOVEIT_MODEL_POSITION_LIMITS",
                    "lower_rad": list(limits.lower_rad) if limits is not None else None,
                    "upper_rad": list(limits.upper_rad) if limits is not None else None,
                },
                "velocity_limit_source": (phase4a.get("velocity") or {}).get(
                    "limit_source"
                ),
                "acceleration_limit_status": (phase4a.get("acceleration") or {}).get(
                    "limit_status"
                ),
                "acceleration_limit_source": (phase4a.get("acceleration") or {}).get(
                    "limit_source"
                ),
                "object_scene": phase4b.get("object_scene"),
                "environment_scene": phase4b.get("environment_scene"),
            }
            final_authority_validation = (
                self._phase4_authoritative_input_validation(
                    request_payload.get("plan")
                )
            )
            identity_fields = (
                "grasp_content_revision",
                "lock_generation",
                "lock_revision",
                "calibration_revision",
                "model_calibration_revision",
            )
            if (
                final_authority_validation.get("status") != "PASS"
                or any(
                    final_authority_validation.get(field)
                    != authority_validation.get(field)
                    for field in identity_fields
                )
            ):
                raise Phase4UnifiedValidationInputError(
                    "AUTHORITATIVE_INPUT_CHANGED_DURING_VALIDATION: "
                    "grasp or calibration authority changed while Phase-4 was in flight"
                )
            report = build_phase4_unified_report(
                request_payload.get("plan"),
                phase4a,
                phase4b,
                joint_position_limits=limits,
                start_state=request_payload.get("start_state"),
                position_tolerance_m=request_payload.get(
                    "position_tolerance_m", DEFAULT_POSITION_TOLERANCE_M
                ),
                orientation_tolerance_rad=request_payload.get(
                    "orientation_tolerance_rad", DEFAULT_ORIENTATION_TOLERANCE_RAD
                ),
                max_joint_step_rad=request_payload.get("max_joint_step_rad", 0.05),
                validation_context=validation_context,
                authoritative_input_validation=authority_validation,
            )
            artifact = None
            artifact_error = None
            if (
                report.get("overall_status") == "PASS"
                and (report.get("execution_gate") or {}).get("execution_ready") is True
            ):
                try:
                    artifact = freeze_validated_execution_artifact(
                        request_payload.get("plan"),
                        report,
                        validation_start_state=request_payload.get("start_state"),
                    )
                except Phase5ExecutionArtifactError as error:
                    artifact_error = (
                        f"PHASE5_EXECUTION_ARTIFACT_FREEZE_FAILED: {error}"
                    )
                except Exception as error:
                    artifact_error = (
                        "PHASE5_EXECUTION_ARTIFACT_UNEXPECTED_ERROR: "
                        f"{error}"
                    )
            else:
                artifact_error = "PHASE4_REPORT_NOT_EXECUTION_READY"
            with self.phase4_unified_validation_lock:
                if validation_generation == self.phase4_unified_validation_generation:
                    self.phase4_unified_validation_report = report
                    self.phase4_unified_validation_invalidated_reason = None
                    with self.phase5_execution_artifact_lock:
                        self.phase5_execution_artifact = artifact
                        self.phase5_execution_artifact_invalidated_reason = (
                            None if artifact is not None else artifact_error
                        )
                        self.phase5_execution_artifact_generation = validation_generation
            return report
        finally:
            with self.phase4_unified_validation_lock:
                if validation_generation == self.phase4_unified_validation_generation:
                    self.phase4_unified_validation_running = False

    def phase4_execution_gate_state(self, current_plan_fingerprint=None):
        """Return the backend-authoritative additive execution precondition."""
        with self.phase4_unified_validation_lock:
            report = self.phase4_unified_validation_report
            running = self.phase4_unified_validation_running
            invalidated = self.phase4_unified_validation_invalidated_reason
        state = evaluate_execution_gate(
            report,
            current_plan_fingerprint,
            validation_running=running,
        )
        grasp_state = self.rigid_grasp_configuration.snapshot()
        grasp_blockers = []
        if grasp_state["state"] != GRASP_LOCKED:
            grasp_blockers.append("RIGID_GRASP_NOT_LOCKED")
        evidence = None
        if isinstance(report, dict):
            check = (report.get("checks") or {}).get("authoritative_inputs")
            evidence = check.get("evidence") if isinstance(check, dict) else None
        if not isinstance(evidence, dict):
            grasp_blockers.append("AUTHORITATIVE_PLAN_IDENTITY_MISSING")
        else:
            current_locked = self.rigid_grasp_configuration.locked_snapshot()
            if current_locked is None:
                grasp_blockers.append("RIGID_GRASP_NOT_LOCKED")
            else:
                if evidence.get("grasp_content_revision") != current_locked.content_revision:
                    grasp_blockers.append("PLAN_GRASP_REVISION_STALE")
                if evidence.get("lock_generation") != current_locked.lock_generation:
                    grasp_blockers.append("PLAN_LOCK_GENERATION_STALE")
                if evidence.get("lock_revision") != current_locked.lock_revision:
                    grasp_blockers.append("PLAN_LOCK_REVISION_STALE")
            try:
                current_calibration_revision = get_world_calibration_revision()
                current_model_calibration_revision = (
                    get_web_model_calibration_revision()
                )
            except Exception:
                current_calibration_revision = None
                current_model_calibration_revision = None
                grasp_blockers.append("CALIBRATION_STATE_UNAVAILABLE_OR_INVALID")
            if evidence.get("calibration_revision") != current_calibration_revision:
                grasp_blockers.append("PLAN_CALIBRATION_REVISION_STALE")
            if current_model_calibration_revision != current_calibration_revision:
                grasp_blockers.append("MODEL_CALIBRATION_REVISION_MISMATCH")
            if (
                evidence.get("model_calibration_revision")
                != current_model_calibration_revision
            ):
                grasp_blockers.append("MODEL_CALIBRATION_REVISION_MISMATCH")
        if grasp_blockers:
            state["execution_ready"] = False
            state["execution_ready_label"] = "NO"
            state["status"] = "BLOCKED"
            state["blocking_reasons"] = list(dict.fromkeys(
                [*state.get("blocking_reasons", []), *grasp_blockers]
            ))
        state["invalidated_reason"] = invalidated
        return state

    def safe_state_ok(self, side):
        selected = []
        if side in ("left", "both"):
            selected.append(("LEFT", self.left_state))
        if side in ("right", "both"):
            selected.append(("RIGHT", self.right_state))

        for label, s in selected:
            if s is None:
                return False, f"{label}: no state"
            if s.power_state != 1:
                return False, f"{label}: power_state={s.power_state}"
            if s.servo_state != 1:
                return False, f"{label}: servo_state={s.servo_state}"
            if s.collision_state != 0:
                return False, f"{label}: collision_state={s.collision_state}"

        return True, "ok"

    def axis_to_index(self, axis, direction):
        axis = axis.lower()
        if axis not in AXIS_INDEX:
            raise ValueError(f"Unknown axis: {axis}")

        base = AXIS_INDEX[axis] * 2
        if direction == "+":
            return base
        if direction == "-":
            return base + 1
        raise ValueError("direction must be + or -")

    def invert_index_direction(self, idx):
        return idx + 1 if idx % 2 == 0 else idx - 1

    def get_side_signs(self, side, coord):
        motion = self.cfg.setdefault("motion", {})

        if side == "left":
            if coord == "joint":
                return motion.get("left_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "base":
                return motion.get("left_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "tool":
                return motion.get("left_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

        if side == "right":
            if coord == "joint":
                return motion.get("right_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "base":
                return motion.get("right_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "tool":
                return motion.get("right_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

        return [1, 1, 1, 1, 1, 1]

    def apply_sign_to_index(self, idx, factor):
        return self.invert_index_direction(idx) if factor < 0 else idx

    def compute_indices(self, side, coord, axis, direction, sync_mode):
        base_idx = self.axis_to_index(axis, direction)

        if sync_mode == "raw":
            return base_idx, base_idx

        axis_i = AXIS_INDEX[axis.lower()]
        left_signs = self.get_side_signs("left", coord)
        right_signs = self.get_side_signs("right", coord)

        if len(left_signs) != 6:
            raise ValueError("left sign map must contain 6 values")
        if len(right_signs) != 6:
            raise ValueError("right sign map must contain 6 values")

        left_factor = float(left_signs[axis_i])
        right_factor = float(right_signs[axis_i])

        if side == "both" and sync_mode == "mirror":
            right_factor *= -1.0
        elif sync_mode != "same":
            raise ValueError("sync_mode must be raw, same, or mirror")

        left_idx = self.apply_sign_to_index(base_idx, left_factor)
        right_idx = self.apply_sign_to_index(base_idx, right_factor)

        return left_idx, right_idx

    def make_jog_request(self, coord, index, speed):
        req = Move.Request()
        req.pose = [0.0] * 6
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = 0.0
        req.mvacc = float(speed)  # driver jog ใช้ mvacc เป็น jog velocity
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = int(COORD_MODE[coord])
        req.index = int(index)
        return req

    def wait_future_result(self, future, timeout=0.4):
        start = time.monotonic()
        while not future.done() and (time.monotonic() - start) < timeout:
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None


    def digital_twin_frame_state(self):
        """Read active JAKA Tool/User/Mounting state without modifying the robot."""
        result = {
            "ok": True,
            "mode": "READ_ONLY_NO_MOTION",
            "semantic": (
                "JAKA kine_forward/get_fk uses current tool, current mounting angle, "
                "and current user coordinate"
            ),
            "units": {"translation": "mm", "rotation": "rad"},
            "sides": {},
        }
        for side, client in (
            ("left", self.left_get_frame_state),
            ("right", self.right_get_frame_state),
        ):
            side_result = {"available": False, "error": None}
            if not client.service_is_ready():
                side_result["error"] = f"{side} /get_frame_state service unavailable"
                result["ok"] = False
                result["sides"][side] = side_result
                continue
            response = self.wait_future_result(
                client.call_async(GetFrameState.Request()), timeout=0.8
            )
            if response is None:
                side_result["error"] = f"{side} /get_frame_state timed out"
                result["ok"] = False
                result["sides"][side] = side_result
                continue
            if int(getattr(response, "ret", 0)) != 1:
                side_result["error"] = str(
                    getattr(response, "message", "frame-state getter failed")
                )
                result["ok"] = False
                result["sides"][side] = side_result
                continue
            side_result.update({
                "available": True,
                "tool_id": int(response.tool_id),
                "tool_pose": [float(value) for value in response.tool_pose],
                "user_frame_id": int(response.user_frame_id),
                "user_frame_pose": [float(value) for value in response.user_frame_pose],
                "installation_rpy": [float(value) for value in response.installation_rpy],
                "installation_quaternion_wxyz": [
                    float(value) for value in response.installation_quaternion
                ],
                "error": None,
            })
            result["sides"][side] = side_result
        return result

    def get_fk_pose(self, side_name):
        if side_name == "left":
            joint = self.left_joint
            client = self.left_get_fk
        else:
            joint = self.right_joint
            client = self.right_get_fk

        if joint is None:
            return None

        if not client.service_is_ready():
            return None

        req = GetFK.Request()
        req.joint = [float(x) for x in joint]

        resp = self.wait_future_result(client.call_async(req), timeout=0.4)
        if resp is None:
            return None

        pose = list(getattr(resp, "cartesian_pose", []) or [])
        if len(pose) < 6:
            return None

        return [float(x) for x in pose[:6]]


    def make_linear_move_request(self, pose, vel, acc, ref_joint=None):
        req = Move.Request()
        req.pose = [float(x) for x in pose]

        # สำคัญมาก:
        # Cartesian pose เดียวกันอาจมีหลาย IK solution
        # ถ้าไม่ส่ง ref_joint หุ่นอาจ flip joint ไปอีก posture ได้
        if ref_joint is not None:
            req.has_ref = True
            req.ref_joint = [float(x) for x in ref_joint]
        else:
            req.has_ref = False
            req.ref_joint = []

        req.mvvelo = float(vel)
        req.mvacc = float(acc)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        return req

    def direct_tcp_move(self, side="both", left_pose=None, right_pose=None, vel=None, acc=None):
        # SAFE TCP TARGET:
        # Do NOT use /linear_move directly.
        # Convert TCP target -> IK with current joint as ref_joint -> execute by joint_move.
        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        ok, msg = self.safe_state_ok(side)
        if not ok:
            return {"ok": False, "error": msg}

        joint_vel = float(vel if vel is not None else self.cfg["motion"].get("default_joint_vel", 0.12))
        joint_acc = float(acc if acc is not None else self.cfg["motion"].get("default_joint_acc", 0.20))

        pos_eps = float(self.cfg["motion"].get("tcp_noop_position_epsilon", 0.01))
        rot_eps = float(self.cfg["motion"].get("tcp_noop_rotation_epsilon", 0.001))

        def validate_pose(vals, label):
            if vals is None:
                return None
            if len(vals) != 6:
                return {"error": f"{label} must contain 6 values: X Y Z Rx Ry Rz"}
            try:
                return [float(v) for v in vals]
            except Exception as e:
                return {"error": f"{label} parse error: {e}"}

        def pose_delta(current, target):
            if current is None or target is None:
                return None
            dp = max(abs(float(target[i]) - float(current[i])) for i in range(3))
            dr = max(abs(float(target[i]) - float(current[i])) for i in range(3, 6))
            return dp, dr

        def solve_ik(side_name, pose, ref_joint):
            if side_name == "left":
                client = self.left_get_ik
            else:
                client = self.right_get_ik

            if ref_joint is None:
                return None, f"{side_name} joint feedback missing"

            if not client.service_is_ready():
                return None, f"{side_name} get_ik service not ready"

            req = GetIK.Request()
            req.ref_joint = [float(x) for x in ref_joint]
            req.cartesian_pose = [float(x) for x in pose]

            resp = self.wait_future_result(client.call_async(req), timeout=1.0)
            if resp is None:
                return None, f"{side_name} get_ik timeout/no response"

            joint = list(getattr(resp, "joint", []) or [])
            message = str(getattr(resp, "message", "") or "")

            if len(joint) < 6:
                return None, f"{side_name} IK failed: {message}"

            return [float(x) for x in joint[:6]], message

        left_target_pose = None
        right_target_pose = None

        if side in ("left", "both"):
            left_target_pose = validate_pose(left_pose, "left_pose")
            if isinstance(left_target_pose, dict):
                return {"ok": False, **left_target_pose}

        if side in ("right", "both"):
            right_target_pose = validate_pose(right_pose, "right_pose")
            if isinstance(right_target_pose, dict):
                return {"ok": False, **right_target_pose}

        left_current_tcp = self.get_fk_pose("left") if side in ("left", "both") else None
        right_current_tcp = self.get_fk_pose("right") if side in ("right", "both") else None

        left_delta = None
        right_delta = None
        left_joint_target = None
        right_joint_target = None
        ik_messages = {}

        if side in ("left", "both"):
            left_delta = pose_delta(left_current_tcp, left_target_pose)
            if left_delta is None:
                return {"ok": False, "error": "left TCP feedback/FK missing"}

            if left_delta[0] > pos_eps or left_delta[1] > rot_eps:
                left_joint_target, msg = solve_ik("left", left_target_pose, self.left_joint)
                ik_messages["left"] = msg
                if left_joint_target is None:
                    return {"ok": False, "error": msg}
            else:
                left_joint_target = self.left_joint

        if side in ("right", "both"):
            right_delta = pose_delta(right_current_tcp, right_target_pose)
            if right_delta is None:
                return {"ok": False, "error": "right TCP feedback/FK missing"}

            if right_delta[0] > pos_eps or right_delta[1] > rot_eps:
                right_joint_target, msg = solve_ik("right", right_target_pose, self.right_joint)
                ik_messages["right"] = msg
                if right_joint_target is None:
                    return {"ok": False, "error": msg}
            else:
                right_joint_target = self.right_joint

        left_send = side in ("left", "both") and left_delta is not None and (left_delta[0] > pos_eps or left_delta[1] > rot_eps)
        right_send = side in ("right", "both") and right_delta is not None and (right_delta[0] > pos_eps or right_delta[1] > rot_eps)

        if not left_send and not right_send:
            return {
                "ok": True,
                "accepted": False,
                "message": "TCP target is already at current pose; no move sent",
                "method": "get_ik_to_joint_move",
                "left_delta": left_delta,
                "right_delta": right_delta,
            }

        result = self.move_both_joint(
            left_target=left_joint_target,
            right_target=right_joint_target,
            side=side,
            vel=joint_vel,
            acc=joint_acc,
        )

        result["method"] = "get_ik_to_joint_move"
        result["ik_messages"] = ik_messages
        result["left_tcp_delta"] = left_delta
        result["right_tcp_delta"] = right_delta
        result["note"] = "TCP target solved by IK and executed using joint_move, not linear_move"

        return result

    def make_joint_move_request(self, joints, vel, acc):
        req = Move.Request()
        req.pose = [float(x) for x in joints]
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = float(vel)
        req.mvacc = float(acc)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        return req

    def make_execute_joint_trajectory_request(
        self,
        time_from_start_s,
        flattened_joint_positions_rad,
        start_time_unix_ns,
        trajectory_id,
        servo_filter_config=None,
        servo_step_num=1,
    ):
        req = ExecuteJointTrajectory.Request()
        req.time_from_start_s = [float(value) for value in time_from_start_s]
        req.joint_positions_rad_flat = [
            float(value) for value in flattened_joint_positions_rad
        ]
        req.start_time_unix_ns = int(start_time_unix_ns)
        req.trajectory_id = str(trajectory_id)
        filter_config = dict(
            servo_filter_config
            or {"mode": "LEGACY_FORESIGHT", "legacy_max_buf": 15, "legacy_kp": 0.03}
        )
        mode = str(filter_config.get("mode", "LEGACY_FORESIGHT")).upper()
        req.servo_filter_mode = {
            "NONE": 0,
            "LPF": 1,
            "NLF": 2,
            "LEGACY_FORESIGHT": 3,
        }.get(mode, 255)
        req.servo_filter_legacy_max_buf = int(
            filter_config.get("legacy_max_buf", 0)
        )
        req.servo_filter_legacy_kp = float(filter_config.get("legacy_kp", 0.0))
        req.servo_filter_lpf_cutoff_hz = float(
            filter_config.get("lpf_cutoff_hz", 0.0)
        )
        req.servo_filter_nlf_max_velocity_deg_s = float(
            filter_config.get("nlf_max_velocity_deg_s", 0.0)
        )
        req.servo_filter_nlf_max_acceleration_deg_s2 = float(
            filter_config.get("nlf_max_acceleration_deg_s2", 0.0)
        )
        req.servo_filter_nlf_max_jerk_deg_s3 = float(
            filter_config.get("nlf_max_jerk_deg_s3", 0.0)
        )
        req.servo_step_num = int(servo_step_num)
        return req

    def send_jog_once(self, cmd):
        side = cmd["side"]
        coord = cmd["coord"]
        axis = cmd["axis"]
        direction = cmd["direction"]
        speed = cmd["speed"]
        sync_mode = cmd["sync_mode"]

        ok, msg = self.safe_state_ok(side)
        if not ok:
            self.get_logger().error(f"Unsafe state during jog: {msg}")
            self.stop_motion("both")
            with self.active_lock:
                self.active_jog = None
            return

        left_idx, right_idx = self.compute_indices(
            side=side,
            coord=coord,
            axis=axis,
            direction=direction,
            sync_mode=sync_mode,
        )

        if side in ("left", "both"):
            self.left_jog.call_async(self.make_jog_request(coord, left_idx, speed))

        if side in ("right", "both"):
            self.right_jog.call_async(self.make_jog_request(coord, right_idx, speed))

    def jog_loop(self):
        while True:
            time.sleep(self.repeat_period)
            with self.active_lock:
                cmd = dict(self.active_jog) if self.active_jog else None

            if cmd is None:
                continue

            if time.time() - self.last_jog_time > self.deadman_timeout:
                self.get_logger().warn("Jog deadman timeout. Stop both.")
                self.stop_motion("both")
                with self.active_lock:
                    self.active_jog = None
                continue

            self.send_jog_once(cmd)

    def start_jog(self, req: JogStartRequest):
        if self.phase5_execution_coordinator.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ACTIVE"}
        if req.side not in ("left", "right", "both"):
            return {"ok": False, "error": "side must be left, right, or both"}
        if req.coord not in ("joint", "base", "tool"):
            return {"ok": False, "error": "coord must be joint, base, or tool"}
        if req.direction not in ("+", "-"):
            return {"ok": False, "error": "direction must be + or -"}
        if req.sync_mode not in ("raw", "same", "mirror"):
            return {"ok": False, "error": "sync_mode must be raw, same, or mirror"}

        ok, msg = self.safe_state_ok(req.side)
        if not ok:
            return {"ok": False, "error": msg}

        cmd = {
            "side": req.side,
            "coord": req.coord,
            "axis": req.axis.lower(),
            "direction": req.direction,
            "speed": float(req.speed),
            "sync_mode": req.sync_mode,
        }

        with self.active_lock:
            self.active_jog = cmd
            self.last_jog_time = time.time()

        self.send_jog_once(cmd)

        return {"ok": True, "active_jog": cmd}

    def heartbeat_jog(self):
        with self.active_lock:
            if self.active_jog is None:
                return {"ok": False, "error": "no active jog"}
            self.last_jog_time = time.time()
            return {"ok": True, "active_jog": self.active_jog}

    def stop_jog(self, side="both"):
        if self.phase5_execution_coordinator.is_active():
            return self.request_stop_all("both")
        with self.active_lock:
            self.active_jog = None
            self.active_motion = None
        self.stop_motion(side)
        return {"ok": True, "stopped": side}

    def stop_motion(self, side="both"):
        req = Empty.Request()
        if side in ("left", "both"):
            self.left_stop.call_async(req)
        if side in ("right", "both"):
            self.right_stop.call_async(req)

    def move_both_joint(self, left_target=None, right_target=None, side="both", vel=None, acc=None):
        # Dual-arm synchronized joint move:
        # Treat incoming vel/acc as maximum values, then scale left/right velocity
        # based on each arm's joint distance so both arms finish closer together.
        if self.phase5_execution_coordinator.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ACTIVE"}
        ok, msg = self.safe_state_ok(side)
        if not ok:
            return {"ok": False, "error": msg}

        max_vel = float(vel if vel is not None else self.cfg["motion"].get("default_joint_vel", 0.18))
        max_acc = float(acc if acc is not None else self.cfg["motion"].get("default_joint_acc", 0.30))

        sync_enabled = bool(self.cfg["motion"].get("sync_joint_move_enabled", True))
        min_duration = float(self.cfg["motion"].get("sync_min_duration", 1.0))
        min_vel = float(self.cfg["motion"].get("sync_min_joint_vel", 0.005))
        min_acc = float(self.cfg["motion"].get("sync_min_joint_acc", 0.02))
        eps = float(self.cfg["motion"].get("sync_delta_epsilon", 0.0003))

        if side in ("left", "both") and left_target is None:
            return {"ok": False, "error": "left target missing"}
        if side in ("right", "both") and right_target is None:
            return {"ok": False, "error": "right target missing"}

        if side in ("left", "both") and self.left_joint is None:
            return {"ok": False, "error": "left joint feedback missing"}
        if side in ("right", "both") and self.right_joint is None:
            return {"ok": False, "error": "right joint feedback missing"}

        def max_abs_delta(current, target):
            return max(abs(float(t) - float(c)) for c, t in zip(current, target))

        left_delta = 0.0
        right_delta = 0.0

        if side in ("left", "both"):
            left_delta = max_abs_delta(self.left_joint, left_target)

        if side in ("right", "both"):
            right_delta = max_abs_delta(self.right_joint, right_target)

        left_send = side in ("left", "both") and left_delta > eps
        right_send = side in ("right", "both") and right_delta > eps

        if not left_send and not right_send:
            return {
                "ok": True,
                "accepted": False,
                "message": "already at target within epsilon",
                "left_delta": left_delta,
                "right_delta": right_delta,
            }

        if sync_enabled and side == "both":
            max_delta = max(left_delta, right_delta)
            target_duration = max(min_duration, max_delta / max(max_vel, 1e-9))

            left_vel = left_delta / target_duration if left_send else 0.0
            right_vel = right_delta / target_duration if right_send else 0.0

            if left_send:
                left_vel = min(max_vel, max(min_vel, left_vel))
            if right_send:
                right_vel = min(max_vel, max(min_vel, right_vel))

            # Scale acceleration roughly with velocity ratio.
            # This helps the shorter-motion arm avoid snapping too aggressively.
            left_acc = max_acc * (left_vel / max(max_vel, 1e-9)) if left_send else 0.0
            right_acc = max_acc * (right_vel / max(max_vel, 1e-9)) if right_send else 0.0

            if left_send:
                left_acc = min(max_acc, max(min_acc, left_acc))
            if right_send:
                right_acc = min(max_acc, max(min_acc, right_acc))

        else:
            target_duration = None
            left_vel = max_vel if left_send else 0.0
            right_vel = max_vel if right_send else 0.0
            left_acc = max_acc if left_send else 0.0
            right_acc = max_acc if right_send else 0.0

        self.motion_cancel_requested = False
        self.active_motion = {
            "side": side,
            "type": "sync_joint_move" if sync_enabled and side == "both" else "single_joint_move",
            "status": "sent",
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "sync_enabled": sync_enabled,
            "max_vel": max_vel,
            "max_acc": max_acc,
            "target_duration": target_duration,
            "left_delta": left_delta,
            "right_delta": right_delta,
            "left_vel": left_vel,
            "right_vel": right_vel,
            "left_acc": left_acc,
            "right_acc": right_acc,
            "left_send": left_send,
            "right_send": right_send,
            "sent_at_unix_s": time.time(),
        }

        if left_send:
            self.left_move.call_async(
                self.make_joint_move_request(left_target, left_vel, left_acc)
            )

        if right_send:
            self.right_move.call_async(
                self.make_joint_move_request(right_target, right_vel, right_acc)
            )

        return {
            "ok": True,
            "accepted": True,
            "message": "sync joint_move sent",
            "side": side,
            "sync_enabled": sync_enabled,
            "target_duration": target_duration,
            "left_delta": left_delta,
            "right_delta": right_delta,
            "left_vel": left_vel,
            "right_vel": right_vel,
            "left_acc": left_acc,
            "right_acc": right_acc,
            "left_send": left_send,
            "right_send": right_send,
        }

    def request_stop_all(self, side="both"):
        # Global stop latch:
        # กด STOP ครั้งเดียวต้องหยุด motion ปัจจุบัน + ยกเลิก sequence ทั้งหมด
        self.stop_generation = getattr(self, "stop_generation", 0) + 1
        self.phase5_execution_coordinator.on_stop(self.stop_generation)
        self.motion_cancel_requested = True
        self.sequence_cancel_requested = True

        with self.active_lock:
            self.active_jog = None

            if self.active_motion is not None:
                self.active_motion["status"] = "stop_requested"
                self.active_motion["stop_generation"] = self.stop_generation

            if self.active_sequence is not None:
                self.active_sequence["status"] = "stop_requested"
                self.active_sequence["stop_generation"] = self.stop_generation
                self.active_sequence["reason"] = "STOP BOTH pressed"

        self.stop_motion("both")

        return {
            "ok": True,
            "stop_requested": side,
            "stop_generation": self.stop_generation,
            "message": "STOP BOTH: current motion and full sequence cancelled",
        }

    def home(self, side="both", vel=None, acc=None):
        if self.phase5_execution_coordinator.is_active():
            return {
                "ok": False,
                "error": "PHASE5_EXECUTION_ACTIVE_HOME_BLOCKED_USE_STOP_FIRST",
            }
        home_cfg = self.cfg.get("home", {})
        left_home = home_cfg.get("left")
        right_home = home_cfg.get("right")

        return self.move_both_joint(
            left_target=left_home,
            right_target=right_home,
            side=side,
            vel=vel,
            acc=acc,
        )

    def load_waypoints(self):
        if not self.waypoint_path.exists():
            return {}
        try:
            return json.loads(self.waypoint_path.read_text())
        except Exception:
            return {}

    def save_waypoints(self, data):
        self.waypoint_path.write_text(json.dumps(data, indent=2))

    def save_waypoint(self, name, side="both"):
        name = name.strip()
        if not name:
            return {"ok": False, "error": "waypoint name is empty"}

        if self.left_joint is None or self.right_joint is None:
            return {"ok": False, "error": "joint feedback missing"}

        data = self.load_waypoints()
        data[name] = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "side": side,
        }

        if side in ("left", "both"):
            data[name]["left"] = self.left_joint
        if side in ("right", "both"):
            data[name]["right"] = self.right_joint

        self.save_waypoints(data)
        return {"ok": True, "name": name, "waypoint": data[name]}

    def direct_joint_move(self, side="both", left_deg=None, right_deg=None, vel=None, acc=None):
        # Direct joint input from Web UI.
        # UI uses degrees for readability, backend converts to radians.
        import math

        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        def deg_to_rad(vals, label):
            if vals is None:
                return None
            if len(vals) != 6:
                return {"error": f"{label} must contain 6 joint values"}
            try:
                return [float(v) * math.pi / 180.0 for v in vals]
            except Exception as e:
                return {"error": f"{label} parse error: {e}"}

        left_target = None
        right_target = None

        if side in ("left", "both"):
            left_target = deg_to_rad(left_deg, "left_deg")
            if isinstance(left_target, dict):
                return {"ok": False, **left_target}

        if side in ("right", "both"):
            right_target = deg_to_rad(right_deg, "right_deg")
            if isinstance(right_target, dict):
                return {"ok": False, **right_target}

        return self.move_both_joint(
            left_target=left_target,
            right_target=right_target,
            side=side,
            vel=vel,
            acc=acc,
        )


    def delete_waypoint(self, name):
        name = (name or "").strip()
        if not name:
            return {"ok": False, "error": "waypoint name is empty"}

        data = self.load_waypoints()
        if name not in data:
            return {"ok": False, "error": f"waypoint not found: {name}"}

        del data[name]

        wp_file = Path.home() / "jaka_ws/dual_arm_app/tasks/waypoints.json"
        wp_file.parent.mkdir(parents=True, exist_ok=True)
        wp_file.write_text(json.dumps(data, indent=2, ensure_ascii=False))

        return {
            "ok": True,
            "deleted": name,
            "remaining": len(data),
        }


    def run_waypoint(self, name, side="both", vel=None, acc=None):
        data = self.load_waypoints()
        if name not in data:
            return {"ok": False, "error": f"waypoint not found: {name}"}

        wp = data[name]
        left = wp.get("left")
        right = wp.get("right")

        return self.move_both_joint(
            left_target=left,
            right_target=right,
            side=side,
            vel=vel,
            acc=acc,
        )

    def is_motion_idle(self, side="both"):
        selected = []
        if side in ("left", "both"):
            selected.append(self.left_state)
        if side in ("right", "both"):
            selected.append(self.right_state)

        for st in selected:
            if st is None:
                return False
            if int(st.motion_state) != 0:
                return False
        return True

    def wait_motion_done(self, side="both", timeout=120.0, stop_generation_at_start=None):
        start = time.time()

        def should_cancel():
            if self.sequence_cancel_requested:
                return True
            if self.motion_cancel_requested:
                return True
            if stop_generation_at_start is not None:
                if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                    return True
            return False

        def motion_state_of(state):
            if state is None:
                return None

            if isinstance(state, dict):
                return state.get("motion_state", None)

            try:
                return getattr(state, "motion_state")
            except Exception:
                return None

        def is_moving_state(v):
            # JAKA motion_state = 0 means idle / not moving
            if v is None:
                return False
            try:
                return int(v) != 0
            except Exception:
                return False

        active = self.active_motion if isinstance(self.active_motion, dict) else {}

        track_left = side in ("left", "both")
        track_right = side in ("right", "both")

        # ถ้า move_both_joint ระบุว่าแขนนั้นไม่ได้ถูกส่งคำสั่ง ก็ไม่ต้อง track
        if side == "both":
            if active.get("left_send") is False:
                track_left = False
            if active.get("right_send") is False:
                track_right = False

        left_started_at = None
        right_started_at = None
        left_finished_at = None
        right_finished_at = None

        # ถ้าไม่ได้ track ให้ถือว่า finished ตั้งแต่ start
        if not track_left:
            left_finished_at = start
        if not track_right:
            right_finished_at = start

        while time.time() - start < timeout:
            now = time.time()

            if should_cancel():
                self.stop_motion("both")
                self._update_motion_finish_log(
                    start,
                    left_started_at,
                    right_started_at,
                    left_finished_at,
                    right_finished_at,
                    status="cancelled",
                )
                return False, "cancelled"

            left_state = motion_state_of(self.left_state)
            right_state = motion_state_of(self.right_state)

            left_moving = is_moving_state(left_state)
            right_moving = is_moving_state(right_state)

            if track_left:
                if left_started_at is None and left_moving:
                    left_started_at = now

                if left_started_at is not None and not left_moving and left_finished_at is None:
                    left_finished_at = now

                # motion สั้นมากจนไม่ทันเห็น moving state
                if left_started_at is None and left_finished_at is None and now - start > 1.0 and not left_moving:
                    left_finished_at = now

            if track_right:
                if right_started_at is None and right_moving:
                    right_started_at = now

                if right_started_at is not None and not right_moving and right_finished_at is None:
                    right_finished_at = now

                # motion สั้นมากจนไม่ทันเห็น moving state
                if right_started_at is None and right_finished_at is None and now - start > 1.0 and not right_moving:
                    right_finished_at = now

            if left_finished_at is not None and right_finished_at is not None:
                self._update_motion_finish_log(
                    start,
                    left_started_at,
                    right_started_at,
                    left_finished_at,
                    right_finished_at,
                    status="done",
                )
                return True, "done"

            time.sleep(0.05)

        self._update_motion_finish_log(
            start,
            left_started_at,
            right_started_at,
            left_finished_at,
            right_finished_at,
            status="timeout",
        )
        return False, "timeout"


    def _update_motion_finish_log(
        self,
        start,
        left_started_at,
        right_started_at,
        left_finished_at,
        right_finished_at,
        status="unknown",
    ):
        def rel(t):
            if t is None:
                return None
            return round(float(t - start), 3)

        finish_delta = None
        if left_finished_at is not None and right_finished_at is not None:
            finish_delta = round(abs(left_finished_at - right_finished_at), 3)

        log = {
            "status": status,
            "left_started_sec": rel(left_started_at),
            "right_started_sec": rel(right_started_at),
            "left_finished_sec": rel(left_finished_at),
            "right_finished_sec": rel(right_finished_at),
            "finish_delta_sec": finish_delta,
        }

        if isinstance(self.active_motion, dict):
            self.active_motion["finish_log"] = log
            if status in ("done", "cancelled", "timeout"):
                self.active_motion["finish_status"] = status

        return log

    def list_center_paths(self):
        try:
            return self.center_path_store.list()
        except Exception as error:
            return {"ok": False, "error": str(error), "paths": []}

    def save_center_path(self, name, path_payload, overwrite=False):
        try:
            return self.center_path_store.save(name, path_payload, overwrite=bool(overwrite))
        except Exception as error:
            return {"ok": False, "error": str(error)}

    def load_center_path(self, name):
        try:
            return self.center_path_store.load(name)
        except Exception as error:
            return {"ok": False, "error": str(error)}

    def rename_center_path(self, name, new_name):
        try:
            return self.center_path_store.rename(name, new_name)
        except Exception as error:
            return {"ok": False, "error": str(error)}

    def delete_center_path(self, name):
        try:
            return self.center_path_store.delete(name)
        except Exception as error:
            return {"ok": False, "error": str(error)}


    def _programs_dir(self):
        d = Path(__file__).resolve().parents[1] / "programs"
        d.mkdir(parents=True, exist_ok=True)
        return d


    def _safe_program_filename(self, name):
        raw = str(name or "").strip()
        raw = raw.replace(" ", "_")
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw).strip("._")

        if not safe:
            raise ValueError("program name is empty")

        if not safe.endswith(".json"):
            safe += ".json"

        # กัน path traversal
        if "/" in safe or "\\" in safe or ".." in safe:
            raise ValueError("invalid program name")

        return safe


    def _program_step_to_dict(self, step, default_side="both", default_vel=None, default_acc=None):
        def get(key, default=None):
            if isinstance(step, dict):
                return step.get(key, default)
            return getattr(step, key, default)

        name = get("name", None)
        if not name:
            raise ValueError("program step missing waypoint name")

        side = str(get("side", default_side) or default_side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            raise ValueError(f"invalid step side: {side}")

        vel = get("vel", default_vel)
        acc = get("acc", default_acc)
        delay = get("delay", 0.0)

        return {
            "name": str(name),
            "side": side,
            "vel": float(vel) if vel is not None else None,
            "acc": float(acc) if acc is not None else None,
            "delay": max(0.0, float(delay or 0.0)),
        }


    def normalize_program_steps(self, steps, default_side="both", default_vel=None, default_acc=None):
        return [
            self._program_step_to_dict(
                step,
                default_side=default_side,
                default_vel=default_vel,
                default_acc=default_acc,
            )
            for step in steps
        ]


    def save_program_file(self, name, steps):
        try:
            filename = self._safe_program_filename(name)
            path = self._programs_dir() / filename

            normalized_steps = self.normalize_program_steps(steps)

            payload = {
                "name": filename[:-5],
                "file": filename,
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "steps": normalized_steps,
            }

            path.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )

            return {
                "ok": True,
                "message": "program saved",
                "file": filename,
                "path": str(path),
                "steps": normalized_steps,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def list_program_files(self):
        try:
            d = self._programs_dir()
            items = []

            for path in sorted(d.glob("*.json")):
                try:
                    stat = path.stat()
                    items.append({
                        "name": path.stem,
                        "file": path.name,
                        "size": stat.st_size,
                        "modified_at": time.strftime(
                            "%Y-%m-%d %H:%M:%S",
                            time.localtime(stat.st_mtime),
                        ),
                    })
                except Exception:
                    pass

            return {"ok": True, "programs": items}

        except Exception as e:
            return {"ok": False, "error": str(e), "programs": []}


    def load_program_file(self, name):
        try:
            filename = self._safe_program_filename(name)
            path = self._programs_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"program not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))
            steps = self.normalize_program_steps(payload.get("steps", []))

            return {
                "ok": True,
                "message": "program loaded",
                "name": payload.get("name", path.stem),
                "file": filename,
                "steps": steps,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def delete_program_file(self, name):
        try:
            filename = self._safe_program_filename(name)
            path = self._programs_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"program not found: {filename}"}

            path.unlink()
            return {"ok": True, "message": "program deleted", "file": filename}

        except Exception as e:
            return {"ok": False, "error": str(e)}



    def run_program(self, steps, side="both", vel=None, acc=None, loop_mode="once", loop_count=1):
        if self.phase5_execution_coordinator.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ACTIVE"}
        if not steps:
            return {"ok": False, "error": "program is empty"}

        loop_mode = str(loop_mode or "once").lower().strip()
        if loop_mode not in ("once", "count", "forever"):
            return {"ok": False, "error": f"invalid loop mode: {loop_mode}"}

        loop_count = int(loop_count if loop_count is not None else 1)
        if loop_mode == "count" and loop_count < 1:
            return {"ok": False, "error": "loop_count must be at least 1 for count mode"}
        if loop_mode == "once":
            loop_count = 1

        if self.active_sequence is not None and self.active_sequence.get("status") == "running":
            return {
                "ok": False,
                "error": "sequence/program already running",
                "active_sequence": self.active_sequence,
            }

        def step_get(step, key, default=None):
            if isinstance(step, dict):
                return step.get(key, default)
            return getattr(step, key, default)

        normalized_steps = []
        for i, step in enumerate(steps):
            name = step_get(step, "name", None)
            if not name:
                return {"ok": False, "error": f"step {i} missing waypoint name"}

            step_side = (step_get(step, "side", None) or side or "both").lower().strip()
            if step_side not in ("left", "right", "both"):
                return {"ok": False, "error": f"step {i} invalid side: {step_side}"}

            step_vel = step_get(step, "vel", None)
            step_acc = step_get(step, "acc", None)
            step_delay = step_get(step, "delay", 0.0)

            step_vel = float(step_vel if step_vel is not None else vel) if (step_vel is not None or vel is not None) else None
            step_acc = float(step_acc if step_acc is not None else acc) if (step_acc is not None or acc is not None) else None
            step_delay = max(0.0, float(step_delay or 0.0))

            normalized_steps.append({
                "name": str(name),
                "side": step_side,
                "vel": step_vel,
                "acc": step_acc,
                "delay": step_delay,
            })

        self.sequence_cancel_requested = False
        self.motion_cancel_requested = False
        stop_generation_at_start = getattr(self, "stop_generation", 0)
        status_loop_count = None if loop_mode == "forever" else loop_count

        self.active_sequence = {
            "type": "program",
            "status": "running",
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "current_index": 0,
            "current_name": None,
            "steps": normalized_steps,
            "side": side,
            "default_vel": vel,
            "default_acc": acc,
            "stop_generation": stop_generation_at_start,
            "loop_mode": loop_mode,
            "loop_count": status_loop_count,
            "current_cycle": 1,
            "completed_cycles": 0,
            "total_steps": len(normalized_steps),
        }

        th = threading.Thread(
            target=self._program_thread,
            args=(normalized_steps, stop_generation_at_start, loop_mode, loop_count),
            daemon=True,
        )
        th.start()

        return {
            "ok": True,
            "accepted": True,
            "message": "program started",
            "steps": normalized_steps,
            "stop_generation": stop_generation_at_start,
            "loop_mode": loop_mode,
            "loop_count": status_loop_count,
        }


    def _program_thread(self, steps, stop_generation_at_start=0, loop_mode="once", loop_count=1):
        def should_cancel():
            if self.sequence_cancel_requested:
                return True
            if self.motion_cancel_requested:
                return True
            if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                return True
            return False

        def update_state(status, **fields):
            if self.active_sequence is None:
                self.active_sequence = {}
            self.active_sequence["type"] = "program"
            self.active_sequence["status"] = status
            self.active_sequence.update(fields)

        try:
            requested_cycles = None if loop_mode == "forever" else loop_count
            completed_cycles = 0

            while requested_cycles is None or completed_cycles < requested_cycles:
                current_cycle = completed_cycles + 1
                update_state("running", current_cycle=current_cycle)

                for i, step in enumerate(steps):
                    if should_cancel():
                        update_state(
                            "cancelled",
                            cancelled_at_index=i,
                            reason="cancel requested before step",
                        )
                        return

                    name = step["name"]
                    step_side = step.get("side", "both")
                    step_vel = step.get("vel", None)
                    step_acc = step.get("acc", None)
                    step_delay = float(step.get("delay", 0.0) or 0.0)

                    update_state(
                        "running",
                        current_index=i,
                        current_name=name,
                        current_step=step,
                        progress=f"{i + 1}/{len(steps)}",
                    )

                    result = self.run_waypoint(
                        name,
                        side=step_side,
                        vel=step_vel,
                        acc=step_acc,
                    )

                    if should_cancel():
                        update_state(
                            "cancelled",
                            cancelled_at_index=i,
                            reason="cancel requested after waypoint command",
                        )
                        return

                    if not result.get("ok", False):
                        update_state(
                            "error",
                            failed_index=i,
                            failed_name=name,
                            result=result,
                        )
                        return

                    done, msg = self.wait_motion_done(
                        side=step_side,
                        timeout=120.0,
                        stop_generation_at_start=stop_generation_at_start,
                    )

                    if not done:
                        update_state(
                            "cancelled" if msg == "cancelled" else "error",
                            failed_index=i,
                            failed_name=name,
                            reason=msg,
                        )
                        return

                    if should_cancel():
                        update_state(
                            "cancelled",
                            cancelled_at_index=i,
                            reason="cancel requested after current motion",
                        )
                        return

                    if step_delay > 0:
                        update_state("delay", delay_sec=step_delay)

                        delay_start = time.time()
                        while time.time() - delay_start < step_delay:
                            if should_cancel():
                                update_state(
                                    "cancelled",
                                    cancelled_at_index=i,
                                    reason="cancel requested during delay",
                                )
                                return
                            time.sleep(0.05)

                completed_cycles += 1
                update_state("running", completed_cycles=completed_cycles)

                if requested_cycles is not None and completed_cycles >= requested_cycles:
                    break

                if should_cancel():
                    update_state("cancelled", reason="cancel requested between cycles")
                    return

            update_state(
                "done",
                finished_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                count=len(steps),
            )

        except Exception as e:
            update_state("exception", error=str(e))


    def run_sequence(self, names, side="both", vel=None, acc=None):
        if self.phase5_execution_coordinator.is_active():
            return {"ok": False, "error": "PHASE5_EXECUTION_ACTIVE"}
        if not names:
            return {"ok": False, "error": "sequence is empty"}

        if self.active_sequence is not None and self.active_sequence.get("status") == "running":
            return {
                "ok": False,
                "error": "sequence already running",
                "active_sequence": self.active_sequence,
            }

        data = self.load_waypoints()
        missing = [n for n in names if n not in data]
        if missing:
            return {"ok": False, "error": f"missing waypoints: {missing}"}

        self.sequence_cancel_requested = False
        self.motion_cancel_requested = False

        stop_generation_at_start = getattr(self, "stop_generation", 0)

        self.active_sequence = {
            "status": "running",
            "names": names,
            "current_index": 0,
            "current_name": None,
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "side": side,
            "vel": vel,
            "acc": acc,
            "stop_generation_at_start": stop_generation_at_start,
        }

        th = threading.Thread(
            target=self._sequence_thread,
            args=(names, side, vel, acc, stop_generation_at_start),
            daemon=True,
        )
        th.start()

        return {
            "ok": True,
            "accepted": True,
            "message": "sequence started",
            "names": names,
            "side": side,
            "vel": vel,
            "acc": acc,
            "stop_generation_at_start": stop_generation_at_start,
        }

    def _sequence_thread(self, names, side="both", vel=None, acc=None, stop_generation_at_start=0):
        try:
            def should_cancel():
                return (
                    self.sequence_cancel_requested
                    or self.motion_cancel_requested
                    or getattr(self, "stop_generation", 0) != stop_generation_at_start
                )

            for i, name in enumerate(names):
                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested before step",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                if self.active_sequence is not None:
                    self.active_sequence["status"] = "running"
                    self.active_sequence["current_index"] = i
                    self.active_sequence["current_name"] = name

                result = self.run_waypoint(name, side=side, vel=vel, acc=acc)

                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested after sending waypoint",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                if not result.get("ok", False):
                    self.active_sequence = {
                        "status": "error",
                        "error": result,
                        "failed_at": name,
                        "index": i,
                    }
                    return

                ok, msg = self.wait_motion_done(
                    side=side,
                    timeout=120.0,
                    stop_generation_at_start=stop_generation_at_start,
                )

                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested after current motion",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                if not ok:
                    self.active_sequence = {
                        "status": "cancelled" if msg == "cancelled" else "error",
                        "message": msg,
                        "at": name,
                        "index": i,
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                time.sleep(0.15)

                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested before next step",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

            self.active_sequence = {
                "status": "done",
                "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "names": names,
            }

        except Exception as e:
            self.active_sequence = {
                "status": "error",
                "error": str(e),
            }



def load_config():
    p = Path(__file__).resolve().parents[1] / "config/robots.yaml"
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")

    cfg = yaml.safe_load(p.read_text())

    cfg.setdefault("motion", {})
    cfg["motion"].setdefault("default_joint_vel", 0.18)
    cfg["motion"].setdefault("default_joint_acc", 0.30)

    cfg["motion"].setdefault("left_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])

    cfg["motion"].setdefault("left_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])

    cfg["motion"].setdefault("left_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

    cfg.setdefault("home", {})
    cfg.setdefault("left", {})
    cfg.setdefault("right", {})

    return cfg




class DirectTcpMoveRequest(BaseModel):
    side: str = "both"
    left_pose: Optional[List[float]] = None
    right_pose: Optional[List[float]] = None
    vel: Optional[float] = None
    acc: Optional[float] = None

class DirectJointMoveRequest(BaseModel):
    side: str = "both"
    left_deg: Optional[List[float]] = None
    right_deg: Optional[List[float]] = None
    vel: Optional[float] = None
    acc: Optional[float] = None


class WaypointDeleteRequest(BaseModel):
    name: str


# ---------------------------------------------------------------------
# FastAPI app bootstrap
# ---------------------------------------------------------------------

import threading
import rclpy
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path

# Prevent two backend processes from creating the same ROS node name.
# The lock file remains in the workspace; it is never deleted. The OS releases
# the advisory lock automatically when the owning process exits.
import builtins
import fcntl

_BACKEND_INSTANCE_LOCK_PATH = Path(__file__).resolve().parent / ".dual_jaka_web_backend.instance.lock"
_PROCESS_LOCK_ATTRIBUTE = "_dual_jaka_web_backend_instance_lock_stream"
if hasattr(builtins, _PROCESS_LOCK_ATTRIBUTE):
    # Test harnesses may import this module repeatedly inside one Python process.
    # Reuse that process's existing lock ownership rather than self-conflicting.
    _BACKEND_INSTANCE_LOCK_STREAM = getattr(builtins, _PROCESS_LOCK_ATTRIBUTE)
else:
    _BACKEND_INSTANCE_LOCK_STREAM = _BACKEND_INSTANCE_LOCK_PATH.open("a+", encoding="utf-8")
    try:
        fcntl.flock(
            _BACKEND_INSTANCE_LOCK_STREAM.fileno(),
            fcntl.LOCK_EX | fcntl.LOCK_NB,
        )
    except BlockingIOError as error:
        _BACKEND_INSTANCE_LOCK_STREAM.close()
        raise RuntimeError(
            "Another dual_jaka_web_backend process already owns the ROS backend instance lock"
        ) from error
    setattr(builtins, _PROCESS_LOCK_ATTRIBUTE, _BACKEND_INSTANCE_LOCK_STREAM)

if not rclpy.ok():
    rclpy.init()

node = DualJakaWebNode(load_config())

spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
spin_thread.start()

app = FastAPI(title="Dual JAKA A12 Web Backend")

app.include_router(system_control_router(node))

_BACKEND_DIR = Path(__file__).resolve().parent
_DUAL_ARM_APP_DIR = _BACKEND_DIR.parent
_REPOSITORY_ROOT = _DUAL_ARM_APP_DIR.parent
_WEB_DIR = _DUAL_ARM_APP_DIR / "web"
_DIGITAL_TWIN_ASSETS_DIR = _WEB_DIR / "assets"
_A12_MESH_DIR = (
    _REPOSITORY_ROOT
    / "src/jaka_ros2/src/jaka_description/meshes/jaka_a12_meshes"
)

# Static-only Digital Twin resources. These mounts do not initialize clients or
# add any robot command surface.
app.mount(
    "/digital-twin/assets",
    StaticFiles(directory=str(_DIGITAL_TWIN_ASSETS_DIR), check_dir=True),
    name="digital-twin-assets",
)
app.mount(
    "/digital-twin/meshes",
    StaticFiles(directory=str(_A12_MESH_DIR), check_dir=True),
    name="digital-twin-meshes",
)
app.mount(
    "/web-assets",
    StaticFiles(directory=str(_WEB_DIR), check_dir=True),
    name="web-assets",
)


# D33.3B Backend Robot Activity State
# Tracks whether a web/API motion command is currently being handled by backend.
import time as _d33_activity_time
from threading import Lock as _D33ActivityLock

_D33_ACTIVITY_LOCK = _D33ActivityLock()
_D33_ACTIVE_COMMANDS = 0
# D33.3C Backend Settling Hold
_D33_SETTLE_HOLD_SEC = 3.0
_D33_HOLD_UNTIL = 0.0
_D33_LAST_ACTIVITY = {
    "state": "idle",
    "active": False,
    "active_commands": 0,
    "last_path": None,
    "last_started_at": None,
    "last_finished_at": None,
    "last_duration_sec": None,
    "last_error": None,
}

_D33_MOTION_EXACT_PATHS = {
    "/api/digital-twin/phase5/execute",
    "/api/digital-twin/phase5/move-to-initial",
}

_D33_MOTION_KEYWORDS = (
    "/home",
    "/home_both",
    "/jog",
    "/stop",
    "/run",
    "/program/run",
    "/joint_move",
    "/tcp_move",
    "/direct_move",
    "/direct_joint",
    "/move_both",
)

_D33_NO_MOTION_PATHS = (
    "/api/experiments",
    "/api/status",
    "/api/robot/activity",
    "/api/waypoints",
    "/api/program/list",
    "/api/program/load",
    "/api/digital-twin/validate-trajectory",
    "/api/digital-twin/plan-object-global",
    "/api/digital-twin/center-path",
    "/api/digital-twin/planning-start-state",
    "/api/digital-twin/world-calibration",
    "/api/digital-twin/phase5/execution-artifact",
    "/api/digital-twin/phase5/execution-transport",
    "/api/digital-twin/phase5/execution",
    "/api/digital-twin/phase5/prepare",
    "/api/digital-twin/phase5/replan-from-current",
    "/api/digital-twin/grasp-configuration",
    "/api/digital-twin/grasp-configuration/lock",
    "/api/digital-twin/grasp-configuration/unlock",
    "/api/digital-twin/validate-phase4-trajectory",
    "/api/digital-twin/validate-phase4-collision",
    "/api/digital-twin/validate-phase4",
)

def _d33_is_motion_command(path: str, method: str) -> bool:
    method = (method or "GET").upper()
    path = str(path or "")

    if method == "GET":
        return False

    for no_motion in _D33_NO_MOTION_PATHS:
        if path.startswith(no_motion):
            return False

    if path in _D33_MOTION_EXACT_PATHS:
        return True

    return any(k in path for k in _D33_MOTION_KEYWORDS)

def _d33_activity_start(path: str):
    global _D33_ACTIVE_COMMANDS
    now = _d33_activity_time.time()
    with _D33_ACTIVITY_LOCK:
        _D33_ACTIVE_COMMANDS += 1
        _D33_LAST_ACTIVITY.update({
            "state": "running",
            "active": True,
            "active_commands": _D33_ACTIVE_COMMANDS,
            "last_path": path,
            "last_started_at": now,
            "last_finished_at": None,
            "last_duration_sec": None,
            "last_error": None,
        })
    return now

def _d33_activity_finish(path: str, started_at: float, error: str = None):
    global _D33_ACTIVE_COMMANDS, _D33_HOLD_UNTIL
    now = _d33_activity_time.time()
    duration = now - started_at if started_at else None

    with _D33_ACTIVITY_LOCK:
        _D33_ACTIVE_COMMANDS = max(0, _D33_ACTIVE_COMMANDS - 1)

        hold_sec = 0.0
        if error is None:
            low_path = str(path or "").lower()

            # Do not keep RUNNING after an explicit stop command.
            if "/stop" not in low_path:
                hold_sec = _D33_SETTLE_HOLD_SEC
                _D33_HOLD_UNTIL = max(_D33_HOLD_UNTIL, now + hold_sec)

        if error:
            state = "error"
        elif _D33_ACTIVE_COMMANDS > 0:
            state = "running"
        elif hold_sec > 0:
            state = "settling"
        else:
            state = "idle"

        _D33_LAST_ACTIVITY.update({
            "state": state,
            "active": (_D33_ACTIVE_COMMANDS > 0) or (state == "settling"),
            "active_commands": _D33_ACTIVE_COMMANDS,
            "last_path": path,
            "last_finished_at": now,
            "last_duration_sec": duration,
            "last_error": error,
            "settling_until": _D33_HOLD_UNTIL,
            "settling_hold_sec": hold_sec,
        })


def _d33_activity_snapshot():
    now = _d33_activity_time.time()
    with _D33_ACTIVITY_LOCK:
        data = dict(_D33_LAST_ACTIVITY)

        settling_until = float(data.get("settling_until") or _D33_HOLD_UNTIL or 0.0)
        settling_remaining = max(0.0, settling_until - now)

        if _D33_ACTIVE_COMMANDS > 0:
            data["state"] = "running"
            data["active"] = True
            data["active_commands"] = _D33_ACTIVE_COMMANDS
        elif settling_remaining > 0 and data.get("state") != "error":
            data["state"] = "settling"
            data["active"] = True
            data["active_commands"] = 0
            data["settling_remaining_sec"] = settling_remaining
        elif data.get("state") != "error":
            data["state"] = "idle"
            data["active"] = False
            data["active_commands"] = 0
            data["settling_remaining_sec"] = 0.0

        data["now"] = now
        return data


@app.middleware("http")
async def d33_robot_activity_middleware(request, call_next):
    path = str(request.url.path)
    motion = _d33_is_motion_command(path, request.method)
    started_at = None

    if motion:
        started_at = _d33_activity_start(path)

    try:
        response = await call_next(request)
        if motion and response.status_code >= 400:
            _d33_activity_finish(path, started_at, error=f"HTTP {response.status_code}")
        elif motion:
            _d33_activity_finish(path, started_at, error=None)
        return response
    except Exception as exc:
        if motion:
            _d33_activity_finish(path, started_at, error=str(exc))
        raise

@app.get("/api/robot/activity")
async def api_robot_activity():
    return _d33_activity_snapshot()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
def index():
    return FileResponse(str(_WEB_DIR / "index.html"))


@app.get("/api/status")
def api_status():
    return node.status()


@app.get("/api/digital-twin/joints")
def api_digital_twin_joints():
    return node.digital_twin_joint_status()


@app.get("/api/digital-twin/robot-status")
def api_digital_twin_robot_status():
    return node.digital_twin_robot_status()


@app.get("/api/digital-twin/tcp")
def api_digital_twin_tcp():
    return node.digital_twin_tcp_status()


@app.get("/api/digital-twin/frame-state")
def api_digital_twin_frame_state():
    """Read active controller Tool/User/Mounting state; no motion or setters."""
    return node.digital_twin_frame_state()


@app.get("/api/digital-twin/planning-start-state")
def api_digital_twin_planning_start_state():
    return node.digital_twin_planning_start_state()


@app.get("/api/digital-twin/world-calibration")
def api_digital_twin_world_calibration():
    """Inspect canonical model alignment without accessing either robot."""
    return {"ok": True, "calibration": world_calibration_public_payload()}


@app.get("/api/digital-twin/phase5/execution-artifact")
def api_digital_twin_phase5_execution_artifact():
    """Inspect frozen P5.1-P5.3 execution data; never command either robot."""
    return node.phase5_execution_artifact_state()


@app.get("/api/digital-twin/phase5/execution-transport")
def api_digital_twin_phase5_execution_transport():
    """Inspect P5.10 transport readiness; never dispatch motion."""
    return node.phase5_execution_transport_state()


@app.get("/api/digital-twin/phase5/execution")
def api_digital_twin_phase5_execution_state():
    """Inspect the Phase-5-only gate, operator authority, and minimal state."""
    return node.phase5_execution_state()


@app.post("/api/digital-twin/phase5/prepare")
def api_digital_twin_phase5_prepare():
    """No-motion capture of a no-expiry single-use pending operator authority."""
    return node.prepare_phase5_execution()


@app.post("/api/digital-twin/phase5/execute")
def api_digital_twin_phase5_execute(req: DigitalTwinPhase5ExecuteRequest):
    """Consume explicit confirmation and submit the frozen dual trajectory."""
    return node.execute_phase5_execution(req.operator_confirmed)


@app.post("/api/digital-twin/phase5/replan-from-current")
def api_digital_twin_phase5_replan_from_current():
    """Capture fresh cached Actual joints for the next plan; no motion."""
    return node.phase5_replan_from_current()


@app.post("/api/digital-twin/phase5/move-to-initial")
def api_digital_twin_phase5_move_to_initial():
    """Explicit real recovery motion to frozen artifact sample zero."""
    return node.phase5_move_to_initial()


@app.get("/api/experiments")
def api_experiment_runs():
    """List persisted observational runs; never command either robot."""
    return node.list_experiment_runs()


@app.get("/api/experiments/recorder")
def api_experiment_recorder_state():
    return node.experiment_recorder_state()


@app.post("/api/experiments/arm")
def api_experiment_arm(req: ExperimentArmRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    try:
        return node.arm_experiment_recorder(payload)
    except ExperimentError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error


@app.post("/api/experiments/disarm")
def api_experiment_disarm():
    try:
        return node.disarm_experiment_recorder()
    except ExperimentError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


@app.get("/api/experiments/{run_id}")
def api_experiment_load(run_id: str):
    try:
        return node.load_experiment_run(run_id)
    except ExperimentError as error:
        raise HTTPException(status_code=404, detail=str(error)) from error


@app.get("/api/experiments/{run_id}/export")
def api_experiment_export(run_id: str):
    """Export one complete machine-readable run as JSON."""
    try:
        return node.load_experiment_run(run_id)
    except ExperimentError as error:
        raise HTTPException(status_code=404, detail=str(error)) from error


@app.post("/api/experiments/{run_id}/exp1-analysis")
def api_experiment_exp1_analysis(run_id: str):
    """Compute and persist EXP-1 from recorded files; no motion or live reads."""
    try:
        return node.analyze_experiment_run_exp1(run_id)
    except ExperimentError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error


@app.post("/api/experiments/{run_id}/exp2-analysis")
def api_experiment_exp2_analysis(run_id: str):
    """Compute/persist offline EXP-2 rigid-grasp metrics; no motion/live reads."""
    try:
        return node.analyze_experiment_run_exp2(run_id)
    except ExperimentError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error


@app.patch("/api/experiments/{run_id}")
def api_experiment_relabel(run_id: str, req: ExperimentRelabelRequest):
    try:
        return node.relabel_experiment_run(run_id, req.label)
    except ExperimentError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error


@app.delete("/api/experiments/{run_id}")
def api_experiment_delete(run_id: str):
    try:
        return node.delete_experiment_run(run_id)
    except ExperimentError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error


@app.get("/api/digital-twin/center-paths")
def api_digital_twin_center_paths():
    return node.list_center_paths()


@app.post("/api/digital-twin/center-path/save")
def api_digital_twin_center_path_save(req: DigitalTwinCenterPathSaveRequest):
    return node.save_center_path(req.name, req.path, req.overwrite)


@app.post("/api/digital-twin/center-path/load")
def api_digital_twin_center_path_load(req: DigitalTwinCenterPathNameRequest):
    return node.load_center_path(req.name)


@app.post("/api/digital-twin/center-path/rename")
def api_digital_twin_center_path_rename(req: DigitalTwinCenterPathRenameRequest):
    return node.rename_center_path(req.name, req.new_name)


@app.post("/api/digital-twin/center-path/delete")
def api_digital_twin_center_path_delete(req: DigitalTwinCenterPathNameRequest):
    return node.delete_center_path(req.name)


@app.get("/api/digital-twin/grasp-configuration")
def api_digital_twin_grasp_configuration():
    return {"ok": True, "grasp": node.digital_twin_grasp_configuration_state()}


@app.post("/api/digital-twin/grasp-configuration/lock")
def api_digital_twin_lock_grasp_configuration(req: DigitalTwinGraspLockRequest):
    payload = req.model_dump() if hasattr(req, "model_dump") else req.dict()
    try:
        return {
            "ok": True,
            "grasp": node.lock_digital_twin_grasp_configuration(payload),
        }
    except RigidGraspConfigurationError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error


@app.post("/api/digital-twin/grasp-configuration/unlock")
def api_digital_twin_unlock_grasp_configuration():
    return {
        "ok": True,
        "grasp": node.unlock_digital_twin_grasp_configuration(),
    }


@app.post("/api/digital-twin/validate-trajectory")
def api_digital_twin_validate_trajectory(req: DigitalTwinTrajectoryValidationRequest):
    try:
        validation = node.validate_digital_twin_trajectory(
            req.trajectory, req.sampled_path
        )
    except (TrajectoryValidationInputError, SampledPathValidationInputError) as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    sampled_status = (validation.get("sampled_path") or {}).get("status")
    return {
        "ok": (
            validation["status"] in {"PASS", "FAIL"}
            and sampled_status not in {"UNAVAILABLE", "TIMEOUT", "ERROR"}
        ),
        "validation": validation,
    }


@app.post("/api/digital-twin/plan-object-global")
def api_digital_twin_plan_object_global(req: DigitalTwinObjectGlobalPlanRequest):
    payload = (
        req.model_dump(exclude_unset=True)
        if hasattr(req, "model_dump")
        else req.dict(exclude_unset=True)
    )
    try:
        return node.plan_digital_twin_object_global(payload)
    except ObjectGlobalPlanInputError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error


@app.post("/api/digital-twin/validate-phase4-trajectory")
def api_digital_twin_validate_phase4_trajectory(
    req: DigitalTwinPhase4TrajectoryValidationRequest,
):
    payload = (
        req.model_dump(exclude_unset=True)
        if hasattr(req, "model_dump")
        else req.dict(exclude_unset=True)
    )
    try:
        validation = node.validate_digital_twin_phase4_trajectory(payload)
    except Phase4ValidationInputError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    return {"ok": validation["status"] in {"PASS", "INCOMPLETE"}, "validation": validation}


@app.post("/api/digital-twin/validate-phase4-collision")
def api_digital_twin_validate_phase4_collision(
    req: DigitalTwinPhase4CollisionValidationRequest,
):
    payload = (
        req.model_dump(exclude_unset=True)
        if hasattr(req, "model_dump")
        else req.dict(exclude_unset=True)
    )
    try:
        validation = node.validate_digital_twin_phase4_collision(payload)
    except (Phase4CollisionInputError, SampledPathValidationInputError) as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    return {"ok": validation["status"] in {"PASS", "INCOMPLETE"}, "validation": validation}


@app.post("/api/digital-twin/validate-phase4")
def api_digital_twin_validate_phase4(
    req: DigitalTwinPhase4UnifiedValidationRequest,
):
    payload = (
        req.model_dump(exclude_unset=True)
        if hasattr(req, "model_dump")
        else req.dict(exclude_unset=True)
    )
    try:
        report = node.validate_digital_twin_phase4_unified(payload)
    except (
        Phase4ValidationInputError,
        Phase4CollisionInputError,
        Phase4UnifiedValidationInputError,
        SampledPathValidationInputError,
    ) as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    return {"ok": True, "report": report}


@app.post("/api/jog/start")
def api_jog_start(req: JogStartRequest):
    return node.start_jog(req)


@app.post("/api/jog/heartbeat")
def api_jog_heartbeat():
    return node.heartbeat_jog()


@app.post("/api/jog/stop")
def api_jog_stop(req: StopRequest):
    return node.stop_jog(req.side)


@app.post("/api/stop")
def api_stop(req: StopRequest):
    return node.request_stop_all(req.side)


@app.post("/api/home")
def api_home(req: HomeRequest):
    return node.home(req.side, req.vel, req.acc)


@app.get("/api/waypoints")
def api_waypoints():
    return node.load_waypoints()


@app.post("/api/waypoint/save")
def api_waypoint_save(req: WaypointSaveRequest):
    return node.save_waypoint(req.name, req.side)


@app.post("/api/waypoint/run")
def api_waypoint_run(req: WaypointRunRequest):
    return node.run_waypoint(req.name, req.side, req.vel, req.acc)


@app.post("/api/sequence/run")
def api_sequence_run(req: SequenceRunRequest):
    return node.run_sequence(req.names, req.side, req.vel, req.acc)









@app.get("/api/program/list")
def api_program_list():
    return node.list_program_files()


@app.post("/api/program/save")
def api_program_save(req: ProgramSaveRequest):
    return node.save_program_file(req.name, req.steps)


@app.post("/api/program/load")
def api_program_load(req: ProgramNameRequest):
    return node.load_program_file(req.name)


@app.post("/api/program/delete")
def api_program_delete(req: ProgramNameRequest):
    return node.delete_program_file(req.name)

@app.post("/api/program/run")
def api_program_run(req: ProgramRunRequest):
    # Legacy saved-waypoint programs are independent of the Digital-Twin
    # Phase-4 plan authority. Phase-5 validated-plan execution must use its
    # own explicit gated contract rather than silently repurposing this route.
    return node.run_program(
        req.steps,
        req.side,
        req.vel,
        req.acc,
        req.loop_mode,
        req.loop_count,
    )

@app.post("/api/sequence/stop")
def api_sequence_stop(req: StopRequest):
    return node.request_stop_all(req.side)


@app.post("/api/direct/joint_move")
def api_direct_joint_move(req: DirectJointMoveRequest):
    return node.direct_joint_move(req.side, req.left_deg, req.right_deg, req.vel, req.acc)


@app.post("/api/waypoint/delete")
def api_waypoint_delete(req: WaypointDeleteRequest):
    return node.delete_waypoint(req.name)


@app.post("/api/direct/tcp_move")
def api_direct_tcp_move(req: DirectTcpMoveRequest):
    return node.direct_tcp_move(req.side, req.left_pose, req.right_pose, req.vel, req.acc)
