// Phase 1B/1C.1 pinned browser dependencies:
// Three.js 0.160.0 and urdf-loader 0.12.5 (resolved by index.html import map).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import URDFLoader from "urdf-loader";
import {
  DEFAULT_STALE_TIMEOUT_MS,
  isNormalizedSnapshotStale,
  normalizeDualArmStatusSnapshot,
} from "./digital_twin_status_adapter.js";
import {
  DEFAULT_VISUAL_SMOOTHING_TAU_MS,
  copyDualArmPose,
  smoothDualArmPose,
} from "./digital_twin_live_smoothing.js";
import {
  maxAbsoluteJointDelta,
  normalizePlannedDualArmPose,
} from "./digital_twin_planned_preview.js";
import {
  normalizeDualArmTrajectory,
  sampleTrajectoryAtTime,
  trajectoryDurationSeconds,
  trajectoryPointAtIndex,
} from "./digital_twin_trajectory_preview.js";
import { DUAL_JAKA_A12_JOINT_LIMIT_METADATA } from "./digital_twin_joint_limit_metadata.js";
import { validateTrajectoryJointLimits } from "./digital_twin_trajectory_validation.js";
import { requestMoveItTrajectoryValidation } from "./digital_twin_moveit_validation_source.js";
import { requestPlanningStartState } from "./digital_twin_planning_start_state_source.js";
import {
  INITIAL_SYNTHETIC_OBJECT_POSE,
  computeObjectGraspRelativeState,
  computeWorldGraspFrameMatrices,
  deriveGraspFrameResetFromWorldTips,
  inverseRigidMatrix4,
  matrix4FromTranslationRpy,
  multiplyMatrix4,
  normalizeObjectRelativeGraspPose,
  normalizeObjectPreviewPose,
  poseFromRigidMatrix4,
  verifyGraspFrameAlignment,
} from "./digital_twin_object_grasp_preview.js";
import {
  requestGraspConfiguration,
  requestLockGraspConfiguration,
  requestUnlockGraspConfiguration,
} from "./digital_twin_grasp_configuration_source.js";
import {
  SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
  nearestObjectTrajectorySample,
  objectTrajectorySampleAtIndex,
} from "./digital_twin_object_trajectory_preview.js";
import {
  PHASE3_DEMO_FIXED_ORIENTATION_RPY_RAD,
  PHASE3_DEMO_OBJECT_WAYPOINTS,
  addObjectPlanningWaypoint,
  buildObjectGlobalPlanRequest,
  deleteObjectPlanningWaypoint,
  duplicateObjectPlanningWaypoint,
  globalPlanToPlannedTrajectory,
  moveObjectPlanningWaypoint,
  renumberObjectPlanningWaypoints,
  reorderObjectPlanningWaypoint,
  normalizeApproachWaypoints,
  normalizePlanningStartState,
  normalizeObjectGlobalPlanResponse,
  normalizeObjectPlanningWaypoints,
  requestObjectGlobalPlan,
  sampleIntegratedObjectGlobalPlan,
} from "./digital_twin_phase3_planning.js";
import {
  angleUnitMetadata,
  formatRotationRadians,
  normalizeAngleUnit,
  rotationDisplayToRadians,
  rotationRadiansToDisplay,
} from "./digital_twin_angle_units.js";
import {
  deleteCenterPath,
  listCenterPaths,
  loadCenterPath,
  renameCenterPath,
  saveCenterPath,
} from "./digital_twin_center_path_library.js";
import {
  matrix4ElementsToWorldPose,
} from "./digital_twin_phase1_state.js";
import {
  buildPhase4ValidationRequest,
  normalizePhase4ValidationStartState,
  requestPhase4TrajectoryValidation,
} from "./digital_twin_phase4_validation.js";
import {
  buildPhase4CollisionRequest,
  requestPhase4CollisionValidation,
} from "./digital_twin_phase4_collision.js";
import {
  buildPhase4UnifiedValidationRequest,
  phase4ExecutionGateState,
  requestPhase4UnifiedValidation,
} from "./digital_twin_phase4_unified_validation.js";
import {
  bindPhase5ExecutionControls,
} from "./digital_twin_phase5_execution.js?v=feature7-simple-execute-confirmation-v1";
import {
  MODEL_CALIBRATION_MISMATCH,
  requestCalibrationRevisionState,
} from "./digital_twin_world_calibration.js";

const MODEL_URL = "/digital-twin/assets/dual_jaka_a12_web.urdf";
const LOAD_TIMEOUT_MS = 20000;
const MODEL_READINESS_RETRY_MS = 40;
const EXPECTED_VISUAL_COUNT = 14;
const PLANNED_GHOST_OPACITY = 0.38;
const MODEL_TCP_AXES_SIZE_M = 0.14;
const NORMALIZED_WHEEL_ZOOM_RATE = 0.035;
const WHEEL_PIXEL_DELTA_PER_STEP = 100;
const WHEEL_LINE_DELTA_PER_STEP = 3;
const ORBIT_MIN_DISTANCE_RADIUS_MULTIPLIER = 1.1;
const ORBIT_MAX_DISTANCE_RADIUS_MULTIPLIER = 8;
const INITIALIZATION_FLAG = "__dualArmDigitalTwinInitialized";
const EXPECTED_JOINTS = [
  "left_joint_1",
  "left_joint_2",
  "left_joint_3",
  "left_joint_4",
  "left_joint_5",
  "left_joint_6",
  "right_joint_1",
  "right_joint_2",
  "right_joint_3",
  "right_joint_4",
  "right_joint_5",
  "right_joint_6",
];
// Kept in the controller slice used by legacy offline harnesses.
const PHASE3_WEB_OPTIMALITY_SCOPE = (
  "Exact optimum over generated layered candidate graph"
);
const PHASE3_WEB_PLANNER_SOURCE = (
  "GLOBAL GRAPH SEARCH — GENERATED GRAPH OPTIMUM"
);
const MODEL_TCP_LINKS = Object.freeze({ left: "left_J6", right: "right_J6" });
const MODEL_TCP_SOURCE = "URDF MODEL FK FROM MIRRORED JOINT STATE";
const MODEL_TCP_SEMANTIC = "Digital Twin model TCP/flange";
const MODEL_TCP_RPY_CONVENTION =
  "RPY radians; R = Rz(yaw) * Ry(pitch) * Rx(roll)";

const loadState = {
  status: "INITIALIZING",
  loadedMovableJoints: 0,
  loadedVisuals: 0,
  error: null,
};

const worldCalibrationState = {
  status: "LOADING",
  matches: false,
  planningReady: false,
  backendRevision: null,
  modelRevision: null,
  calibrationState: "UNKNOWN",
  physicallyCalibrated: false,
  error: null,
};

const MIRROR_MODES = Object.freeze({
  STATIC: "STATIC",
  MIRROR_READY: "MIRROR_READY",
  LIVE_MIRROR: "LIVE_MIRROR",
  STALE: "STALE",
  INVALID: "INVALID",
});

const ZERO_JOINT_VALUES = Object.freeze({
  left: Object.freeze([0, 0, 0, 0, 0, 0]),
  right: Object.freeze([0, 0, 0, 0, 0, 0]),
});

const MOCK_POSE_A = Object.freeze({
  left: Object.freeze({ joint: Object.freeze([0, 0, 0, 0, 0, 0]) }),
  right: Object.freeze({ joint: Object.freeze([0, 0, 0, 0, 0, 0]) }),
});

const MOCK_POSE_B = Object.freeze({
  left: Object.freeze({ joint: Object.freeze([0.20, -0.35, 0.25, 0.15, -0.20, 0.10]) }),
  right: Object.freeze({ joint: Object.freeze([-0.20, 0.35, -0.25, -0.15, 0.20, -0.10]) }),
});

const PLANNED_MOCK_POSE_A = Object.freeze({
  left: Object.freeze([0, 0, 0, 0, 0, 0]),
  right: Object.freeze([0, 0, 0, 0, 0, 0]),
});

const PLANNED_MOCK_POSE_B = Object.freeze({
  left: Object.freeze([0.30, -0.40, 0.20, 0.25, -0.15, 0.12]),
  right: Object.freeze([-0.30, 0.40, -0.20, -0.25, 0.15, -0.12]),
});

const MOCK_TRAJECTORY_A = {
  name: "Mock Trajectory A",
  points: [
    {
      time_from_start_s: 0,
      left: [0, 0, 0, 0, 0, 0],
      right: [0, 0, 0, 0, 0, 0],
    },
    {
      time_from_start_s: 1.5,
      left: [0.15, -0.20, 0.15, 0.10, -0.10, 0.05],
      right: [-0.15, 0.20, -0.15, -0.10, 0.10, -0.05],
    },
    {
      time_from_start_s: 3,
      left: [0.30, -0.40, 0.20, 0.25, -0.15, 0.12],
      right: [-0.30, 0.40, -0.20, -0.25, 0.15, -0.12],
    },
  ],
};

const MOCK_TRAJECTORY_B = {
  name: "Mock Trajectory B",
  points: [
    {
      time_from_start_s: 0,
      left: [0, 0, 0, 0, 0, 0],
      right: [0, 0, 0, 0, 0, 0],
    },
    {
      time_from_start_s: 0.6,
      left: [0.08, -0.10, 0.06, 0.04, -0.03, 0.02],
      right: [-0.05, 0.08, -0.04, -0.03, 0.04, -0.02],
    },
    {
      time_from_start_s: 1.4,
      left: [0.18, -0.24, 0.12, 0.10, -0.08, 0.06],
      right: [-0.14, 0.20, -0.10, -0.08, 0.09, -0.05],
    },
    {
      time_from_start_s: 2.7,
      left: [0.10, -0.12, 0.20, 0.16, -0.12, 0.09],
      right: [-0.22, 0.30, -0.18, -0.16, 0.12, -0.08],
    },
    {
      time_from_start_s: 4.2,
      left: [0.26, -0.34, 0.18, 0.22, -0.14, 0.11],
      right: [-0.26, 0.34, -0.18, -0.22, 0.14, -0.11],
    },
  ],
};

const MOCK_INVALID_LIMIT_TRAJECTORY = {
  name: "Invalid Joint Limit Test",
  points: [
    {
      time_from_start_s: 0,
      left: [0, 0, 0, 0, 0, 0],
      right: [0, 0, 0, 0, 0, 0],
    },
    {
      time_from_start_s: 1.5,
      left: [0.15, -0.20, 6.50, 0.10, -0.10, 0.05],
      right: [-0.15, 0.20, -0.15, -0.10, 0.10, -0.05],
    },
    {
      time_from_start_s: 3,
      left: [0.30, -0.40, 0.20, 0.25, -0.15, 0.12],
      right: [-0.30, 0.40, -0.20, -0.25, 0.15, -0.12],
    },
  ],
};

const KNOWN_MOVEIT_VALID_POSE = Object.freeze({
  left: Object.freeze([3.14, 0.52124, -0.800072, 0, 0.798816, 0]),
  right: Object.freeze([0, 2.617504, 0.798816, 0, 2.339928, 0]),
});

const KNOWN_MOVEIT_COLLISION_POSE = Object.freeze({
  left: Object.freeze([
    2.713157, -0.969667, -2.416695, -2.393133, 2.062073, 0.615523,
  ]),
  right: Object.freeze([
    1.822536, 1.363256, 0.214982, 2.807525, -0.720792, 0.308815,
  ]),
});

const KNOWN_MOVEIT_COLLISION_TRAJECTORY = {
  name: "Known MoveIt Collision Test",
  points: [
    {
      time_from_start_s: 0,
      left: [...KNOWN_MOVEIT_VALID_POSE.left],
      right: [...KNOWN_MOVEIT_VALID_POSE.right],
    },
    {
      time_from_start_s: 1,
      left: [...KNOWN_MOVEIT_COLLISION_POSE.left],
      right: [...KNOWN_MOVEIT_COLLISION_POSE.right],
    },
  ],
};

const DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD = 0.05;
const SAMPLED_PATH_MOCK_TRAJECTORY = {
  name: "SAMPLED-PATH MOCK TEST",
  points: [
    {
      time_from_start_s: 0,
      left: [0, 0, 0, 0, 0, 0],
      right: [0, 0, 0, 0, 0, 0],
    },
    {
      time_from_start_s: 1,
      left: [0.12, 0, 0, 0, 0, 0],
      right: [0, 0, 0, 0, 0, 0],
    },
  ],
};

const CONFIRMED_SAMPLED_COLLISION_TRAJECTORY = {
  name: "Confirmed Sampled Collision — MoveIt Runtime Fixture",
  points: [
    {
      time_from_start_s: 0,
      left: [-1.757703, -0.355475, 0.115155, -1.264532, -2.483733, -2.463276],
      right: [2.017201, -1.186278, -0.911243, 1.576992, 1.103272, -0.331881],
    },
    {
      time_from_start_s: 2,
      left: [0.278808, 1.515081, -0.391248, -1.921538, 2.082136, 1.900224],
      right: [-0.736672, -2.398909, 0.073763, -0.023872, 1.615867, 0.576897],
    },
  ],
};

const mirrorState = {
  mode: MIRROR_MODES.STATIC,
  enabled: false,
  staleTimeoutMs: DEFAULT_STALE_TIMEOUT_MS,
  latestValidSnapshot: null,
  lastAcceptedSnapshotMs: null,
  lastAppliedSnapshotMs: null,
  validationError: null,
  updateSource: "NONE",
};

const LIVE_MIRROR_VISUAL_UI_SYNC_MS = 50;
const mirrorVisualState = {
  currentPose: null,
  lastFrameTimeMs: null,
  lastUiSyncFrameMs: null,
  smoothingTauMs: DEFAULT_VISUAL_SMOOTHING_TAU_MS,
  mode: "REQUEST_ANIMATION_FRAME_EXPONENTIAL",
};

const mainModelState = {
  source: "URDF DEFAULT POSE",
  configurationSource: "UNAVAILABLE",
};

const plannedPreviewState = {
  status: "UNAVAILABLE",
  visible: false,
  latestPose: null,
  source: "NONE",
  validationError: null,
  maxJointDeltaRad: null,
};

const plannedLoadState = {
  status: "UNAVAILABLE",
  loadedMovableJoints: 0,
  loadedVisuals: 0,
  error: null,
};

const trajectoryPreviewState = {
  status: "EMPTY",
  trajectory: null,
  currentTimeS: 0,
  currentPointIndex: 0,
  currentSegmentIndex: 0,
  currentAlpha: 0,
  playing: false,
  playbackRate: 1.0,
  source: "NONE",
  validationError: null,
  syncIntegratedPlan: true,
  previousFrameTimeMs: null,
  animationFrameId: null,
};

// Independent from planned joint trajectory, planned ghost, and mirror state.
const objectTrajectoryPreviewState = {
  status: "EMPTY",
  trajectory: null,
  currentTimeS: 0,
  currentSampleIndex: 0,
  playing: false,
  playbackRate: 1.0,
  error: null,
  previousFrameTimeMs: null,
  animationFrameId: null,
};

const PHASE3_WEB_DEFAULT_CANDIDATE_ATTEMPTS_PER_ARM = 3;
const ANGLE_UNIT_STORAGE_KEY = "jaka.digitalTwin.rotationUnit";

function storedAngleUnitPreference() {
  try {
    return window.localStorage.getItem(ANGLE_UNIT_STORAGE_KEY) === "radians"
      ? "radians" : "degrees";
  } catch (_error) {
    return "degrees";
  }
}

const angleUnitState = {
  unit: storedAngleUnitPreference(),
};
const newObjectWaypointDraftRpyRad = [0.0, 0.0, 0.0];

// Phase 3B planning-only state. Integrated preview deliberately borrows the
// planned trajectory clock above; it never owns a second animation frame.
const objectWaypointPlanningState = {
  waypoints: [],
  approachWaypoints: [],
  // Initialized without an imported fixture so existing isolated controller
  // harnesses remain self-contained; planning captures the live preview RPY.
  fixedOrientationRpyRad: [0.0, 0.0, 0.0],
  orientationSource: "CURRENT CENTER PREVIEW ORIENTATION",
  segmentDurationS: 2.0,
  samplesPerSegment: 3,
  candidateAttemptsPerArm: PHASE3_WEB_DEFAULT_CANDIDATE_ATTEMPTS_PER_ARM,
  status: "EMPTY",
  error: null,
  planning: false,
};

const centerPathLibraryState = {
  paths: [],
  selectedName: null,
  loadedName: null,
  dirty: false,
  status: "IDLE",
  error: null,
  requestInFlight: false,
};
let centerPathLibraryApplying = false;

const planningStartState = {
  left: null,
  right: null,
  jointNames: [],
  units: "radian",
  source: "UNAVAILABLE",
  status: "LOADING",
  error: null,
};

let nextPlanningStartOverride = null;

const objectDragState = {
  enabled: false,
  dragging: false,
  mode: "translate",
  rotationEnabled: false,
  scaleEnabled: false,
  ownership: "NONE",
  error: null,
};

const GRASP_SELECTED_FRAMES = Object.freeze({
  CENTER: "CENTER",
  LEFT_GRASP: "LEFT_GRASP",
  RIGHT_GRASP: "RIGHT_GRASP",
});

const rigidGraspState = {
  status: "LOADING",
  state: "GRASP_UNLOCKED",
  draft: null,
  draftSource: "UNAVAILABLE",
  draftContentRevision: null,
  lockedSnapshot: null,
  authoritativeRevision: null,
  lockGeneration: 0,
  rotationConvention: "R = Rz(yaw) * Ry(pitch) * Rx(roll)",
  units: { translation: "meter", rotation: "radian" },
  semantic: "OPERATOR-CONFIGURABLE RIGID GRASP",
  physicallyCalibrated: false,
  plannerIntegration: "UNAVAILABLE",
  worldCalibrationRevision: null,
  selectedFrame: GRASP_SELECTED_FRAMES.CENTER,
  gizmoMode: "translate",
  requestInFlight: false,
  alignmentStatus: "NOT CHECKED",
  alignmentResidual: null,
  error: null,
};

const objectWaypointVisualizationState = {
  semantic: "CENTER WAYPOINT PATH — PRE-PLAN ONLY",
  markerCount: 0,
  pathOrder: [],
};

const objectGlobalPlanState = {
  status: "EMPTY",
  plan: null,
  error: null,
  requestGeneration: 0,
};

const integratedPlanPreviewState = {
  active: false,
  owner: "NONE",
  plan: null,
  error: null,
};

const phase4TrajectoryValidationState = {
  status: "NOT_VALIDATED",
  validating: false,
  startState: null,
  startStateKind: "UNAVAILABLE",
  // Literals keep legacy controller-slice harnesses self-contained; the pure
  // request module exports the same authoritative Phase-4A defaults.
  positionToleranceM: 0.001,
  orientationToleranceRad: 0.001,
  result: null,
  error: null,
};

const phase4CollisionValidationState = {
  status: "NOT_VALIDATED",
  validating: false,
  result: null,
  error: null,
};

const phase4UnifiedValidationState = {
  status: "NOT_VALIDATED",
  validating: false,
  report: null,
  error: null,
  stale: false,
  invalidationReason: "NO REPORT",
  currentPlanFingerprint: null,
  generation: 0,
};

const trajectoryValidationState = {
  status: "NOT_VALIDATED",
  jointLimits: "NOT_VALIDATED",
  moveitStateValidity: "NOT_RUN",
  collision: "NOT_RUN",
  valid: null,
  checkedPointCount: 0,
  checkedJointCount: 0,
  violationCount: 0,
  violations: [],
  firstViolation: null,
  moveitCheckedPointCount: 0,
  moveitFailedPointCount: 0,
  firstMoveItFailedPoint: null,
  firstCollisionPair: null,
  collisionPairCount: 0,
  maxPenetrationDepthM: null,
  sampledPathStatus: "NOT_RUN",
  sampledPathMaxJointStepRad: DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD,
  sampledPathSegmentCount: 0,
  sampledPathGeneratedSampleCount: 0,
  sampledPathCheckedSampleCount: 0,
  sampledPathFailedSampleCount: 0,
  firstSampledPathFailure: null,
  sampledPathCollisionPairCount: 0,
  sampledPathFirstCollisionPair: null,
  sampledPathMaxPenetrationDepthM: null,
  source: "NONE",
  error: null,
};

let container = null;
let statusElement = null;
let scene = null;
let camera = null;
let renderer = null;
let controls = null;
let objectTransformControls = null;
let objectGizmoTarget = null;
let objectGizmoSynchronizing = false;
let grid = null;
let axes = null;
let robot = null;
let plannedRobot = null;
let latestActualPose = null;
let plannedLoadStarted = false;
let moveitValidationInFlight = false;
let trajectoryValidationGeneration = 0;
let homeCameraPosition = null;
let homeCameraTarget = null;
let objectFrame = null;
let objectFrameAxes = null;
let leftGraspFrame = null;
let rightGraspFrame = null;
let objectWaypointHoverGroup = null;
const objectWaypointHoverFrames = { center: null, left: null, right: null };
const objectWaypointHoverState = {
  visible: false,
  index: null,
  identifier: null,
  error: null,
};
let objectWaypointVisualizationGroup = null;
let objectWaypointPath = null;
const modelTcpFrames = { left: null, right: null };

const modelTcpState = {
  selectedLinks: { ...MODEL_TCP_LINKS },
  frameVisible: { left: false, right: false },
  feedbackStatus: { left: "MISSING", right: "MISSING" },
  lastValidWorldPose: { left: null, right: null },
  frame: "Web/URDF world",
  translationUnit: "meter",
  orientationUnit: "radian",
  orientationConvention: MODEL_TCP_RPY_CONVENTION,
  semantic: MODEL_TCP_SEMANTIC,
  source: MODEL_TCP_SOURCE,
  error: null,
};

const objectPreviewState = {
  pose: null,
  objectVisible: false,
  objectFrameVisible: true,
  graspFramesVisible: true,
  error: null,
};

// Fresh-load initialization is a one-shot snapshot.  It is never re-applied
// merely because live joint feedback changes after the planning frames exist.
let initialGraspBackendStateObserved = false;
let initialGraspFrameSnapshotPending = false;
let initialGraspFrameSnapshotComplete = false;

function setStatus(message, kind = "info") {
  if (loadState.status === "ERROR" && kind !== "error") return;
  loadState.status = kind === "error" ? "ERROR" : loadState.status;
  if (statusElement) {
    statusElement.textContent = message;
    statusElement.dataset.state = kind;
  }
}

function fail(message, error = null) {
  const detail = error && error.message ? `${message}: ${error.message}` : message;
  loadState.status = "ERROR";
  loadState.error = detail;
  setStatus(detail, "error");
  console.error(`[DualArmDigitalTwin] ${detail}`, error || "");
}

function render() {
  if (renderer && scene && camera) {
    renderer.render(scene, camera);
  }
}

function copyModelTcpPose(pose) {
  return pose
    ? {
        translationM: [...pose.translationM],
        rpyRad: [...pose.rpyRad],
      }
    : null;
}

function formatModelTcpValue(value) {
  return typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(6)
    : "UNAVAILABLE";
}

function updateModelTcpUi() {
  const values = {
    digitalTwinModelTcpSource: modelTcpState.source,
    digitalTwinModelTcpSemantic: modelTcpState.semantic,
    digitalTwinModelTcpError: modelTcpState.error || "NONE",
  };
  for (const side of ["left", "right"]) {
    const title = side[0].toUpperCase() + side.slice(1);
    const pose = modelTcpState.lastValidWorldPose[side];
    values[`digitalTwin${title}ModelTcpStatus`] = pose
      ? modelTcpState.feedbackStatus[side]
      : "UNAVAILABLE";
    values[`digitalTwin${title}ModelTcpLink`] = modelTcpState.selectedLinks[side];
    const components = pose
      ? [...pose.translationM, ...pose.rpyRad]
      : [null, null, null, null, null, null];
    ["X", "Y", "Z", "Rx", "Ry", "Rz"].forEach((component, index) => {
      values[`digitalTwin${title}ModelTcp${component}`] =
        formatModelTcpValue(components[index]);
    });
  }
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
}

function setModelTcpFrameVisibility(side, visible) {
  const nextVisible = visible === true && modelTcpFrames[side] !== null;
  if (modelTcpFrames[side]) modelTcpFrames[side].visible = nextVisible;
  modelTcpState.frameVisible[side] = nextVisible;
}

function createModelTcpFrames() {
  if (!robot || !robot.links) {
    modelTcpState.error = "Actual URDF model links are unavailable";
    updateModelTcpUi();
    return false;
  }
  for (const side of ["left", "right"]) {
    const linkName = MODEL_TCP_LINKS[side];
    const terminalLink = robot.links[linkName];
    if (!terminalLink) {
      modelTcpState.error = `Model TCP/flange link unavailable: ${linkName}`;
      setModelTcpFrameVisibility(side, false);
      updateModelTcpUi();
      return false;
    }
    if (!modelTcpFrames[side]) {
      const frame = new THREE.AxesHelper(MODEL_TCP_AXES_SIZE_M);
      frame.name = `${side} Digital Twin model TCP/flange axes`;
      frame.userData.semantic = MODEL_TCP_SEMANTIC;
      frame.userData.source = MODEL_TCP_SOURCE;
      frame.visible = false;
      terminalLink.add(frame);
      modelTcpFrames[side] = frame;
    }
  }
  modelTcpState.error = null;
  robot.updateMatrixWorld(true);
  updateModelTcpUi();
  render();
  return true;
}

function captureModelTcpWorldPoses() {
  if (!robot || !robot.links) return false;
  robot.updateMatrixWorld(true);
  const nextPoses = {};
  for (const side of ["left", "right"]) {
    const terminalLink = robot.links[MODEL_TCP_LINKS[side]];
    if (!terminalLink || !terminalLink.matrixWorld) {
      modelTcpState.error = `Cannot inspect ${side} model TCP/flange world pose`;
      setModelTcpFrameVisibility(side, false);
      updateModelTcpUi();
      return false;
    }
    try {
      nextPoses[side] = matrix4ElementsToWorldPose(
        Array.from(terminalLink.matrixWorld.elements),
      );
    } catch (error) {
      modelTcpState.error = error && error.message
        ? error.message
        : `Invalid ${side} model TCP/flange transform`;
      setModelTcpFrameVisibility(side, false);
      updateModelTcpUi();
      return false;
    }
  }
  for (const side of ["left", "right"]) {
    modelTcpState.lastValidWorldPose[side] = nextPoses[side];
    modelTcpState.feedbackStatus[side] = "LIVE";
    setModelTcpFrameVisibility(side, true);
  }
  modelTcpState.error = null;
  updateModelTcpUi();
  render();
  return true;
}

function setModelTcpFeedbackStatus(statusBySide) {
  if (!statusBySide || typeof statusBySide !== "object") return getModelTcpState();
  for (const side of ["left", "right"]) {
    const status = statusBySide[side];
    if (!["LIVE", "STALE", "MISSING", "INVALID"].includes(status)) continue;
    // LIVE is asserted only when a mirrored snapshot has actually been applied.
    if (status === "LIVE") continue;
    modelTcpState.feedbackStatus[side] = status;
    setModelTcpFrameVisibility(side, false);
  }
  updateModelTcpUi();
  render();
  return getModelTcpState();
}

function getModelTcpState() {
  return {
    selectedLinks: { ...modelTcpState.selectedLinks },
    frameVisible: { ...modelTcpState.frameVisible },
    feedbackStatus: { ...modelTcpState.feedbackStatus },
    currentWorldPose: {
      left: copyModelTcpPose(modelTcpState.lastValidWorldPose.left),
      right: copyModelTcpPose(modelTcpState.lastValidWorldPose.right),
    },
    frame: modelTcpState.frame,
    translationUnit: modelTcpState.translationUnit,
    orientationUnit: modelTcpState.orientationUnit,
    orientationConvention: modelTcpState.orientationConvention,
    semantic: modelTcpState.semantic,
    source: modelTcpState.source,
    error: modelTcpState.error,
  };
}

function normalizedWheelSteps(deltaY, deltaMode) {
  if (typeof deltaY !== "number" || !Number.isFinite(deltaY)) return 0;
  const divisor = deltaMode === 1
    ? WHEEL_LINE_DELTA_PER_STEP
    : (deltaMode === 2 ? 1 : WHEEL_PIXEL_DELTA_PER_STEP);
  return Math.max(-1, Math.min(1, deltaY / divisor));
}

function handleViewerWheel(event) {
  if (!camera || !controls) return;
  const wheelSteps = normalizedWheelSteps(event.deltaY, event.deltaMode);
  if (wheelSteps === 0) return;
  event.preventDefault();

  const offset = camera.position.clone().sub(controls.target);
  const currentDistance = offset.length();
  if (!Number.isFinite(currentDistance) || currentDistance <= 0) return;
  const requestedDistance = currentDistance * Math.exp(
    wheelSteps * NORMALIZED_WHEEL_ZOOM_RATE,
  );
  const nextDistance = Math.max(
    controls.minDistance,
    Math.min(controls.maxDistance, requestedDistance),
  );
  offset.multiplyScalar(nextDistance / currentDistance);
  camera.position.copy(controls.target).add(offset);
  controls.update();
  render();
}

function fitModel(rememberAsHome = false) {
  if (!robot || !camera || !controls || !container) {
    return false;
  }

  robot.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(robot);
  if (bounds.isEmpty()) {
    fail("Cannot fit camera because the model bounds are empty");
    return false;
  }

  const size = bounds.getSize(new THREE.Vector3());
  const center = bounds.getCenter(new THREE.Vector3());
  const maxDimension = Math.max(size.x, size.y, size.z, 0.1);
  const boundingSphere = bounds.getBoundingSphere(new THREE.Sphere());
  const sceneRadius = (
    Number.isFinite(boundingSphere.radius) && boundingSphere.radius > 0
  ) ? boundingSphere.radius : maxDimension * 0.5;
  const halfFovRadians = THREE.MathUtils.degToRad(camera.fov * 0.5);
  const distance = (maxDimension * 0.65) / Math.tan(halfFovRadians);
  const viewDirection = new THREE.Vector3(1, -1, 0.7).normalize();

  controls.minDistance = sceneRadius * ORBIT_MIN_DISTANCE_RADIUS_MULTIPLIER;
  controls.maxDistance = sceneRadius * ORBIT_MAX_DISTANCE_RADIUS_MULTIPLIER;
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(viewDirection, distance);
  camera.near = Math.max(distance / 1000, 0.001);
  camera.far = Math.max(distance * 100, 100);
  camera.aspect = container.clientWidth / Math.max(container.clientHeight, 1);
  camera.updateProjectionMatrix();
  controls.update();

  if (rememberAsHome || !homeCameraPosition) {
    homeCameraPosition = camera.position.clone();
    homeCameraTarget = controls.target.clone();
  }
  render();
  return true;
}

function resetCamera() {
  if (!camera || !controls || !homeCameraPosition || !homeCameraTarget) {
    return false;
  }
  camera.position.copy(homeCameraPosition);
  controls.target.copy(homeCameraTarget);
  controls.update();
  render();
  return true;
}

function toggleGrid() {
  if (!grid) return false;
  grid.visible = !grid.visible;
  render();
  return grid.visible;
}

function toggleAxes() {
  if (!axes) return false;
  axes.visible = !axes.visible;
  render();
  return axes.visible;
}

function validateJointArray(side, values) {
  if (!Array.isArray(values) || values.length !== 6) {
    throw new TypeError(`${side} must be an array of exactly six radians values`);
  }
  if (!values.every((value) => typeof value === "number" && Number.isFinite(value))) {
    throw new TypeError(`${side} must contain six finite numbers`);
  }
}

function applyJointValuesToModel(targetModel, values) {
  for (const side of ["left", "right"]) {
    values[side].forEach((value, index) => {
      const jointName = `${side}_joint_${index + 1}`;
      const joint = targetModel.joints[jointName];
      if (!joint) {
        throw new Error(`Model joint is unavailable: ${jointName}`);
      }
      joint.setJointValue(value);
    });
  }
  targetModel.updateMatrixWorld(true);
  render();
}

function updatePlannedDelta() {
  plannedPreviewState.maxJointDeltaRad = (
    plannedPreviewState.latestPose && latestActualPose
  )
    ? maxAbsoluteJointDelta(plannedPreviewState.latestPose, latestActualPose)
    : null;
}

function setJointValues(values) {
  if (!robot) {
    throw new Error("Digital Twin model is not ready");
  }
  if (!values || typeof values !== "object") {
    throw new TypeError("Joint values must contain left and right arrays");
  }

  validateJointArray("left", values.left);
  validateJointArray("right", values.right);
  applyJointValuesToModel(robot, values);
  latestActualPose = {
    left: [...values.left],
    right: [...values.right],
  };
  updatePlannedDelta();
  updatePlannedPreviewUi();
  return true;
}

function getLatestActualPose() {
  return latestActualPose
    ? { left: [...latestActualPose.left], right: [...latestActualPose.right] }
    : null;
}

function dualArmPoseMatches(left, right) {
  if (!latestActualPose) return false;
  return ["left", "right"].every((side) => (
    latestActualPose[side].every((value, index) => (
      value === (side === "left" ? left : right)[index]
    ))
  ));
}

function hasValidLiveFeedbackOwnership(nowMs = Date.now()) {
  return Boolean(
    mirrorState.enabled
    && mirrorState.latestValidSnapshot
    && mirrorState.mode !== MIRROR_MODES.INVALID
    && mirrorState.mode !== MIRROR_MODES.STALE
    && !isNormalizedSnapshotStale(
      mirrorState.latestValidSnapshot,
      nowMs,
      mirrorState.staleTimeoutMs,
    )
  );
}

function applyPlanningStartStateToMainWhenOffline(nowMs = Date.now()) {
  if (
    planningStartState.status !== "READY"
    || !planningStartState.left
    || !planningStartState.right
    || loadState.status !== "READY"
    || !robot
    || hasValidLiveFeedbackOwnership(nowMs)
  ) return false;

  if (!dualArmPoseMatches(planningStartState.left, planningStartState.right)) {
    setJointValues({
      left: planningStartState.left,
      right: planningStartState.right,
    });
  }
  mainModelState.source = "OFFLINE CODE CONFIG";
  mainModelState.configurationSource = planningStartState.source;
  initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady();
  updateRigidGraspConfigurationUi();
  return true;
}

function getLoadState() {
  return { ...loadState };
}

function formatPlannedDelta(value) {
  return typeof value === "number" && Number.isFinite(value)
    ? `${value.toFixed(6)} rad`
    : "UNAVAILABLE";
}

function updatePlannedPreviewUi() {
  const values = {
    digitalTwinPlannedState: plannedPreviewState.status,
    digitalTwinPlannedVisible: String(plannedPreviewState.visible),
    digitalTwinPlannedSource: plannedPreviewState.source,
    digitalTwinPlannedMaxDelta: formatPlannedDelta(
      plannedPreviewState.maxJointDeltaRad,
    ),
    digitalTwinPlannedError: plannedPreviewState.validationError || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
}

function plannedPreviewStateSnapshot() {
  return {
    status: plannedPreviewState.status,
    visible: plannedPreviewState.visible,
    latestPose: plannedPreviewState.latestPose
      ? {
        left: [...plannedPreviewState.latestPose.left],
        right: [...plannedPreviewState.latestPose.right],
      }
      : null,
    source: plannedPreviewState.source,
    validationError: plannedPreviewState.validationError,
    maxJointDeltaRad: plannedPreviewState.maxJointDeltaRad,
    loadState: { ...plannedLoadState },
  };
}

function acceptedPlannedSourceLabel(sourceLabel) {
  return typeof sourceLabel === "string" && sourceLabel.trim().length > 0
    ? sourceLabel.trim()
    : "LOCAL PLANNED POSE";
}

function applyLatestPlannedPose() {
  if (!plannedRobot || plannedLoadState.status !== "READY") return false;
  if (!plannedPreviewState.latestPose) return false;
  applyJointValuesToModel(plannedRobot, plannedPreviewState.latestPose);
  plannedRobot.visible = true;
  plannedPreviewState.visible = true;
  plannedPreviewState.status = "VISIBLE";
  plannedPreviewState.validationError = null;
  updatePlannedPreviewUi();
  return true;
}

function setPlannedJointValues(
  values,
  sourceLabel = "LOCAL PLANNED POSE",
) {
  let normalized;
  try {
    normalized = normalizePlannedDualArmPose(values);
  } catch (error) {
    plannedPreviewState.status = "INVALID";
    plannedPreviewState.validationError = error && error.message
      ? error.message
      : "Invalid planned pose";
    updatePlannedPreviewUi();
    return plannedPreviewStateSnapshot();
  }

  plannedPreviewState.latestPose = normalized;
  plannedPreviewState.source = acceptedPlannedSourceLabel(sourceLabel);
  plannedPreviewState.validationError = null;
  updatePlannedDelta();

  if (!applyLatestPlannedPose()) {
    plannedPreviewState.status = "UNAVAILABLE";
    plannedPreviewState.visible = false;
    plannedPreviewState.validationError = plannedLoadState.error
      || "Planned Ghost Model is not ready";
    updatePlannedPreviewUi();
  }
  return plannedPreviewStateSnapshot();
}

function showPlannedModel() {
  if (!plannedPreviewState.latestPose) {
    plannedPreviewState.status = "INVALID";
    plannedPreviewState.validationError = "No planned pose is available";
    updatePlannedPreviewUi();
    return plannedPreviewStateSnapshot();
  }
  if (!applyLatestPlannedPose()) {
    plannedPreviewState.status = "UNAVAILABLE";
    plannedPreviewState.validationError = plannedLoadState.error
      || "Planned Ghost Model is not ready";
    updatePlannedPreviewUi();
  }
  return plannedPreviewStateSnapshot();
}

function hidePlannedModel() {
  if (plannedRobot) plannedRobot.visible = false;
  plannedPreviewState.visible = false;
  plannedPreviewState.status = plannedLoadState.status === "READY"
    ? "HIDDEN"
    : "UNAVAILABLE";
  plannedPreviewState.validationError = plannedLoadState.error;
  updatePlannedPreviewUi();
  render();
  return plannedPreviewStateSnapshot();
}

function clearPlannedPreview() {
  if (plannedRobot) plannedRobot.visible = false;
  plannedPreviewState.visible = false;
  plannedPreviewState.latestPose = null;
  plannedPreviewState.source = "NONE";
  plannedPreviewState.validationError = plannedLoadState.error;
  plannedPreviewState.maxJointDeltaRad = null;
  plannedPreviewState.status = plannedLoadState.status === "READY"
    ? "READY"
    : "UNAVAILABLE";
  updatePlannedPreviewUi();
  render();
  return plannedPreviewStateSnapshot();
}

function getPlannedPreviewState() {
  return plannedPreviewStateSnapshot();
}

function captureActualAsPlanned() {
  if (!latestActualPose) {
    plannedPreviewState.status = "INVALID";
    plannedPreviewState.validationError = "Actual joint pose is unavailable";
    updatePlannedPreviewUi();
    return plannedPreviewStateSnapshot();
  }
  return setPlannedJointValues(
    {
      left: [...latestActualPose.left],
      right: [...latestActualPose.right],
    },
    "ACTUAL POSE SNAPSHOT",
  );
}

function trajectoryStateSnapshot() {
  const trajectory = trajectoryPreviewState.trajectory;
  const durationS = trajectory ? trajectoryDurationSeconds(trajectory) : 0;
  return {
    status: trajectoryPreviewState.status,
    trajectory: trajectory
      ? {
        name: trajectory.name,
        points: trajectory.points.map((point) => ({
          time_from_start_s: point.time_from_start_s,
          left: [...point.left],
          right: [...point.right],
        })),
      }
      : null,
    currentTimeS: trajectoryPreviewState.currentTimeS,
    currentPointIndex: trajectoryPreviewState.currentPointIndex,
    currentSegmentIndex: trajectoryPreviewState.currentSegmentIndex,
    currentAlpha: trajectoryPreviewState.currentAlpha,
    playing: trajectoryPreviewState.playing,
    playbackRate: trajectoryPreviewState.playbackRate,
    source: trajectoryPreviewState.source,
    validationError: trajectoryPreviewState.validationError,
    syncIntegratedPlan: trajectoryPreviewState.syncIntegratedPlan,
    pointCount: trajectory ? trajectory.points.length : 0,
    durationS,
    progress: durationS > 0
      ? trajectoryPreviewState.currentTimeS / durationS
      : 0,
  };
}

function formatTrajectoryTime(value, fractionDigits = 1) {
  return Number.isFinite(value) ? value.toFixed(fractionDigits) : "0.0";
}

function updateTrajectoryPreviewUi() {
  const state = trajectoryStateSnapshot();
  const pointLabel = state.pointCount > 0
    ? `segment ${state.currentSegmentIndex + 1}/${state.pointCount - 1}; `
      + `point ${state.currentPointIndex + 1}/${state.pointCount}`
    : "NONE";
  const values = {
    digitalTwinTrajectoryState: state.status,
    digitalTwinTrajectoryName: state.trajectory ? state.trajectory.name : "NONE",
    digitalTwinTrajectoryPointCount: String(state.pointCount),
    digitalTwinTrajectoryPosition: pointLabel,
    digitalTwinTrajectoryTime: `${formatTrajectoryTime(state.currentTimeS, 2)} s`,
    digitalTwinTrajectoryDuration: `${state.durationS.toFixed(2)} s`,
    digitalTwinTrajectoryProgress: `${(state.progress * 100).toFixed(1)}%`,
    digitalTwinTrajectoryRate: `${state.playbackRate.toFixed(2)}x`,
    digitalTwinTrajectorySource: state.source,
    digitalTwinTrajectoryError: state.validationError || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });

  const scrubber = document.getElementById("digitalTwinTrajectoryScrubber");
  if (scrubber) {
    scrubber.max = String(state.durationS);
    scrubber.value = String(state.currentTimeS);
    scrubber.disabled = !state.trajectory;
    scrubber.setAttribute(
      "aria-valuenow",
      formatTrajectoryTime(state.currentTimeS, 1),
    );
  }
  const scrubberTime = document.getElementById(
    "digitalTwinTrajectoryScrubberTime",
  );
  if (scrubberTime) {
    scrubberTime.textContent = `${formatTrajectoryTime(state.currentTimeS, 1)} s`;
  }
  const timeInput = document.getElementById("digitalTwinTrajectoryTimeInput");
  if (timeInput) {
    timeInput.max = String(state.durationS);
    timeInput.value = formatTrajectoryTime(state.currentTimeS, 1);
    timeInput.disabled = !state.trajectory;
  }
  const rateSelector = document.getElementById("digitalTwinTrajectoryPlaybackRate");
  if (rateSelector && rateSelector.value !== String(state.playbackRate)) {
    rateSelector.value = String(state.playbackRate);
  }
  for (const id of [
    "digitalTwinValidateStoredPoints",
    "digitalTwinValidateJointLimits",
  ]) {
    const validationButton = document.getElementById(id);
    if (validationButton) validationButton.disabled = !state.trajectory;
  }
  updateObjectGlobalPlanUi();
}

function acceptedTrajectorySourceLabel(sourceLabel) {
  return typeof sourceLabel === "string" && sourceLabel.trim().length > 0
    ? sourceLabel.trim()
    : "LOCAL TRAJECTORY";
}

function cancelTrajectoryAnimation() {
  if (trajectoryPreviewState.animationFrameId !== null) {
    cancelAnimationFrame(trajectoryPreviewState.animationFrameId);
    trajectoryPreviewState.animationFrameId = null;
  }
  trajectoryPreviewState.previousFrameTimeMs = null;
}

function integratedPlanDurationS(plan) {
  if (!plan) return 0;
  return typeof plan.combined_duration_s === "number"
    && Number.isFinite(plan.combined_duration_s)
    ? plan.combined_duration_s
    : (typeof plan.duration_s === "number" && Number.isFinite(plan.duration_s)
      ? plan.duration_s : 0);
}

function integratedPlanPath(plan) {
  if (!plan) return [];
  return Array.isArray(plan.combined_path) && plan.combined_path.length
    ? plan.combined_path
    : (Array.isArray(plan.global_path) ? plan.global_path : []);
}

function integratedPlanStateSnapshot() {
  const plan = integratedPlanPreviewState.plan;
  const path = integratedPlanPath(plan);
  return {
    active: integratedPlanPreviewState.active,
    owner: integratedPlanPreviewState.owner,
    status: objectGlobalPlanState.status,
    currentTimeS: trajectoryPreviewState.currentTimeS,
    currentSegmentIndex: trajectoryPreviewState.currentSegmentIndex,
    currentAlpha: trajectoryPreviewState.currentAlpha,
    playing: trajectoryPreviewState.playing,
    playbackRate: trajectoryPreviewState.playbackRate,
    durationS: integratedPlanDurationS(plan),
    sampleCount: path.length,
    error: integratedPlanPreviewState.error || objectGlobalPlanState.error,
  };
}

function getIntegratedPlanPreviewState() {
  return integratedPlanStateSnapshot();
}

function globalPlanStateSnapshot() {
  const plan = objectGlobalPlanState.plan;
  return {
    status: objectGlobalPlanState.status,
    plan: plan ? normalizeObjectGlobalPlanResponse(plan) : null,
    error: objectGlobalPlanState.error,
    integratedPreview: integratedPlanStateSnapshot(),
  };
}

function getObjectGlobalPlanState() {
  return globalPlanStateSnapshot();
}

function phase4ValidationStateSnapshot() {
  const effectiveStart = resolveCurrentPhase4ValidationStartState();
  return {
    status: phase4TrajectoryValidationState.status,
    validating: phase4TrajectoryValidationState.validating,
    startState: effectiveStart.startState,
    startStateKind: effectiveStart.kind,
    explicitStartState: phase4TrajectoryValidationState.startState
      ? normalizePhase4ValidationStartState(phase4TrajectoryValidationState.startState)
      : null,
    positionToleranceM: phase4TrajectoryValidationState.positionToleranceM,
    orientationToleranceRad: phase4TrajectoryValidationState.orientationToleranceRad,
    result: phase4TrajectoryValidationState.result
      ? JSON.parse(JSON.stringify(phase4TrajectoryValidationState.result))
      : null,
    error: phase4TrajectoryValidationState.error,
  };
}

function getPhase4TrajectoryValidationState() {
  return phase4ValidationStateSnapshot();
}

function phase4Display(value, suffix = "") {
  return typeof value === "number" && Number.isFinite(value)
    ? `${value.toFixed(6)}${suffix}`
    : "NOT EVALUATED";
}

function hasFreshLiveActualForPhase4(nowMs = Date.now()) {
  return Boolean(
    latestActualPose
    && mirrorState.mode === MIRROR_MODES.LIVE_MIRROR
    && mirrorState.latestValidSnapshot
    && !isNormalizedSnapshotStale(
      mirrorState.latestValidSnapshot,
      nowMs,
      mirrorState.staleTimeoutMs,
    )
  );
}

function normalizeResolvedPhase4StartState(value, sourceOverride = null) {
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  const source = sourceOverride === null ? value.source : sourceOverride;
  if (typeof source !== "string" || !source.trim()) return null;
  const validSide = (side) => Array.isArray(side)
    && side.length === 6
    && side.every((item) => typeof item === "number" && Number.isFinite(item));
  if (!validSide(value.left) || !validSide(value.right)) return null;
  return {
    left: [...value.left],
    right: [...value.right],
    source: source.trim(),
  };
}

function resolvePhase4ValidationStartState({
  explicitStartState = null,
  explicitStartStateKind = "EXPLICIT OFFLINE",
  plan = null,
} = {}) {
  const hasExplicitOverride = explicitStartState !== null
    && explicitStartState !== undefined;
  const candidate = hasExplicitOverride
    ? normalizeResolvedPhase4StartState(explicitStartState)
    : normalizeResolvedPhase4StartState(
      plan && plan.ok === true ? plan.planning_start_state_rad : null,
      plan && plan.ok === true ? plan.planning_start_state_source : null,
    );
  if (!candidate) {
    return {
      available: false,
      startState: null,
      kind: "UNAVAILABLE",
      reason: "Validation start state unavailable.",
    };
  }
  return {
    available: true,
    startState: candidate,
    kind: hasExplicitOverride ? explicitStartStateKind : "PLANNING START STATE",
    reason: null,
  };
}

function phase4ValidationReadiness({
  planStatus = "EMPTY",
  plan = null,
  authorityMatches = false,
  validationRunning = false,
  explicitStartState = null,
  explicitStartStateKind = "EXPLICIT OFFLINE",
} = {}) {
  const effectiveStart = resolvePhase4ValidationStartState({
    explicitStartState,
    explicitStartStateKind,
    plan,
  });
  let reason = "Trajectory ready for validation.";
  if (validationRunning) {
    reason = "Trajectory validation is already running.";
  } else if (planStatus !== "READY" || !plan || plan.ok !== true) {
    reason = "Generate a trajectory, then validate it.";
  } else if (!authorityMatches) {
    reason = "Trajectory authority no longer matches the current setup.";
  } else if (!effectiveStart.available) {
    reason = effectiveStart.reason;
  }
  const ready = (
    !validationRunning
    && planStatus === "READY"
    && Boolean(plan && plan.ok === true)
    && authorityMatches
    && effectiveStart.available
  );
  return { ready, reason, effectiveStart };
}

function currentPlanMatchesPhase3Authority(plan = objectGlobalPlanState.plan) {
  const authority = phase3PlanningAuthorityIdentity();
  return Boolean(
    authority
    && plan
    && plan.ok === true
    && plan.grasp
    && plan.calibration
    && plan.grasp.content_revision === authority.graspContentRevision
    && plan.grasp.lock_generation === authority.lockGeneration
    && plan.grasp.lock_revision === authority.lockRevision
    && plan.calibration.revision === authority.calibrationRevision
    && plan.calibration.model_revision === authority.modelCalibrationRevision
  );
}

function resolveCurrentPhase4ValidationStartState(explicitStartState = (
  phase4TrajectoryValidationState.startState
)) {
  return resolvePhase4ValidationStartState({
    explicitStartState,
    explicitStartStateKind: phase4TrajectoryValidationState.startStateKind,
    plan: objectGlobalPlanState.plan,
  });
}

function currentPhase4ValidationReadiness({
  validationRunning = phase4UnifiedValidationState.validating,
  explicitStartState = phase4TrajectoryValidationState.startState,
} = {}) {
  return phase4ValidationReadiness({
    planStatus: objectGlobalPlanState.status,
    plan: objectGlobalPlanState.plan,
    authorityMatches: currentPlanMatchesPhase3Authority(),
    validationRunning,
    explicitStartState,
    explicitStartStateKind: phase4TrajectoryValidationState.startStateKind,
  });
}

function updatePhase4TrajectoryValidationUi() {
  const result = phase4TrajectoryValidationState.result;
  const readiness = currentPhase4ValidationReadiness({
    validationRunning: phase4TrajectoryValidationState.validating,
  });
  const fk = result && result.fk ? result.fk : {};
  const relative = result && result.relative_pose ? result.relative_pose : {};
  const velocity = result && result.velocity ? result.velocity : {};
  const acceleration = result && result.acceleration ? result.acceleration : {};
  const end = result && result.end_state ? result.end_state : {};
  const values = {
    digitalTwinPhase4Overall: phase4TrajectoryValidationState.validating
      ? "PHASE-4 VALIDATION IN PROGRESS"
      : phase4TrajectoryValidationState.status,
    digitalTwinPhase4StartState: readiness.effectiveStart.kind,
    digitalTwinPhase4CommonTimeline: result && result.timeline
      ? result.timeline.status : "NOT EVALUATED",
    digitalTwinPhase4StartTransition: result && result.start_transition
      ? result.start_transition.status : "NOT EVALUATED",
    digitalTwinPhase4EndState: end.status
      ? `${end.status} — SAMPLE ${end.sample_index}` : "NOT EVALUATED",
    digitalTwinPhase4FkTarget: fk.status || "NOT EVALUATED",
    digitalTwinPhase4LeftPositionError: phase4Display(fk.max_left_position_error_m, " m"),
    digitalTwinPhase4RightPositionError: phase4Display(fk.max_right_position_error_m, " m"),
    digitalTwinPhase4LeftOrientationError: phase4Display(fk.max_left_orientation_error_rad, " rad"),
    digitalTwinPhase4RightOrientationError: phase4Display(fk.max_right_orientation_error_rad, " rad"),
    digitalTwinPhase4RelativeTranslationError: phase4Display(relative.max_translation_error_m, " m"),
    digitalTwinPhase4RelativeOrientationError: phase4Display(relative.max_orientation_error_rad, " rad"),
    digitalTwinPhase4FixedGrasp: result && result.fixed_grasp
      ? result.fixed_grasp.status : "NOT EVALUATED",
    digitalTwinPhase4MaxVelocity: phase4Display(velocity.max_abs_velocity_rad_s, " rad/s"),
    digitalTwinPhase4VelocityLimit: Array.isArray(velocity.limits_rad_s)
      ? phase4Display(Math.max(...velocity.limits_rad_s), " rad/s")
      : "NOT EVALUATED",
    digitalTwinPhase4VelocityLimitSource: velocity.limit_source || "NOT EVALUATED",
    digitalTwinPhase4VelocityStatus: velocity.status || "NOT EVALUATED",
    digitalTwinPhase4MaxAcceleration: phase4Display(
      acceleration.max_abs_discrete_acceleration_rad_s2,
      " rad/s²",
    ),
    digitalTwinPhase4AccelerationLimitStatus: acceleration.limit_status || "NOT EVALUATED",
    digitalTwinPhase4AccelerationValidation: acceleration.limit_validation || "NOT EVALUATED",
    digitalTwinPhase4Error: phase4TrajectoryValidationState.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const validateButton = document.getElementById("digitalTwinValidatePhase4Trajectory");
  if (validateButton) {
    validateButton.disabled = !readiness.ready;
  }
  const captureButton = document.getElementById("digitalTwinPhase4UseActualStart");
  if (captureButton) {
    captureButton.disabled = !hasFreshLiveActualForPhase4();
  }
}

function setPhase4ValidationStartState(startState, kind = "EXPLICIT OFFLINE") {
  phase4TrajectoryValidationState.startState = normalizePhase4ValidationStartState(
    startState,
  );
  phase4TrajectoryValidationState.startStateKind = kind === "LIVE ACTUAL"
    ? "LIVE ACTUAL" : "EXPLICIT OFFLINE";
  phase4TrajectoryValidationState.status = "NOT_VALIDATED";
  phase4TrajectoryValidationState.result = null;
  phase4TrajectoryValidationState.error = null;
  invalidatePhase4UnifiedValidation("PHASE-4 START STATE CHANGED");
  updatePhase4TrajectoryValidationUi();
  return phase4ValidationStateSnapshot();
}

function clearPhase4ValidationStartState() {
  phase4TrajectoryValidationState.startState = null;
  phase4TrajectoryValidationState.startStateKind = "UNAVAILABLE";
  phase4TrajectoryValidationState.status = "NOT_VALIDATED";
  phase4TrajectoryValidationState.result = null;
  phase4TrajectoryValidationState.error = null;
  invalidatePhase4UnifiedValidation("PHASE-4 START STATE CLEARED");
  updatePhase4TrajectoryValidationUi();
  return phase4ValidationStateSnapshot();
}

function useCurrentActualJointsAsPhase4ValidationStart() {
  if (!hasFreshLiveActualForPhase4()) {
    phase4TrajectoryValidationState.error = "Fresh Live Actual joints are unavailable";
    updatePhase4TrajectoryValidationUi();
    return phase4ValidationStateSnapshot();
  }
  return setPhase4ValidationStartState({
    left: [...latestActualPose.left],
    right: [...latestActualPose.right],
    source: "LIVE_ACTUAL_MIRROR",
  }, "LIVE ACTUAL");
}

async function validateCurrentGlobalPlanPhase4(options = {}, fetchImpl = fetch) {
  try {
    const hasStartOverride = Object.prototype.hasOwnProperty.call(options, "startState");
    const readiness = currentPhase4ValidationReadiness({
      validationRunning: phase4TrajectoryValidationState.validating,
      explicitStartState: hasStartOverride
        ? options.startState : phase4TrajectoryValidationState.startState,
    });
    if (!readiness.ready) throw new Error(readiness.reason);
    const request = buildPhase4ValidationRequest({
      plan: objectGlobalPlanState.plan,
      startState: readiness.effectiveStart.startState,
      positionToleranceM: Object.prototype.hasOwnProperty.call(
        options,
        "positionToleranceM",
      ) ? options.positionToleranceM : phase4TrajectoryValidationState.positionToleranceM,
      orientationToleranceRad: Object.prototype.hasOwnProperty.call(
        options,
        "orientationToleranceRad",
      ) ? options.orientationToleranceRad : phase4TrajectoryValidationState.orientationToleranceRad,
    });
    phase4TrajectoryValidationState.validating = true;
    invalidatePhase4UnifiedValidation("PHASE-4A RESULT CHANGED");
    phase4TrajectoryValidationState.status = "VALIDATING";
    phase4TrajectoryValidationState.error = null;
    updatePhase4TrajectoryValidationUi();
    const result = await requestPhase4TrajectoryValidation(request, fetchImpl);
    phase4TrajectoryValidationState.result = result;
    phase4TrajectoryValidationState.status = result.status;
    return phase4ValidationStateSnapshot();
  } catch (error) {
    phase4TrajectoryValidationState.status = "ERROR";
    phase4TrajectoryValidationState.result = null;
    phase4TrajectoryValidationState.error = error && error.message
      ? error.message : "Phase-4 validation failed";
    return phase4ValidationStateSnapshot();
  } finally {
    phase4TrajectoryValidationState.validating = false;
    updatePhase4TrajectoryValidationUi();
  }
}

function bindPhase4TrajectoryValidationControls() {
  const validateButton = document.getElementById("digitalTwinValidatePhase4Trajectory");
  if (validateButton) validateButton.addEventListener("click", () => {
    validateCurrentGlobalPlanPhase4();
  });
  const actualButton = document.getElementById("digitalTwinPhase4UseActualStart");
  if (actualButton) actualButton.addEventListener(
    "click",
    useCurrentActualJointsAsPhase4ValidationStart,
  );
  const clearButton = document.getElementById("digitalTwinPhase4ClearStart");
  if (clearButton) clearButton.addEventListener("click", clearPhase4ValidationStartState);
}

function phase4CollisionStateSnapshot() {
  return {
    status: phase4CollisionValidationState.status,
    validating: phase4CollisionValidationState.validating,
    result: phase4CollisionValidationState.result
      ? JSON.parse(JSON.stringify(phase4CollisionValidationState.result))
      : null,
    error: phase4CollisionValidationState.error,
  };
}

function getPhase4CollisionValidationState() {
  return phase4CollisionStateSnapshot();
}

function phase4CollisionLocation(firstFailure) {
  if (!firstFailure) return "NONE";
  if (Number.isInteger(firstFailure.point_index)) {
    return `POINT ${firstFailure.point_index}`;
  }
  if (Number.isInteger(firstFailure.segment_index)) {
    return `SEGMENT ${firstFailure.segment_index} / SAMPLE ${
      firstFailure.sample_index === null || firstFailure.sample_index === undefined
        ? "N/A" : firstFailure.sample_index
    }`;
  }
  return firstFailure.location_kind || "UNKNOWN";
}

function updatePhase4CollisionValidationUi() {
  const result = phase4CollisionValidationState.result;
  const robot = result && result.robot_collision ? result.robot_collision : {};
  const objectScene = result && result.object_scene ? result.object_scene : {};
  const objectCollision = result && result.object_collision ? result.object_collision : {};
  const environmentScene = result && result.environment_scene
    ? result.environment_scene : {};
  const environmentCollision = result && result.environment_collision
    ? result.environment_collision : {};
  const summary = result && result.collision_summary ? result.collision_summary : {};
  const first = summary.first_failure || null;
  const values = {
    digitalTwinPhase4BCollisionOverall: phase4CollisionValidationState.validating
      ? "PHASE-4B VALIDATION IN PROGRESS" : phase4CollisionValidationState.status,
    digitalTwinPhase4BRobotStored: robot.stored_points
      ? robot.stored_points.status : "NOT EVALUATED",
    digitalTwinPhase4BRobotSampled: robot.sampled_path
      ? robot.sampled_path.status : "NOT EVALUATED",
    digitalTwinPhase4BSelfCollision: robot.self_collision
      ? robot.self_collision.status : "NOT EVALUATED",
    digitalTwinPhase4BInterArmCollision: robot.inter_arm_collision
      ? robot.inter_arm_collision.status : "NOT EVALUATED",
    digitalTwinPhase4BObjectScene: objectScene.status || "NOT CONFIGURED",
    digitalTwinPhase4BObjectCollision: objectCollision.status || "NOT EVALUATED",
    digitalTwinPhase4BEnvironmentScene: environmentScene.status || "NOT CONFIGURED",
    digitalTwinPhase4BEnvironmentCollision: environmentCollision.status || "NOT EVALUATED",
    digitalTwinPhase4BFirstCategory: first ? first.category : "NONE",
    digitalTwinPhase4BFirstPair: first
      ? `${first.body_1} ↔ ${first.body_2}` : "NONE",
    digitalTwinPhase4BFirstLocation: phase4CollisionLocation(first),
    digitalTwinPhase4BFirstAlphaTime: first
      ? `${phase4Display(first.alpha)} / ${phase4Display(first.time_from_start_s, " s")}`
      : "N/A",
    digitalTwinPhase4BPenetration: first
      ? phase4Display(first.depth_m, " m") : "N/A",
    digitalTwinPhase4BCollisionError: phase4CollisionValidationState.error
      || (result ? result.scene_error : null) || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const button = document.getElementById("digitalTwinValidatePhase4Collision");
  if (button) button.disabled = (
    phase4CollisionValidationState.validating
    || objectGlobalPlanState.status !== "READY"
    || !objectGlobalPlanState.plan
  );
}

async function validateCurrentGlobalPlanPhase4Collision(
  options = {},
  fetchImpl = fetch,
) {
  try {
    const stepInput = document.getElementById("digitalTwinSampledPathMaxJointStep");
    const defaultStep = stepInput ? stepInput.valueAsNumber : 0.05;
    const step = Object.prototype.hasOwnProperty.call(options, "maxJointStepRad")
      ? options.maxJointStepRad : defaultStep;
    const request = buildPhase4CollisionRequest(objectGlobalPlanState.plan, step);
    phase4CollisionValidationState.validating = true;
    invalidatePhase4UnifiedValidation("PHASE-4B RESULT CHANGED");
    phase4CollisionValidationState.status = "VALIDATING";
    phase4CollisionValidationState.error = null;
    updatePhase4CollisionValidationUi();
    const result = await requestPhase4CollisionValidation(request, fetchImpl);
    phase4CollisionValidationState.result = result;
    phase4CollisionValidationState.status = result.status;
    return phase4CollisionStateSnapshot();
  } catch (error) {
    phase4CollisionValidationState.status = "ERROR";
    phase4CollisionValidationState.result = null;
    phase4CollisionValidationState.error = error && error.message
      ? error.message : "Phase-4B collision validation failed";
    return phase4CollisionStateSnapshot();
  } finally {
    phase4CollisionValidationState.validating = false;
    updatePhase4CollisionValidationUi();
  }
}

function clearPhase4CollisionValidation() {
  phase4CollisionValidationState.status = "NOT_VALIDATED";
  phase4CollisionValidationState.result = null;
  phase4CollisionValidationState.error = null;
  invalidatePhase4UnifiedValidation("PHASE-4B RESULT CLEARED");
  updatePhase4CollisionValidationUi();
  return phase4CollisionStateSnapshot();
}

function bindPhase4CollisionValidationControls() {
  const validateButton = document.getElementById("digitalTwinValidatePhase4Collision");
  if (validateButton) validateButton.addEventListener("click", () => {
    validateCurrentGlobalPlanPhase4Collision();
  });
  const clearButton = document.getElementById("digitalTwinClearPhase4Collision");
  if (clearButton) clearButton.addEventListener("click", clearPhase4CollisionValidation);
}

function phase4UnifiedValidationStateSnapshot() {
  return {
    status: phase4UnifiedValidationState.status,
    validating: phase4UnifiedValidationState.validating,
    report: phase4UnifiedValidationState.report
      ? JSON.parse(JSON.stringify(phase4UnifiedValidationState.report))
      : null,
    error: phase4UnifiedValidationState.error,
    stale: phase4UnifiedValidationState.stale,
    invalidationReason: phase4UnifiedValidationState.invalidationReason,
    currentPlanFingerprint: phase4UnifiedValidationState.currentPlanFingerprint,
    executionGate: getExecutionGateState(),
  };
}

function getPhase4UnifiedValidationState() {
  return phase4UnifiedValidationStateSnapshot();
}

function getExecutionGateState() {
  const gate = phase4ExecutionGateState({
    report: phase4UnifiedValidationState.report,
    currentPlanFingerprint: phase4UnifiedValidationState.currentPlanFingerprint,
    validationRunning: phase4UnifiedValidationState.validating,
    stale: phase4UnifiedValidationState.stale,
  });
  const extraReasons = [];
  if (!worldCalibrationState.matches) {
    extraReasons.push(
      worldCalibrationState.status === MODEL_CALIBRATION_MISMATCH
        ? "MODEL_CALIBRATION_REVISION_MISMATCH"
        : "MODEL_CALIBRATION_REVISION_UNAVAILABLE",
    );
  }
  if (rigidGraspState.state !== "GRASP_LOCKED") {
    extraReasons.push("RIGID_GRASP_NOT_LOCKED");
  }
  const plan = objectGlobalPlanState.plan;
  if (plan && plan.ok && plan.grasp && plan.calibration
      && rigidGraspState.lockedSnapshot) {
    if (plan.grasp.content_revision
        !== rigidGraspState.lockedSnapshot.contentRevision) {
      extraReasons.push("PLAN_GRASP_REVISION_STALE");
    }
    if (plan.grasp.lock_generation !== rigidGraspState.lockGeneration) {
      extraReasons.push("PLAN_LOCK_GENERATION_STALE");
    }
    if (plan.calibration.revision !== worldCalibrationState.backendRevision) {
      extraReasons.push("PLAN_CALIBRATION_REVISION_STALE");
    }
  }
  if (extraReasons.length === 0) return gate;
  return {
    ...gate,
    executionReady: false,
    executionReadyLabel: "NO",
    status: "BLOCKED",
    blockingReasons: [...new Set([...gate.blockingReasons, ...extraReasons])],
  };
}

function getWorldCalibrationState() {
  return { ...worldCalibrationState };
}

function updateWorldCalibrationUi() {
  const values = {
    digitalTwinCalibrationState: worldCalibrationState.calibrationState,
    digitalTwinPhysicalCalibration: String(worldCalibrationState.physicallyCalibrated),
    digitalTwinBackendCalibrationRevision: worldCalibrationState.backendRevision || "UNAVAILABLE",
    digitalTwinModelCalibrationRevision: worldCalibrationState.modelRevision || "UNAVAILABLE",
    digitalTwinCalibrationRevisionStatus: worldCalibrationState.status,
    digitalTwinCalibrationRevisionError: worldCalibrationState.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const alignment = document.getElementById("digitalTwinOperatorModelAlignment");
  if (alignment) {
    alignment.textContent = worldCalibrationState.matches ? "OK" : (
      worldCalibrationState.status === MODEL_CALIBRATION_MISMATCH
        ? "MISMATCH" : "UNAVAILABLE"
    );
    alignment.dataset.state = worldCalibrationState.matches ? "ok" : "warning";
  }
  const physicalWarning = document.getElementById(
    "digitalTwinOperatorPhysicalCalibrationWarning",
  );
  if (physicalWarning) {
    physicalWarning.textContent = worldCalibrationState.physicallyCalibrated
      ? "Physical world calibration completed."
      : "Physical world calibration not completed — offline planning only.";
  }
}

async function loadWorldCalibrationState(fetchImpl = fetch) {
  const previousBackendRevision = worldCalibrationState.backendRevision;
  const previousModelRevision = worldCalibrationState.modelRevision;
  const previousMatches = worldCalibrationState.matches;
  try {
    Object.assign(worldCalibrationState, await requestCalibrationRevisionState(fetchImpl));
  } catch (error) {
    worldCalibrationState.status = "UNAVAILABLE";
    worldCalibrationState.matches = false;
    worldCalibrationState.planningReady = false;
    worldCalibrationState.error = error && error.message
      ? error.message : "World calibration revision inspection failed";
  }
  const authorityChanged = previousBackendRevision !== null && (
    previousBackendRevision !== worldCalibrationState.backendRevision
    || previousModelRevision !== worldCalibrationState.modelRevision
    || previousMatches !== worldCalibrationState.matches
  );
  if (authorityChanged || (!worldCalibrationState.matches && objectGlobalPlanState.plan)) {
    invalidatePlanningForGraspChange(
      worldCalibrationState.matches
        ? "WORLD CALIBRATION REVISION CHANGED"
        : "MODEL/CALIBRATION REVISION MISMATCH",
    );
  }
  updateWorldCalibrationUi();
  updateObjectWaypointPlanningUi();
  updatePhase4UnifiedValidationUi();
  return getWorldCalibrationState();
}

function unifiedCheckStatus(report, name) {
  return report && report.checks && report.checks[name]
    ? report.checks[name].status : "NOT EVALUATED";
}

function unifiedDeferredStatus(report, name) {
  return `DEFERRED — ${unifiedCheckStatus(report, name)}`;
}

function unifiedLocation(firstFailure) {
  if (!firstFailure || !firstFailure.location) return "NONE";
  const entries = Object.entries(firstFailure.location);
  return entries.length > 0
    ? entries.map(([key, value]) => `${key}=${value}`).join(", ")
    : "NONE";
}

function updatePhase4UnifiedValidationUi() {
  const report = phase4UnifiedValidationState.report;
  const first = report ? report.first_failure : null;
  const diagnostics = report && report.diagnostics ? report.diagnostics : {};
  const authoritative = diagnostics.authoritative_inputs || {};
  const collision = diagnostics.collision && diagnostics.collision.summary
    ? diagnostics.collision.summary : {};
  const collisionFirst = collision.first_failure || null;
  const jump = diagnostics.joint_jump || {};
  const jumpSummary = jump.summary || {};
  const relative = diagnostics.relative_pose || {};
  const fk = relative.fk_target || {};
  const relativePose = relative.relative_pose || {};
  const timing = diagnostics.timing_dynamics || {};
  const velocity = timing.velocity || {};
  const acceleration = timing.acceleration || {};
  const gate = getExecutionGateState();
  const readiness = currentPhase4ValidationReadiness();
  const values = {
    digitalTwinPhase4COverall: phase4UnifiedValidationState.validating
      ? "VALIDATING" : phase4UnifiedValidationState.status,
    digitalTwinPhase4CExecutionReady: gate.executionReadyLabel,
    digitalTwinPhase4CScopeStatus: gate.executionReady
      ? "READY UNDER CURRENT PROJECT SCOPE"
      : "NOT READY UNDER CURRENT PROJECT SCOPE",
    digitalTwinPhase4CPlanFingerprint: report ? report.plan_fingerprint : "NONE",
    digitalTwinPhase4CAuthoritativeInputs: unifiedCheckStatus(
      report, "authoritative_inputs",
    ),
    digitalTwinPhase4CGraspRevision: authoritative.grasp_content_revision || "NONE",
    digitalTwinPhase4CLockGeneration: authoritative.lock_generation === undefined
      ? "NONE" : String(authoritative.lock_generation),
    digitalTwinPhase4CCalibrationRevision: authoritative.calibration_revision || "NONE",
    digitalTwinPhase4CStale: String(gate.stale),
    digitalTwinPhase4CIk: unifiedCheckStatus(report, "ik_complete"),
    digitalTwinPhase4CJointLimits: unifiedCheckStatus(report, "joint_position_limits"),
    digitalTwinPhase4CSelfCollision: unifiedCheckStatus(report, "self_collision"),
    digitalTwinPhase4CInterArmCollision: unifiedCheckStatus(report, "inter_arm_collision"),
    digitalTwinPhase4CObjectCollision: unifiedDeferredStatus(report, "object_collision"),
    digitalTwinPhase4CEnvironmentCollision: unifiedDeferredStatus(
      report, "environment_collision",
    ),
    digitalTwinPhase4CSampledCollision: unifiedCheckStatus(report, "sampled_path_collision"),
    digitalTwinPhase4CStartTransition: unifiedCheckStatus(report, "start_transition"),
    digitalTwinPhase4CContinuity: unifiedCheckStatus(report, "joint_continuity"),
    digitalTwinPhase4CJump: unifiedCheckStatus(report, "joint_jump"),
    digitalTwinPhase4CFk: unifiedCheckStatus(report, "fk_target"),
    digitalTwinPhase4CRelativePose: unifiedCheckStatus(report, "relative_pose"),
    digitalTwinPhase4CFixedGrasp: unifiedCheckStatus(report, "fixed_grasp"),
    digitalTwinPhase4CTimeline: unifiedCheckStatus(report, "common_timeline"),
    digitalTwinPhase4CVelocity: unifiedCheckStatus(report, "velocity"),
    digitalTwinPhase4CAcceleration: unifiedDeferredStatus(report, "acceleration"),
    digitalTwinPhase4CFirstCheck: first ? first.check : "NONE",
    digitalTwinPhase4CFailureReason: first
      ? `${first.reason_code}: ${first.reason}` : "NONE",
    digitalTwinPhase4CFailureLocation: unifiedLocation(first),
    digitalTwinPhase4CCollision: collisionFirst
      ? `${collisionFirst.category}: ${collisionFirst.body_1} ↔ ${collisionFirst.body_2}`
      : "NONE",
    digitalTwinPhase4CJumpDetails: report
      ? `raw=${phase4Display(jumpSummary.maximum_abs_joint_step_rad, " rad")} `
        + `(${jumpSummary.maximum_joint_name || "N/A"}); shortest=`
        + `${phase4Display(jumpSummary.maximum_shortest_abs_joint_step_rad, " rad")} `
        + `(${jumpSummary.maximum_shortest_joint_name || "N/A"}); `
        + `suspicious=${jump.suspicious_transition_count === undefined
          ? "N/A" : jump.suspicious_transition_count}`
      : "NONE",
    digitalTwinPhase4CRelativeDetails: report
      ? `FK L/R pos=${phase4Display(fk.max_left_position_error_m, " m")}/`
        + `${phase4Display(fk.max_right_position_error_m, " m")}; relative=`
        + `${phase4Display(relativePose.max_translation_error_m, " m")}/`
        + `${phase4Display(relativePose.max_orientation_error_rad, " rad")}`
      : "NONE",
    digitalTwinPhase4CExpectedRelative: relative.fixed_grasp
      && Array.isArray(relative.fixed_grasp.expected_relative_translation_m)
      ? `translation=[${relative.fixed_grasp.expected_relative_translation_m.join(", ")}], `
        + "orientation=locked rotation matrix"
      : "NONE",
    digitalTwinPhase4CDynamicsDetails: report
      ? `velocity=${phase4Display(velocity.max_abs_velocity_rad_s, " rad/s")}; `
        + `acceleration=${phase4Display(
          acceleration.max_abs_discrete_acceleration_rad_s2, " rad/s²",
        )}; acceleration limit=${acceleration.limit_status || "NOT EVALUATED"}`
      : "NONE",
    digitalTwinPhase4CBlockingReasons: gate.blockingReasons.length > 0
      ? gate.blockingReasons.join(", ") : "NONE",
    digitalTwinPhase4CDeferredWarnings: report && report.deferred_warnings
      ? report.deferred_warnings.map((finding) => (
        `${finding.roadmap_item} ${finding.check}=${finding.status}`
      )).join(", ")
      : "P4.5, P4.6, P4.20 REMAIN DEFERRED AND UNVALIDATED",
    digitalTwinPhase4CError: phase4UnifiedValidationState.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const operatorStatus = document.getElementById("digitalTwinOperatorValidationStatus");
  if (operatorStatus) {
    if (phase4UnifiedValidationState.validating) {
      operatorStatus.textContent = "VALIDATING…";
    } else if (phase4UnifiedValidationState.stale) {
      operatorStatus.textContent = "STALE — VALIDATE AGAIN";
    } else if (phase4UnifiedValidationState.status === "PASS") {
      operatorStatus.textContent = "VALIDATION PASSED";
    } else if (["FAIL", "ERROR"].includes(phase4UnifiedValidationState.status)) {
      operatorStatus.textContent = "VALIDATION FAILED";
    } else {
      operatorStatus.textContent = "NOT VALIDATED";
    }
  }
  const operatorReady = document.getElementById("digitalTwinOperatorExecutionReady");
  if (operatorReady) operatorReady.textContent = gate.executionReadyLabel;
  const operatorReason = document.getElementById("digitalTwinOperatorValidationReason");
  if (operatorReason) {
    let reason = "Generate a trajectory, then validate it.";
    let isFailure = false;
    if (phase4UnifiedValidationState.error) {
      reason = phase4UnifiedValidationState.error;
      isFailure = true;
    } else if (first) {
      reason = first.reason || "Trajectory validation failed.";
      isFailure = true;
    } else if (phase4UnifiedValidationState.stale) {
      reason = "The trajectory or setup changed. Validate the current plan again.";
      isFailure = true;
    } else if (phase4UnifiedValidationState.status === "PASS" && gate.executionReady) {
      reason = "All validation checks in the current project scope passed.";
    } else if (phase4UnifiedValidationState.status === "PASS") {
      reason = "Validation passed, but an execution prerequisite is incomplete.";
      isFailure = true;
    } else {
      reason = readiness.reason;
      isFailure = !readiness.ready;
    }
    operatorReason.textContent = reason;
    operatorReason.dataset.empty = String(!isFailure);
  }
  const validateButton = document.getElementById("digitalTwinValidatePhase4Unified");
  if (validateButton) validateButton.disabled = !readiness.ready;
}

function invalidatePhase4UnifiedValidation(reason = "EXECUTION-RELEVANT INPUT CHANGED") {
  window.dispatchEvent(new CustomEvent("dual-arm-phase5-authority-invalidated", {
    detail: { reason },
  }));
  phase4UnifiedValidationState.generation += 1;
  phase4UnifiedValidationState.validating = false;
  phase4UnifiedValidationState.stale = phase4UnifiedValidationState.report !== null;
  phase4UnifiedValidationState.status = phase4UnifiedValidationState.stale
    ? "STALE" : "NOT_VALIDATED";
  phase4UnifiedValidationState.currentPlanFingerprint = null;
  phase4UnifiedValidationState.invalidationReason = reason;
  phase4UnifiedValidationState.error = null;
  updatePhase4UnifiedValidationUi();
  return phase4UnifiedValidationStateSnapshot();
}

function invalidateAllPhase4Validation(reason) {
  phase4TrajectoryValidationState.status = "NOT_VALIDATED";
  phase4TrajectoryValidationState.result = null;
  phase4TrajectoryValidationState.error = null;
  phase4CollisionValidationState.status = "NOT_VALIDATED";
  phase4CollisionValidationState.result = null;
  phase4CollisionValidationState.error = null;
  invalidatePhase4UnifiedValidation(reason);
  updatePhase4TrajectoryValidationUi();
  updatePhase4CollisionValidationUi();
  updateOperatorPlanStatus();
}

function clearPhase4UnifiedValidation() {
  window.dispatchEvent(new CustomEvent("dual-arm-phase5-authority-invalidated", {
    detail: { reason: "PHASE-4C VALIDATION CLEARED" },
  }));
  phase4UnifiedValidationState.generation += 1;
  phase4UnifiedValidationState.status = "NOT_VALIDATED";
  phase4UnifiedValidationState.validating = false;
  phase4UnifiedValidationState.report = null;
  phase4UnifiedValidationState.error = null;
  phase4UnifiedValidationState.stale = false;
  phase4UnifiedValidationState.invalidationReason = "CLEARED";
  phase4UnifiedValidationState.currentPlanFingerprint = null;
  updatePhase4UnifiedValidationUi();
  return phase4UnifiedValidationStateSnapshot();
}

async function validateCurrentGlobalPlanPhase4Unified(
  options = {},
  fetchImpl = fetch,
) {
  const generation = phase4UnifiedValidationState.generation + 1;
  phase4UnifiedValidationState.generation = generation;
  try {
    const hasStartOverride = Object.prototype.hasOwnProperty.call(options, "startState");
    const readiness = currentPhase4ValidationReadiness({
      explicitStartState: hasStartOverride
        ? options.startState : phase4TrajectoryValidationState.startState,
    });
    if (!readiness.ready) throw new Error(readiness.reason);
    const stepInput = document.getElementById("digitalTwinSampledPathMaxJointStep");
    const request = buildPhase4UnifiedValidationRequest({
      plan: objectGlobalPlanState.plan,
      startState: readiness.effectiveStart.startState,
      positionToleranceM: Object.prototype.hasOwnProperty.call(
        options, "positionToleranceM",
      ) ? options.positionToleranceM : phase4TrajectoryValidationState.positionToleranceM,
      orientationToleranceRad: Object.prototype.hasOwnProperty.call(
        options, "orientationToleranceRad",
      ) ? options.orientationToleranceRad : phase4TrajectoryValidationState.orientationToleranceRad,
      maxJointStepRad: Object.prototype.hasOwnProperty.call(options, "maxJointStepRad")
        ? options.maxJointStepRad : (stepInput ? stepInput.valueAsNumber : 0.05),
    });
    phase4UnifiedValidationState.validating = true;
    phase4UnifiedValidationState.status = "VALIDATING";
    phase4UnifiedValidationState.report = null;
    phase4UnifiedValidationState.error = null;
    phase4UnifiedValidationState.stale = false;
    phase4UnifiedValidationState.currentPlanFingerprint = null;
    phase4UnifiedValidationState.invalidationReason = "VALIDATION RUNNING";
    updatePhase4UnifiedValidationUi();
    const result = await requestPhase4UnifiedValidation(request, fetchImpl);
    if (generation !== phase4UnifiedValidationState.generation) {
      return phase4UnifiedValidationStateSnapshot();
    }
    phase4UnifiedValidationState.report = result;
    phase4UnifiedValidationState.status = result.overall_status;
    phase4UnifiedValidationState.currentPlanFingerprint = result.plan_fingerprint;
    phase4UnifiedValidationState.invalidationReason = null;
    const components = result.components || {};
    if (components.phase4a) {
      phase4TrajectoryValidationState.result = components.phase4a;
      phase4TrajectoryValidationState.status = components.phase4a.status;
    }
    if (components.phase4b) {
      phase4CollisionValidationState.result = components.phase4b;
      phase4CollisionValidationState.status = components.phase4b.status;
    }
    return phase4UnifiedValidationStateSnapshot();
  } catch (error) {
    if (generation !== phase4UnifiedValidationState.generation) {
      return phase4UnifiedValidationStateSnapshot();
    }
    phase4UnifiedValidationState.status = "ERROR";
    phase4UnifiedValidationState.report = null;
    phase4UnifiedValidationState.currentPlanFingerprint = null;
    phase4UnifiedValidationState.error = error && error.message
      ? error.message : "Phase-4C unified validation failed";
    return phase4UnifiedValidationStateSnapshot();
  } finally {
    if (generation === phase4UnifiedValidationState.generation) {
      phase4UnifiedValidationState.validating = false;
      updatePhase4TrajectoryValidationUi();
      updatePhase4CollisionValidationUi();
      updatePhase4UnifiedValidationUi();
    }
  }
}

function bindPhase4UnifiedValidationControls() {
  const validateButton = document.getElementById("digitalTwinValidatePhase4Unified");
  if (validateButton) validateButton.addEventListener("click", () => {
    validateCurrentGlobalPlanPhase4Unified();
  });
  const clearButton = document.getElementById("digitalTwinClearPhase4Unified");
  if (clearButton) clearButton.addEventListener("click", clearPhase4UnifiedValidation);
}

function diagnosticValue(value, suffix = "") {
  return typeof value === "number" && Number.isFinite(value)
    ? `${value.toFixed(6)}${suffix}`
    : "UNAVAILABLE";
}

function updateObjectGlobalPlanUi() {
  const plan = objectGlobalPlanState.plan;
  const failure = plan && plan.failure ? plan.failure : null;
  const maximum = plan && plan.global
    ? plan.global.maximum_raw_single_joint_transition
    : null;
  const startToFirstMaximum = plan
    ? plan.start_to_first_maximum_raw_joint_delta : null;
  const values = {
    digitalTwinGlobalPlanState: objectGlobalPlanState.status,
    digitalTwinGlobalPlanSource: plan ? plan.planner_source : PHASE3_WEB_PLANNER_SOURCE,
    digitalTwinGlobalPlanGraspState: plan && plan.grasp
      ? plan.grasp.status : rigidGraspState.state,
    digitalTwinGlobalPlanGraspRevision: plan && plan.grasp
      ? plan.grasp.content_revision
      : (rigidGraspState.lockedSnapshot
        && rigidGraspState.lockedSnapshot.contentRevision
        ? rigidGraspState.lockedSnapshot.contentRevision : "UNAVAILABLE"),
    digitalTwinGlobalPlanLockGeneration: plan && plan.grasp
      ? String(plan.grasp.lock_generation) : String(rigidGraspState.lockGeneration),
    digitalTwinGlobalPlanCalibrationRevision: plan && plan.calibration
      ? plan.calibration.revision
      : (worldCalibrationState.backendRevision || "UNAVAILABLE"),
    digitalTwinGlobalPlanModelCalibration: plan && plan.calibration
      ? plan.calibration.revision_status : worldCalibrationState.status,
    digitalTwinGlobalPlanGraspSource: plan && plan.grasp
      ? plan.grasp.source : "LOCKED OPERATOR GRASP REQUIRED",
    digitalTwinGlobalPlanPlanningStartSource: plan
      ? String(plan.planning_start_state_source || planningStartState.source)
      : planningStartState.source,
    digitalTwinGlobalPlanStartToFirstMax: startToFirstMaximum
      ? diagnosticValue(startToFirstMaximum.value_rad, " rad")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanStartToFirstJoint: startToFirstMaximum
      ? `${startToFirstMaximum.joint_name || "NONE"} / ${
        startToFirstMaximum.joint_index === null
          || startToFirstMaximum.joint_index === undefined
          ? "NONE" : startToFirstMaximum.joint_index
      }`
      : "UNAVAILABLE",
    digitalTwinGlobalPlanOptimality: plan ? plan.optimality_scope : PHASE3_WEB_OPTIMALITY_SCOPE,
    digitalTwinGlobalPlanSampleCount: plan && plan.object_sample_count !== undefined
      ? String(plan.object_sample_count)
      : "0",
    digitalTwinGlobalPlanWaypointCount: plan && plan.object_waypoint_count !== undefined
      ? String(plan.object_waypoint_count)
      : String(objectWaypointPlanningState.waypoints.length),
    digitalTwinGlobalPlanDuration: plan
      ? `${integratedPlanDurationS(plan).toFixed(2)} s`
      : "0.00 s",
    digitalTwinGlobalPlanBaselineDuration: plan
      && Number.isFinite(plan.segment_duration_s)
      ? `${plan.segment_duration_s.toFixed(3)} s` : "UNAVAILABLE",
    digitalTwinGlobalPlanSegmentDurations: plan
      && Array.isArray(plan.segment_durations_s)
      ? plan.segment_durations_s.map((value) => `${value.toFixed(3)} s`).join(" → ")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanTimingProfiles: plan
      && Array.isArray(plan.segment_timing)
      ? plan.segment_timing.map((item) => (
        `${item.profile_source_waypoint}: ${item.speed_percent}% speed / `
        + `${item.acceleration_percent}% acceleration`
      )).join(" · ")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanLayerCount: plan && plan.graph
      ? String(plan.graph.layer_count === undefined ? 0 : plan.graph.layer_count)
      : "0",
    digitalTwinGlobalPlanCandidateAttempts: plan
      && typeof plan.candidate_attempts_per_arm === "number"
      ? String(plan.candidate_attempts_per_arm)
      : String(objectWaypointPlanningState.candidateAttemptsPerArm),
    digitalTwinGlobalPlanExplorationProfile: plan
      ? String(plan.candidate_exploration_profile || "UNAVAILABLE")
      : "WEB RUNTIME — BOUNDED IK SEED EXPLORATION",
    digitalTwinGlobalPlanCandidatePruning: plan
      && typeof plan.candidate_pruning_applied === "boolean"
      ? String(plan.candidate_pruning_applied)
      : "false",
    digitalTwinGlobalPlanPlanningElapsed: plan
      && typeof plan.planning_elapsed_s === "number"
      ? `${plan.planning_elapsed_s.toFixed(3)} s`
      : "UNAVAILABLE",
    digitalTwinGlobalPlanLayerCounts: plan && plan.graph
      ? (plan.graph.node_counts_per_layer || []).join(" → ") || "NONE"
      : "NONE",
    digitalTwinGlobalPlanSelectedNodes: plan && plan.graph
      ? (plan.graph.selected_node_ids || []).join(" → ") || "NONE"
      : "NONE",
    digitalTwinGlobalPlanGlobalCost: plan && plan.global
      ? diagnosticValue(plan.global.total_cost_rad2, " rad²")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanGreedyCost: plan && plan.greedy
      ? diagnosticValue(plan.greedy.total_cost_rad2, " rad²")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanCostDifference: plan && plan.comparison
      ? diagnosticValue(plan.comparison.cost_difference_rad2, " rad²")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanComplete: plan && plan.global
      ? (plan.global.completed ? "YES" : "NO")
      : "NO",
    digitalTwinGlobalPlanNotWorse: plan && plan.comparison
      && typeof plan.comparison.global_not_worse === "boolean"
      ? (plan.comparison.global_not_worse ? "YES" : "NO")
      : "UNAVAILABLE",
    digitalTwinGlobalPlanMaxTransition: maximum
      ? `${diagnosticValue(maximum.value_rad, " rad")} (${maximum.joint_name || "NONE"})`
      : "UNAVAILABLE",
    digitalTwinGlobalPlanMaxTransitionJoint: maximum
      ? `${maximum.joint_name || "NONE"} / ${
        maximum.joint_index === null || maximum.joint_index === undefined
          ? "NONE"
          : maximum.joint_index
      }`
      : "UNAVAILABLE",
    digitalTwinGlobalPlanFailedSample: failure && failure.failed_sample_index !== null
      && failure.failed_sample_index !== undefined
      ? String(failure.failed_sample_index)
      : "NONE",
    digitalTwinGlobalPlanFailureReason: failure
      ? `${failure.reason || "UNKNOWN"}: ${failure.message || ""}`.trim()
      : "NONE",
    digitalTwinGlobalPlanError: objectGlobalPlanState.error || "NONE",
    digitalTwinIntegratedPreviewOwner: integratedPlanPreviewState.owner,
    digitalTwinIntegratedPreviewTime: `${trajectoryPreviewState.currentTimeS.toFixed(2)} s`,
    digitalTwinIntegratedPreviewSegment: integratedPlanPreviewState.active
      ? `${trajectoryPreviewState.currentSegmentIndex + 1}/${Math.max(
        integratedPlanPath(plan).length - 1,
        1,
      )}`
      : "NONE",
    digitalTwinIntegratedPreviewAlpha: integratedPlanPreviewState.active
      ? trajectoryPreviewState.currentAlpha.toFixed(3)
      : "0.000",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const scrubber = document.getElementById("digitalTwinGlobalPlanScrubber");
  if (scrubber) {
    scrubber.max = String(integratedPlanDurationS(plan));
    scrubber.value = String(trajectoryPreviewState.currentTimeS);
    scrubber.disabled = !integratedPlanPreviewState.active;
  }
  const rate = document.getElementById("digitalTwinGlobalPlanPlaybackRate");
  if (rate && rate.value !== String(trajectoryPreviewState.playbackRate)) {
    rate.value = String(trajectoryPreviewState.playbackRate);
  }
  updatePhase4TrajectoryValidationUi();
  updatePhase4CollisionValidationUi();
}

function deactivateIntegratedPlanPreview(owner = "NONE") {
  integratedPlanPreviewState.active = false;
  integratedPlanPreviewState.owner = owner;
  integratedPlanPreviewState.plan = null;
  integratedPlanPreviewState.error = null;
  updateObjectGlobalPlanUi();
}

function applyIntegratedPlanTime(timeSeconds) {
  if (!integratedPlanPreviewState.active || !integratedPlanPreviewState.plan) return;
  const sample = sampleIntegratedObjectGlobalPlan(
    integratedPlanPreviewState.plan,
    timeSeconds,
  );
  applyObjectPreviewPose(sample.objectPose);
  integratedPlanPreviewState.owner = "INTEGRATED GLOBAL PLAN CLOCK";
  integratedPlanPreviewState.error = null;
}

function applyTrajectoryTime(timeSeconds, nextStatus = null, showGhost = false) {
  const trajectory = trajectoryPreviewState.trajectory;
  if (!trajectory) return false;
  const durationS = trajectoryDurationSeconds(trajectory);
  const clampedTimeS = Math.min(Math.max(timeSeconds, 0), durationS);
  const sample = sampleTrajectoryAtTime(trajectory, clampedTimeS);
  const preserveHidden = !showGhost && !plannedPreviewState.visible;
  setPlannedJointValues(
    { left: sample.left, right: sample.right },
    trajectoryPreviewState.source,
  );
  if (trajectoryPreviewState.syncIntegratedPlan) {
    applyIntegratedPlanTime(sample.time_from_start_s);
  }
  if (preserveHidden) hidePlannedModel();
  trajectoryPreviewState.currentTimeS = sample.time_from_start_s;
  trajectoryPreviewState.currentSegmentIndex = sample.segmentIndex;
  trajectoryPreviewState.currentPointIndex = sample.segmentIndex;
  if (sample.alpha === 1 && sample.time_from_start_s === durationS) {
    trajectoryPreviewState.currentPointIndex = trajectory.points.length - 1;
  }
  trajectoryPreviewState.currentAlpha = sample.alpha;
  trajectoryPreviewState.validationError = null;
  if (nextStatus) trajectoryPreviewState.status = nextStatus;
  updateTrajectoryPreviewUi();
  updateObjectGlobalPlanUi();
  return true;
}

function loadPlannedTrajectory(
  trajectory,
  sourceLabel = "LOCAL TRAJECTORY",
  { preserveIntegratedPlan = false, syncIntegratedPlan = true } = {},
) {
  if (!preserveIntegratedPlan) deactivateIntegratedPlanPreview("PLANNED TRAJECTORY");
  let normalized;
  try {
    normalized = normalizeDualArmTrajectory(trajectory);
  } catch (error) {
    cancelTrajectoryAnimation();
    trajectoryPreviewState.playing = false;
    trajectoryPreviewState.status = "INVALID";
    trajectoryPreviewState.validationError = error && error.message
      ? error.message
      : "Invalid planned trajectory";
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }

  cancelTrajectoryAnimation();
  trajectoryPreviewState.trajectory = normalized;
  trajectoryPreviewState.currentTimeS = 0;
  trajectoryPreviewState.currentPointIndex = 0;
  trajectoryPreviewState.currentSegmentIndex = 0;
  trajectoryPreviewState.currentAlpha = 0;
  trajectoryPreviewState.playing = false;
  trajectoryPreviewState.source = acceptedTrajectorySourceLabel(sourceLabel);
  trajectoryPreviewState.validationError = null;
  trajectoryPreviewState.syncIntegratedPlan = syncIntegratedPlan !== false;
  clearTrajectoryValidation();
  applyTrajectoryTime(0, "READY", true);
  return trajectoryStateSnapshot();
}

function setTrajectoryTime(timeSeconds) {
  if (!trajectoryPreviewState.trajectory) {
    trajectoryPreviewState.status = "EMPTY";
    trajectoryPreviewState.validationError = "No planned trajectory is loaded";
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }
  if (typeof timeSeconds !== "number" || !Number.isFinite(timeSeconds)) {
    trajectoryPreviewState.status = "INVALID";
    trajectoryPreviewState.validationError = "Trajectory time must be a finite number";
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }

  const nextStatus = trajectoryPreviewState.playing ? "PLAYING" : "PAUSED";
  applyTrajectoryTime(timeSeconds, nextStatus);
  if (trajectoryPreviewState.playing) {
    trajectoryPreviewState.previousFrameTimeMs = performance.now();
  }
  return trajectoryStateSnapshot();
}

function setTrajectoryPointIndex(index) {
  if (!trajectoryPreviewState.trajectory) {
    trajectoryPreviewState.status = "EMPTY";
    trajectoryPreviewState.validationError = "No planned trajectory is loaded";
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }
  try {
    const point = trajectoryPointAtIndex(trajectoryPreviewState.trajectory, index);
    return setTrajectoryTime(point.time_from_start_s);
  } catch (error) {
    trajectoryPreviewState.status = "INVALID";
    trajectoryPreviewState.validationError = error.message;
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }
}

function scheduleTrajectoryFrame() {
  if (
    !trajectoryPreviewState.playing
    || trajectoryPreviewState.animationFrameId !== null
  ) return;
  trajectoryPreviewState.animationFrameId = requestAnimationFrame(
    advanceTrajectoryPlayback,
  );
}

function advanceTrajectoryPlayback(frameTimeMs) {
  trajectoryPreviewState.animationFrameId = null;
  if (!trajectoryPreviewState.playing || !trajectoryPreviewState.trajectory) return;
  const previousTimeMs = trajectoryPreviewState.previousFrameTimeMs;
  trajectoryPreviewState.previousFrameTimeMs = frameTimeMs;
  const elapsedSeconds = previousTimeMs === null
    ? 0
    : Math.max(0, (frameTimeMs - previousTimeMs) / 1000);
  const durationS = trajectoryDurationSeconds(trajectoryPreviewState.trajectory);
  const nextTimeS = trajectoryPreviewState.currentTimeS
    + elapsedSeconds * trajectoryPreviewState.playbackRate;

  if (nextTimeS >= durationS) {
    applyTrajectoryTime(durationS, "FINISHED");
    trajectoryPreviewState.playing = false;
    trajectoryPreviewState.previousFrameTimeMs = null;
    updateTrajectoryPreviewUi();
    return;
  }

  applyTrajectoryTime(nextTimeS, "PLAYING");
  scheduleTrajectoryFrame();
}

function playPlannedTrajectory() {
  if (!trajectoryPreviewState.trajectory) {
    trajectoryPreviewState.status = "EMPTY";
    trajectoryPreviewState.validationError = "No planned trajectory is loaded";
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }
  if (trajectoryPreviewState.playing) return trajectoryStateSnapshot();

  const durationS = trajectoryDurationSeconds(trajectoryPreviewState.trajectory);
  if (trajectoryPreviewState.currentTimeS >= durationS) {
    applyTrajectoryTime(0, "READY");
  }
  trajectoryPreviewState.playing = true;
  trajectoryPreviewState.status = "PLAYING";
  trajectoryPreviewState.validationError = null;
  trajectoryPreviewState.previousFrameTimeMs = performance.now();
  updateTrajectoryPreviewUi();
  scheduleTrajectoryFrame();
  return trajectoryStateSnapshot();
}

function pausePlannedTrajectory() {
  if (!trajectoryPreviewState.trajectory) return trajectoryStateSnapshot();
  cancelTrajectoryAnimation();
  trajectoryPreviewState.playing = false;
  trajectoryPreviewState.status = "PAUSED";
  updateTrajectoryPreviewUi();
  return trajectoryStateSnapshot();
}

function stopPlannedTrajectory() {
  if (!trajectoryPreviewState.trajectory) return trajectoryStateSnapshot();
  cancelTrajectoryAnimation();
  trajectoryPreviewState.playing = false;
  applyTrajectoryTime(0, "READY", true);
  return trajectoryStateSnapshot();
}

function clearPlannedTrajectory() {
  deactivateIntegratedPlanPreview("NONE");
  cancelTrajectoryAnimation();
  trajectoryPreviewState.status = "EMPTY";
  trajectoryPreviewState.trajectory = null;
  trajectoryPreviewState.currentTimeS = 0;
  trajectoryPreviewState.currentPointIndex = 0;
  trajectoryPreviewState.currentSegmentIndex = 0;
  trajectoryPreviewState.currentAlpha = 0;
  trajectoryPreviewState.playing = false;
  trajectoryPreviewState.source = "NONE";
  trajectoryPreviewState.validationError = null;
  trajectoryPreviewState.syncIntegratedPlan = true;
  clearTrajectoryValidation();
  clearPlannedPreview();
  updateTrajectoryPreviewUi();
  return trajectoryStateSnapshot();
}

function setTrajectoryPlaybackRate(rate) {
  if (
    typeof rate !== "number"
    || !Number.isFinite(rate)
    || rate < 0.1
    || rate > 4
  ) {
    trajectoryPreviewState.validationError = (
      "Trajectory playback rate must be a finite number from 0.1x to 4.0x"
    );
    updateTrajectoryPreviewUi();
    return trajectoryStateSnapshot();
  }
  trajectoryPreviewState.playbackRate = rate;
  trajectoryPreviewState.validationError = null;
  if (trajectoryPreviewState.playing) {
    trajectoryPreviewState.previousFrameTimeMs = performance.now();
  }
  updateTrajectoryPreviewUi();
  return trajectoryStateSnapshot();
}

function getTrajectoryPreviewState() {
  return trajectoryStateSnapshot();
}

function trajectoryValidationStateSnapshot() {
  const copyMoveItPoint = (point) => point
    ? {
      ...point,
      contacts: Array.isArray(point.contacts)
        ? point.contacts.map((contact) => ({ ...contact }))
        : [],
    }
    : null;
  return {
    status: trajectoryValidationState.status,
    jointLimits: trajectoryValidationState.jointLimits,
    moveitStateValidity: trajectoryValidationState.moveitStateValidity,
    collision: trajectoryValidationState.collision,
    valid: trajectoryValidationState.valid,
    checkedPointCount: trajectoryValidationState.checkedPointCount,
    checkedJointCount: trajectoryValidationState.checkedJointCount,
    violationCount: trajectoryValidationState.violationCount,
    violations: trajectoryValidationState.violations.map(
      (violation) => ({ ...violation }),
    ),
    firstViolation: trajectoryValidationState.firstViolation
      ? { ...trajectoryValidationState.firstViolation }
      : null,
    moveitCheckedPointCount: trajectoryValidationState.moveitCheckedPointCount,
    moveitFailedPointCount: trajectoryValidationState.moveitFailedPointCount,
    firstMoveItFailedPoint: copyMoveItPoint(
      trajectoryValidationState.firstMoveItFailedPoint,
    ),
    firstCollisionPair: trajectoryValidationState.firstCollisionPair
      ? { ...trajectoryValidationState.firstCollisionPair }
      : null,
    collisionPairCount: trajectoryValidationState.collisionPairCount,
    maxPenetrationDepthM: trajectoryValidationState.maxPenetrationDepthM,
    sampledPathStatus: trajectoryValidationState.sampledPathStatus,
    sampledPathMaxJointStepRad: trajectoryValidationState.sampledPathMaxJointStepRad,
    sampledPathSegmentCount: trajectoryValidationState.sampledPathSegmentCount,
    sampledPathGeneratedSampleCount: (
      trajectoryValidationState.sampledPathGeneratedSampleCount
    ),
    sampledPathCheckedSampleCount: trajectoryValidationState.sampledPathCheckedSampleCount,
    sampledPathFailedSampleCount: trajectoryValidationState.sampledPathFailedSampleCount,
    firstSampledPathFailure: copyMoveItPoint(
      trajectoryValidationState.firstSampledPathFailure,
    ),
    sampledPathCollisionPairCount: trajectoryValidationState.sampledPathCollisionPairCount,
    sampledPathFirstCollisionPair: trajectoryValidationState.sampledPathFirstCollisionPair
      ? { ...trajectoryValidationState.sampledPathFirstCollisionPair }
      : null,
    sampledPathMaxPenetrationDepthM: (
      trajectoryValidationState.sampledPathMaxPenetrationDepthM
    ),
    source: trajectoryValidationState.source,
    error: trajectoryValidationState.error,
  };
}

function overallTrajectoryValidationLabel() {
  if (trajectoryValidationState.jointLimits === "FAIL") return "INVALID";
  if (trajectoryValidationState.moveitStateValidity === "CHECKING") {
    return "CHECKING — MOVEIT STORED POINTS";
  }
  if (trajectoryValidationState.moveitStateValidity === "UNAVAILABLE") {
    return "INCOMPLETE — MOVEIT UNAVAILABLE";
  }
  if (trajectoryValidationState.moveitStateValidity === "TIMEOUT") {
    return "INCOMPLETE — MOVEIT TIMEOUT";
  }
  if (trajectoryValidationState.moveitStateValidity === "ERROR") {
    return "INCOMPLETE — MOVEIT ERROR";
  }
  if (
    trajectoryValidationState.moveitStateValidity === "FAIL"
    && trajectoryValidationState.collision === "FAIL"
  ) return "INVALID — COLLISION DETECTED";
  if (trajectoryValidationState.moveitStateValidity === "FAIL") {
    return "INVALID — STATE INVALID (NO COLLISION CONTACT RETURNED)";
  }
  if (trajectoryValidationState.sampledPathStatus === "CHECKING") {
    return "CHECKING — BETWEEN-POINT SAMPLED PATH";
  }
  if (trajectoryValidationState.sampledPathStatus === "TIMEOUT") {
    return "INCOMPLETE — SAMPLED PATH TIMEOUT";
  }
  if (trajectoryValidationState.sampledPathStatus === "UNAVAILABLE") {
    return "INCOMPLETE — SAMPLED PATH UNAVAILABLE";
  }
  if (
    trajectoryValidationState.sampledPathStatus === "ERROR"
    || trajectoryValidationState.sampledPathStatus === "SKIPPED"
  ) return "INCOMPLETE — SAMPLED PATH NOT COMPLETED";
  if (
    trajectoryValidationState.sampledPathStatus === "FAIL"
    && trajectoryValidationState.sampledPathCollisionPairCount > 0
  ) return "INVALID — SAMPLED PATH COLLISION DETECTED";
  if (trajectoryValidationState.sampledPathStatus === "FAIL") {
    return "INVALID — SAMPLED STATE INVALID (NO COLLISION CONTACT RETURNED)";
  }
  if (
    trajectoryValidationState.moveitStateValidity === "PASS"
    && trajectoryValidationState.sampledPathStatus === "PASS"
  ) return "VALIDATED SAMPLED PATH — DISCRETE CHECK";
  if (
    trajectoryValidationState.jointLimits === "PASS"
    && trajectoryValidationState.moveitStateValidity === "PASS"
  ) return "VALIDATED STORED POINTS — JOINT LIMITS + MOVEIT COLLISION";
  if (
    trajectoryValidationState.status === "VALID"
    && trajectoryValidationState.jointLimits === "PASS"
  ) {
    return "PARTIAL PASS — JOINT LIMITS PASS — MOVEIT PENDING";
  }
  if (trajectoryValidationState.status === "INVALID") return "INVALID";
  if (trajectoryValidationState.status === "VALIDATING") return "VALIDATING";
  if (trajectoryValidationState.status === "ERROR") return "ERROR";
  return "NOT VALIDATED";
}

function validationSourceLabel(source) {
  if (typeof source !== "string" || source.trim().length === 0) return "NONE";
  const [logicalSource] = source.trim().split(" — ", 1);
  return logicalSource || "NONE";
}

function validationModelPath(source) {
  if (typeof source !== "string") return null;
  const separator = " — ";
  const separatorIndex = source.indexOf(separator);
  if (separatorIndex < 0) return null;
  const modelPath = source.slice(separatorIndex + separator.length).trim();
  return modelPath || null;
}

function validationPathBasename(path) {
  if (typeof path !== "string" || path.trim().length === 0) return "UNAVAILABLE";
  const segments = path.trim().replace(/\\/g, "/").split("/").filter(Boolean);
  return segments.length > 0 ? segments[segments.length - 1] : "UNAVAILABLE";
}

function validationFailedPointLabel(firstViolation, checkedPointCount) {
  if (!firstViolation) return "NONE";
  const pointIndex = firstViolation.pointIndex;
  if (
    !Number.isInteger(pointIndex)
    || pointIndex < 0
    || !Number.isInteger(checkedPointCount)
    || checkedPointCount <= pointIndex
  ) return "UNAVAILABLE";
  return `Point ${pointIndex + 1} of ${checkedPointCount}`;
}

function moveitFailedPointLabel(firstFailedPoint, checkedPointCount) {
  if (!firstFailedPoint) return "NONE";
  const pointIndex = firstFailedPoint.point_index;
  if (
    !Number.isInteger(pointIndex)
    || pointIndex < 0
    || !Number.isInteger(checkedPointCount)
    || checkedPointCount <= pointIndex
  ) return "UNAVAILABLE";
  return `Point ${pointIndex + 1} of ${checkedPointCount}`;
}

function collisionPairLabel(pair) {
  return pair && typeof pair.body_1 === "string" && typeof pair.body_2 === "string"
    ? `${pair.body_1} ↔ ${pair.body_2}`
    : "NONE";
}

function penetrationDepthLabel(depthM) {
  return typeof depthM === "number" && Number.isFinite(depthM)
    ? `${(depthM * 1000).toFixed(3)} mm (${depthM.toFixed(6)} m)`
    : "NONE";
}

function sampledPathSegmentLabel(failure, segmentCount) {
  if (!failure || !Number.isInteger(failure.segment_index)) return "NONE";
  if (failure.segment_index < 0 || failure.segment_index >= segmentCount) {
    return "UNAVAILABLE";
  }
  return `Segment ${failure.segment_index + 1} of ${segmentCount}`;
}

function sampledPathSampleLabel(failure) {
  if (
    !failure
    || !Number.isInteger(failure.sample_index)
    || !Number.isInteger(failure.subdivision_count)
    || failure.sample_index < 1
    || failure.sample_index >= failure.subdivision_count
  ) return failure ? "UNAVAILABLE" : "NONE";
  return (
    `Interior Sample ${failure.sample_index} of `
    + `${failure.subdivision_count - 1} (subdivision k=${failure.sample_index})`
  );
}

function sampledPathAlphaLabel(failure) {
  return failure && typeof failure.alpha === "number" && Number.isFinite(failure.alpha)
    ? failure.alpha.toFixed(3)
    : "N/A";
}

function sampledPathTimeLabel(failure) {
  return failure
    && typeof failure.time_from_start_s === "number"
    && Number.isFinite(failure.time_from_start_s)
    ? `${failure.time_from_start_s.toFixed(3)} s`
    : "N/A";
}

function sampledPathStepLabel(stepRad) {
  return typeof stepRad === "number" && Number.isFinite(stepRad) && stepRad > 0
    ? `${stepRad.toFixed(3)} rad (${(stepRad * 180 / Math.PI).toFixed(3)}°)`
    : "N/A";
}

function updateTrajectoryValidationUi() {
  const first = trajectoryValidationState.firstViolation;
  const firstSampled = trajectoryValidationState.firstSampledPathFailure;
  const values = {
    digitalTwinValidationOverall: overallTrajectoryValidationLabel(),
    digitalTwinValidationJointLimits: trajectoryValidationState.jointLimits.replace(
      /_/g,
      " ",
    ),
    digitalTwinValidationMoveIt: trajectoryValidationState.moveitStateValidity.replace(
      /_/g,
      " ",
    ),
    digitalTwinValidationCollision: trajectoryValidationState.collision.replace(
      /_/g,
      " ",
    ),
    digitalTwinValidationCheckedPoints: String(
      trajectoryValidationState.checkedPointCount,
    ),
    digitalTwinValidationViolationCount: String(
      trajectoryValidationState.violationCount,
    ),
    digitalTwinValidationFirstPoint: validationFailedPointLabel(
      first,
      trajectoryValidationState.checkedPointCount,
    ),
    digitalTwinValidationFailedJoint: first ? first.jointName : "NONE",
    digitalTwinValidationJointValue: first
      ? `${first.valueRad.toFixed(3)} rad`
      : "N/A",
    digitalTwinValidationAllowedRange: first
      ? `[${first.minRad.toFixed(3)}, ${first.maxRad.toFixed(3)}] rad`
      : "N/A",
    digitalTwinValidationMoveItCheckedPoints: String(
      trajectoryValidationState.moveitCheckedPointCount,
    ),
    digitalTwinValidationMoveItFailedPoints: String(
      trajectoryValidationState.moveitFailedPointCount,
    ),
    digitalTwinValidationFirstMoveItPoint: moveitFailedPointLabel(
      trajectoryValidationState.firstMoveItFailedPoint,
      trajectoryValidationState.moveitCheckedPointCount,
    ),
    digitalTwinValidationFirstCollisionPair: collisionPairLabel(
      trajectoryValidationState.firstCollisionPair,
    ),
    digitalTwinValidationCollisionPairCount: String(
      trajectoryValidationState.collisionPairCount,
    ),
    digitalTwinValidationMaxPenetration: penetrationDepthLabel(
      trajectoryValidationState.maxPenetrationDepthM,
    ),
    digitalTwinValidationSampledStatus: trajectoryValidationState.sampledPathStatus.replace(
      /_/g,
      " ",
    ),
    digitalTwinValidationSampledMaxStep: sampledPathStepLabel(
      trajectoryValidationState.sampledPathMaxJointStepRad,
    ),
    digitalTwinValidationSampledSegmentCount: String(
      trajectoryValidationState.sampledPathSegmentCount,
    ),
    digitalTwinValidationSampledGeneratedCount: String(
      trajectoryValidationState.sampledPathGeneratedSampleCount,
    ),
    digitalTwinValidationSampledCheckedCount: String(
      trajectoryValidationState.sampledPathCheckedSampleCount,
    ),
    digitalTwinValidationSampledFailedCount: String(
      trajectoryValidationState.sampledPathFailedSampleCount,
    ),
    digitalTwinValidationSampledFirstSegment: sampledPathSegmentLabel(
      firstSampled,
      trajectoryValidationState.sampledPathSegmentCount,
    ),
    digitalTwinValidationSampledFirstSample: sampledPathSampleLabel(firstSampled),
    digitalTwinValidationSampledFailureAlpha: sampledPathAlphaLabel(firstSampled),
    digitalTwinValidationSampledFailureTime: sampledPathTimeLabel(firstSampled),
    digitalTwinValidationSampledFirstCollisionPair: collisionPairLabel(
      trajectoryValidationState.sampledPathFirstCollisionPair,
    ),
    digitalTwinValidationSampledCollisionPairCount: String(
      trajectoryValidationState.sampledPathCollisionPairCount,
    ),
    digitalTwinValidationSampledMaxPenetration: penetrationDepthLabel(
      trajectoryValidationState.sampledPathMaxPenetrationDepthM,
    ),
    digitalTwinValidationSource: validationSourceLabel(
      trajectoryValidationState.source,
    ),
    digitalTwinValidationModelSource: validationPathBasename(
      validationModelPath(trajectoryValidationState.source),
    ),
    digitalTwinValidationError: trajectoryValidationState.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
}

function resetMoveItValidationFields() {
  trajectoryValidationState.moveitStateValidity = "NOT_RUN";
  trajectoryValidationState.collision = "NOT_RUN";
  trajectoryValidationState.moveitCheckedPointCount = 0;
  trajectoryValidationState.moveitFailedPointCount = 0;
  trajectoryValidationState.firstMoveItFailedPoint = null;
  trajectoryValidationState.firstCollisionPair = null;
  trajectoryValidationState.collisionPairCount = 0;
  trajectoryValidationState.maxPenetrationDepthM = null;
  trajectoryValidationState.sampledPathStatus = "NOT_RUN";
  trajectoryValidationState.sampledPathMaxJointStepRad = (
    DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD
  );
  trajectoryValidationState.sampledPathSegmentCount = 0;
  trajectoryValidationState.sampledPathGeneratedSampleCount = 0;
  trajectoryValidationState.sampledPathCheckedSampleCount = 0;
  trajectoryValidationState.sampledPathFailedSampleCount = 0;
  trajectoryValidationState.firstSampledPathFailure = null;
  trajectoryValidationState.sampledPathCollisionPairCount = 0;
  trajectoryValidationState.sampledPathFirstCollisionPair = null;
  trajectoryValidationState.sampledPathMaxPenetrationDepthM = null;
}

function sampledPathMaxJointStep(maxJointStepRad = null) {
  let candidate = maxJointStepRad;
  if (candidate === null) {
    const input = document.getElementById("digitalTwinSampledPathMaxJointStep");
    const inputValue = input && typeof input.value === "string"
      ? input.value.trim()
      : "";
    candidate = input
      ? (inputValue === "" ? Number.NaN : Number(inputValue))
      : DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD;
  }
  if (
    typeof candidate !== "number"
    || !Number.isFinite(candidate)
    || candidate <= 0
  ) {
    throw new TypeError("Max Joint Step must be a finite number greater than 0 rad");
  }
  return candidate;
}

function clearTrajectoryValidation() {
  trajectoryValidationGeneration += 1;
  trajectoryValidationState.status = "NOT_VALIDATED";
  trajectoryValidationState.jointLimits = "NOT_VALIDATED";
  resetMoveItValidationFields();
  trajectoryValidationState.valid = null;
  trajectoryValidationState.checkedPointCount = 0;
  trajectoryValidationState.checkedJointCount = 0;
  trajectoryValidationState.violationCount = 0;
  trajectoryValidationState.violations = [];
  trajectoryValidationState.firstViolation = null;
  trajectoryValidationState.source = "NONE";
  trajectoryValidationState.error = null;
  updateTrajectoryValidationUi();
  return trajectoryValidationStateSnapshot();
}

function validateLoadedTrajectoryJointLimits() {
  if (moveitValidationInFlight) trajectoryValidationGeneration += 1;
  if (!trajectoryPreviewState.trajectory) {
    clearTrajectoryValidation();
    trajectoryValidationState.status = "ERROR";
    trajectoryValidationState.error = "No normalized planned trajectory is loaded";
    updateTrajectoryValidationUi();
    return trajectoryValidationStateSnapshot();
  }

  resetMoveItValidationFields();
  trajectoryValidationState.status = "VALIDATING";
  trajectoryValidationState.error = null;
  updateTrajectoryValidationUi();
  try {
    const result = validateTrajectoryJointLimits(
      trajectoryPreviewState.trajectory,
      DUAL_JAKA_A12_JOINT_LIMIT_METADATA,
    );
    trajectoryValidationState.status = result.valid ? "VALID" : "INVALID";
    trajectoryValidationState.jointLimits = result.valid ? "PASS" : "FAIL";
    trajectoryValidationState.moveitStateValidity = "NOT_RUN";
    trajectoryValidationState.collision = "NOT_RUN";
    trajectoryValidationState.valid = result.valid;
    trajectoryValidationState.checkedPointCount = result.checkedPointCount;
    trajectoryValidationState.checkedJointCount = result.checkedJointCount;
    trajectoryValidationState.violationCount = result.violations.length;
    trajectoryValidationState.violations = result.violations.map(
      (violation) => ({ ...violation }),
    );
    trajectoryValidationState.firstViolation = result.firstViolation
      ? { ...result.firstViolation }
      : null;
    trajectoryValidationState.source = (
      `GENERATED URDF POSITION LIMITS — `
      + DUAL_JAKA_A12_JOINT_LIMIT_METADATA.source_model
    );
    trajectoryValidationState.error = null;
  } catch (error) {
    clearTrajectoryValidation();
    trajectoryValidationState.status = "ERROR";
    trajectoryValidationState.error = error && error.message
      ? error.message
      : "Trajectory joint-limit validation failed";
  }
  updateTrajectoryValidationUi();
  return trajectoryValidationStateSnapshot();
}

function getTrajectoryValidationState() {
  return trajectoryValidationStateSnapshot();
}

function applySampledPathValidation(sampledPath) {
  if (!sampledPath || typeof sampledPath !== "object") {
    trajectoryValidationState.sampledPathStatus = "ERROR";
    trajectoryValidationState.error = "Sampled-path validation response is missing";
    return;
  }
  const sampledStatus = sampledPath.status;
  trajectoryValidationState.sampledPathStatus = (
    ["PASS", "FAIL", "SKIPPED", "UNAVAILABLE", "TIMEOUT", "ERROR"].includes(
      sampledStatus,
    )
  ) ? sampledStatus : "ERROR";
  trajectoryValidationState.sampledPathMaxJointStepRad = (
    typeof sampledPath.max_joint_step_rad === "number"
    && Number.isFinite(sampledPath.max_joint_step_rad)
  ) ? sampledPath.max_joint_step_rad : DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD;
  trajectoryValidationState.sampledPathSegmentCount = Number.isInteger(
    sampledPath.segment_count,
  ) ? sampledPath.segment_count : 0;
  trajectoryValidationState.sampledPathGeneratedSampleCount = Number.isInteger(
    sampledPath.generated_interior_sample_count,
  ) ? sampledPath.generated_interior_sample_count : 0;
  trajectoryValidationState.sampledPathCheckedSampleCount = Number.isInteger(
    sampledPath.checked_interior_sample_count,
  ) ? sampledPath.checked_interior_sample_count : 0;
  trajectoryValidationState.sampledPathFailedSampleCount = Number.isInteger(
    sampledPath.failed_interior_sample_count,
  ) ? sampledPath.failed_interior_sample_count : 0;
  trajectoryValidationState.firstSampledPathFailure = sampledPath.first_failed_sample
    ? {
      ...sampledPath.first_failed_sample,
      contacts: Array.isArray(sampledPath.first_failed_sample.contacts)
        ? sampledPath.first_failed_sample.contacts.map((contact) => ({ ...contact }))
        : [],
    }
    : null;
  trajectoryValidationState.sampledPathCollisionPairCount = Number.isInteger(
    sampledPath.collision_pair_count,
  ) ? sampledPath.collision_pair_count : 0;
  trajectoryValidationState.sampledPathFirstCollisionPair = (
    sampledPath.first_collision_pair
  ) ? { ...sampledPath.first_collision_pair } : null;
  trajectoryValidationState.sampledPathMaxPenetrationDepthM = (
    typeof sampledPath.max_penetration_depth_m === "number"
    && Number.isFinite(sampledPath.max_penetration_depth_m)
  ) ? sampledPath.max_penetration_depth_m : null;
  if (sampledPath.error) trajectoryValidationState.error = sampledPath.error;
}

async function validateLoadedTrajectory(maxJointStepRad = null) {
  if (moveitValidationInFlight) return trajectoryValidationStateSnapshot();
  const jointLimitResult = validateLoadedTrajectoryJointLimits();
  if (jointLimitResult.jointLimits !== "PASS" || !jointLimitResult.valid) {
    return jointLimitResult;
  }

  let sampledStep;
  try {
    sampledStep = sampledPathMaxJointStep(maxJointStepRad);
  } catch (error) {
    trajectoryValidationState.status = "ERROR";
    trajectoryValidationState.sampledPathStatus = "ERROR";
    trajectoryValidationState.valid = null;
    trajectoryValidationState.error = error.message;
    updateTrajectoryValidationUi();
    return trajectoryValidationStateSnapshot();
  }

  moveitValidationInFlight = true;
  const validationGeneration = trajectoryValidationGeneration;
  trajectoryValidationState.status = "VALIDATING";
  trajectoryValidationState.moveitStateValidity = "CHECKING";
  trajectoryValidationState.collision = "NOT_RUN";
  trajectoryValidationState.sampledPathStatus = "CHECKING";
  trajectoryValidationState.sampledPathMaxJointStepRad = sampledStep;
  trajectoryValidationState.error = null;
  updateTrajectoryValidationUi();
  try {
    const payload = await requestMoveItTrajectoryValidation(
      trajectoryPreviewState.trajectory,
      {
        sampledPath: {
          enabled: true,
          max_joint_step_rad: sampledStep,
        },
      },
    );
    if (validationGeneration !== trajectoryValidationGeneration) {
      return trajectoryValidationStateSnapshot();
    }
    const validation = payload.validation;
    const status = validation.status;
    trajectoryValidationState.moveitCheckedPointCount = Number.isInteger(
      validation.checked_point_count,
    ) ? validation.checked_point_count : 0;
    trajectoryValidationState.moveitFailedPointCount = Number.isInteger(
      validation.failed_point_count,
    ) ? validation.failed_point_count : 0;
    trajectoryValidationState.firstMoveItFailedPoint = validation.first_failed_point
      ? {
        ...validation.first_failed_point,
        contacts: Array.isArray(validation.first_failed_point.contacts)
          ? validation.first_failed_point.contacts.map((contact) => ({ ...contact }))
          : [],
      }
      : null;
    trajectoryValidationState.firstCollisionPair = validation.first_collision_pair
      ? { ...validation.first_collision_pair }
      : null;
    trajectoryValidationState.collisionPairCount = Number.isInteger(
      validation.collision_pair_count,
    ) ? validation.collision_pair_count : 0;
    trajectoryValidationState.maxPenetrationDepthM = (
      typeof validation.max_penetration_depth_m === "number"
      && Number.isFinite(validation.max_penetration_depth_m)
    ) ? validation.max_penetration_depth_m : null;
    trajectoryValidationState.source = (
      `JOINT LIMITS + MOVEIT STORED + SAMPLED PATH — `
      + DUAL_JAKA_A12_JOINT_LIMIT_METADATA.source_model
    );
    trajectoryValidationState.error = validation.error || null;

    if (status === "PASS") {
      trajectoryValidationState.moveitStateValidity = "PASS";
      trajectoryValidationState.collision = "PASS";
      applySampledPathValidation(validation.sampled_path);
      if (trajectoryValidationState.sampledPathStatus === "PASS") {
        trajectoryValidationState.status = "VALID";
        trajectoryValidationState.valid = true;
      } else if (trajectoryValidationState.sampledPathStatus === "FAIL") {
        trajectoryValidationState.status = "INVALID";
        trajectoryValidationState.valid = false;
      } else {
        trajectoryValidationState.status = "ERROR";
        trajectoryValidationState.valid = null;
      }
    } else if (status === "FAIL") {
      trajectoryValidationState.status = "INVALID";
      trajectoryValidationState.moveitStateValidity = "FAIL";
      trajectoryValidationState.collision = (
        trajectoryValidationState.collisionPairCount > 0
      ) ? "FAIL" : "NOT_DETECTED";
      applySampledPathValidation(validation.sampled_path || { status: "SKIPPED" });
      trajectoryValidationState.valid = false;
    } else {
      trajectoryValidationState.status = "ERROR";
      trajectoryValidationState.moveitStateValidity = (
        ["UNAVAILABLE", "TIMEOUT", "ERROR"].includes(status)
      ) ? status : "ERROR";
      trajectoryValidationState.collision = "NOT_RUN";
      applySampledPathValidation(validation.sampled_path || { status: "SKIPPED" });
      trajectoryValidationState.valid = null;
    }
  } catch (error) {
    if (validationGeneration !== trajectoryValidationGeneration) {
      return trajectoryValidationStateSnapshot();
    }
    trajectoryValidationState.status = "ERROR";
    trajectoryValidationState.moveitStateValidity = error && error.code === "TIMEOUT"
      ? "TIMEOUT"
      : "ERROR";
    trajectoryValidationState.collision = "NOT_RUN";
    trajectoryValidationState.sampledPathStatus = "SKIPPED";
    trajectoryValidationState.valid = null;
    trajectoryValidationState.error = error && error.message
      ? error.message
      : "MoveIt validation request failed";
  } finally {
    moveitValidationInFlight = false;
    if (validationGeneration === trajectoryValidationGeneration) {
      updateTrajectoryValidationUi();
    }
  }
  return trajectoryValidationStateSnapshot();
}

function formatMirrorTimestamp(timestampMs) {
  if (typeof timestampMs !== "number" || !Number.isFinite(timestampMs)) {
    return "NEVER";
  }
  const date = new Date(timestampMs);
  return Number.isFinite(date.getTime()) ? date.toISOString() : "INVALID TIMESTAMP";
}

function updateMirrorUi() {
  const values = {
    digitalTwinMirrorMode: mirrorState.mode,
    digitalTwinLastSnapshot: formatMirrorTimestamp(
      mirrorState.lastAcceptedSnapshotMs,
    ),
    digitalTwinLastApplied: formatMirrorTimestamp(
      mirrorState.lastAppliedSnapshotMs,
    ),
    digitalTwinMirrorEnabled: String(mirrorState.enabled),
    digitalTwinUpdateSource: mirrorState.updateSource,
    digitalTwinMainModelSource: mainModelState.source,
    digitalTwinMirrorError: mirrorState.validationError || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  updatePhase4TrajectoryValidationUi();
}

function mirrorStateSnapshot() {
  return {
    mode: mirrorState.mode,
    enabled: mirrorState.enabled,
    staleTimeoutMs: mirrorState.staleTimeoutMs,
    hasValidSnapshot: mirrorState.latestValidSnapshot !== null,
    lastAcceptedSnapshotMs: mirrorState.lastAcceptedSnapshotMs,
    lastAppliedSnapshotMs: mirrorState.lastAppliedSnapshotMs,
    validationError: mirrorState.validationError,
    updateSource: mirrorState.updateSource,
    mainModelSource: mainModelState.source,
    mainModelConfigurationSource: mainModelState.configurationSource,
  };
}

function updateMirrorStaleness(
  nowMs = Date.now(),
  staleTimeoutMs = mirrorState.staleTimeoutMs,
) {
  if (
    !mirrorState.enabled ||
    !mirrorState.latestValidSnapshot ||
    mirrorState.mode === MIRROR_MODES.INVALID
  ) {
    return false;
  }

  const stale = isNormalizedSnapshotStale(
    mirrorState.latestValidSnapshot,
    nowMs,
    staleTimeoutMs,
  );
  if (stale) {
    mirrorState.mode = MIRROR_MODES.STALE;
    resetMirrorVisualSmoothing();
    setModelTcpFeedbackStatus({ left: "STALE", right: "STALE" });
    applyPlanningStartStateToMainWhenOffline(nowMs);
    updateMirrorUi();
  }
  return stale;
}

function resetMirrorVisualSmoothing() {
  mirrorVisualState.currentPose = null;
  mirrorVisualState.lastFrameTimeMs = null;
  mirrorVisualState.lastUiSyncFrameMs = null;
}

function applyMirrorPoseToMainModel(pose, updateUi = false) {
  if (!robot || loadState.status !== "READY") return false;
  // Rendered MAIN pose is visualization-only. Authoritative actual state is
  // owned by the latest accepted raw feedback snapshot and must not be
  // overwritten by RAF smoothing/interpolation.
  applyJointValuesToModel(robot, pose);
  if (updateUi) updatePlannedPreviewUi();
  return true;
}

function seedMirrorVisualPoseFromLatest(nowMs = Date.now()) {
  if (!mirrorState.latestValidSnapshot) return false;
  const pose = {
    left: [...mirrorState.latestValidSnapshot.left],
    right: [...mirrorState.latestValidSnapshot.right],
  };
  mirrorVisualState.currentPose = copyDualArmPose(pose);
  mirrorVisualState.lastFrameTimeMs = null;
  mirrorVisualState.lastUiSyncFrameMs = null;
  if (!applyMirrorPoseToMainModel(pose, true)) return false;
  mirrorState.lastAppliedSnapshotMs = nowMs;
  return true;
}

function applySmoothedMirrorVisualFrame(frameTimeMs, nowMs = Date.now()) {
  if (
    !mirrorState.enabled
    || !mirrorState.latestValidSnapshot
    || mirrorState.mode === MIRROR_MODES.INVALID
    || mirrorState.mode === MIRROR_MODES.STALE
    || loadState.status !== "READY"
    || !robot
    || isNormalizedSnapshotStale(
      mirrorState.latestValidSnapshot, nowMs, mirrorState.staleTimeoutMs,
    )
  ) return false;

  const targetPose = {
    left: mirrorState.latestValidSnapshot.left,
    right: mirrorState.latestValidSnapshot.right,
  };
  if (!mirrorVisualState.currentPose) {
    seedMirrorVisualPoseFromLatest(nowMs);
  } else {
    const previousFrame = mirrorVisualState.lastFrameTimeMs;
    const deltaTimeMs = Number.isFinite(previousFrame)
      ? Math.max(0, frameTimeMs - previousFrame)
      : 1000 / 60;
    const smoothed = smoothDualArmPose(
      mirrorVisualState.currentPose, targetPose, deltaTimeMs,
      mirrorVisualState.smoothingTauMs,
    );
    mirrorVisualState.currentPose = {
      left: smoothed.left,
      right: smoothed.right,
    };
    applyMirrorPoseToMainModel(mirrorVisualState.currentPose, false);
    mirrorState.lastAppliedSnapshotMs = nowMs;
  }
  mirrorVisualState.lastFrameTimeMs = frameTimeMs;
  mirrorState.mode = MIRROR_MODES.LIVE_MIRROR;
  mainModelState.source = "LIVE JOINT FEEDBACK";
  mainModelState.configurationSource = "NOT ACTIVE";

  if (
    !Number.isFinite(mirrorVisualState.lastUiSyncFrameMs)
    || frameTimeMs - mirrorVisualState.lastUiSyncFrameMs >= LIVE_MIRROR_VISUAL_UI_SYNC_MS
  ) {
    mirrorVisualState.lastUiSyncFrameMs = frameTimeMs;
    updatePlannedPreviewUi();
    captureModelTcpWorldPoses();
    updateMirrorUi();
  }
  return true;
}

function applyLatestMirrorSnapshotIfPossible(nowMs = Date.now()) {
  if (!mirrorState.enabled) return false;
  if (!mirrorState.latestValidSnapshot) {
    mirrorState.mode = MIRROR_MODES.MIRROR_READY;
    applyPlanningStartStateToMainWhenOffline(nowMs);
    updateMirrorUi();
    return false;
  }
  if (updateMirrorStaleness(nowMs)) return false;
  if (loadState.status !== "READY" || !robot) {
    mirrorState.mode = MIRROR_MODES.MIRROR_READY;
    updateMirrorUi();
    return false;
  }

  const firstVisualApply = mirrorVisualState.currentPose === null;
  if (firstVisualApply) {
    seedMirrorVisualPoseFromLatest(nowMs);
    captureModelTcpWorldPoses();
    initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady();
    updateRigidGraspConfigurationUi();
  }
  mirrorState.mode = MIRROR_MODES.LIVE_MIRROR;
  mainModelState.source = "LIVE JOINT FEEDBACK";
  mainModelState.configurationSource = "NOT ACTIVE";
  mirrorState.validationError = null;
  if (firstVisualApply) updateMirrorUi();
  return true;
}

function ingestStatusSnapshot(
  snapshot,
  receivedAtMs = Date.now(),
  sourceLabel = "LOCAL STATUS SNAPSHOT",
) {
  let normalized;
  try {
    normalized = normalizeDualArmStatusSnapshot(snapshot, receivedAtMs);
  } catch (error) {
    mirrorState.mode = MIRROR_MODES.INVALID;
    resetMirrorVisualSmoothing();
    setModelTcpFeedbackStatus({ left: "INVALID", right: "INVALID" });
    mirrorState.validationError = error && error.message
      ? error.message
      : "Invalid status snapshot";
    applyPlanningStartStateToMainWhenOffline(receivedAtMs);
    updateMirrorUi();
    return mirrorStateSnapshot();
  }

  mirrorState.latestValidSnapshot = normalized;
  mirrorState.lastAcceptedSnapshotMs = normalized.receivedAtMs;
  // Keep raw accepted feedback authoritative for all "actual" semantics.
  // White MAIN rendering may be smoothed separately at display frame rate.
  latestActualPose = {
    left: [...normalized.left],
    right: [...normalized.right],
  };
  updatePlannedDelta();
  mirrorState.validationError = null;
  mirrorState.updateSource = (
    typeof sourceLabel === "string" && sourceLabel.trim().length > 0
  )
    ? sourceLabel.trim()
    : "LOCAL STATUS SNAPSHOT";

  if (!mirrorState.enabled) {
    mirrorState.mode = MIRROR_MODES.STATIC;
    applyPlanningStartStateToMainWhenOffline(receivedAtMs);
    updateMirrorUi();
    return mirrorStateSnapshot();
  }

  applyLatestMirrorSnapshotIfPossible(Date.now());
  return mirrorStateSnapshot();
}

function setMirrorEnabled(enabled) {
  if (typeof enabled !== "boolean") {
    mirrorState.mode = MIRROR_MODES.INVALID;
    mirrorState.validationError = "Mirror enabled state must be boolean";
    updateMirrorUi();
    return mirrorStateSnapshot();
  }

  mirrorState.enabled = enabled;
  if (!enabled) {
    mirrorState.mode = MIRROR_MODES.STATIC;
    resetMirrorVisualSmoothing();
    setModelTcpFeedbackStatus({ left: "MISSING", right: "MISSING" });
    applyPlanningStartStateToMainWhenOffline(Date.now());
    updateMirrorUi();
    return mirrorStateSnapshot();
  }

  mirrorState.validationError = null;
  if (!mirrorState.latestValidSnapshot) {
    mirrorState.mode = MIRROR_MODES.MIRROR_READY;
    applyPlanningStartStateToMainWhenOffline(Date.now());
    updateMirrorUi();
  } else {
    applyLatestMirrorSnapshotIfPossible(Date.now());
  }
  return mirrorStateSnapshot();
}

function getMirrorState(
  nowMs = Date.now(),
  staleTimeoutMs = mirrorState.staleTimeoutMs,
) {
  updateMirrorStaleness(nowMs, staleTimeoutMs);
  return mirrorStateSnapshot();
}

function resetToStaticPose() {
  mirrorState.enabled = false;
  mirrorState.mode = MIRROR_MODES.STATIC;
  resetMirrorVisualSmoothing();
  if (!applyPlanningStartStateToMainWhenOffline(Date.now())
      && loadState.status === "READY" && robot) {
    setJointValues({
      left: [...ZERO_JOINT_VALUES.left],
      right: [...ZERO_JOINT_VALUES.right],
    });
    mainModelState.source = "STATIC ZERO FALLBACK";
    mainModelState.configurationSource = "UNAVAILABLE";
  }
  mirrorState.lastAppliedSnapshotMs = null;
  mirrorState.validationError = null;
  mirrorState.updateSource = "STATIC RESET";
  modelTcpState.lastValidWorldPose.left = null;
  modelTcpState.lastValidWorldPose.right = null;
  setModelTcpFeedbackStatus({ left: "MISSING", right: "MISSING" });
  updateMirrorUi();
  return mirrorStateSnapshot();
}

function copyObjectPose(pose) {
  return {
    translationM: [...pose.translationM],
    rpyRad: [...pose.rpyRad],
  };
}

function getObjectPreviewState() {
  return {
    pose: objectPreviewState.pose ? copyObjectPose(objectPreviewState.pose) : null,
    objectVisible: objectPreviewState.objectVisible,
    objectFrameVisible: objectPreviewState.objectFrameVisible,
    graspFramesVisible: objectPreviewState.graspFramesVisible,
    error: objectPreviewState.error,
  };
}

function copyGraspPose(pose) {
  return pose ? {
    translationM: [...pose.translationM],
    rpyRad: [...pose.rpyRad],
  } : null;
}

function copyGraspContent(content) {
  return content ? {
    left: copyGraspPose(content.left),
    right: copyGraspPose(content.right),
  } : null;
}

function currentRigidGraspContent() {
  if (rigidGraspState.state === "GRASP_LOCKED" && rigidGraspState.lockedSnapshot) {
    return rigidGraspState.lockedSnapshot;
  }
  return rigidGraspState.draft;
}

function rigidGraspStateSnapshot() {
  return {
    status: rigidGraspState.status,
    state: rigidGraspState.state,
    draft: copyGraspContent(rigidGraspState.draft),
    draftSource: rigidGraspState.draftSource,
    draftContentRevision: rigidGraspState.draftContentRevision,
    lockedSnapshot: rigidGraspState.lockedSnapshot ? {
      ...copyGraspContent(rigidGraspState.lockedSnapshot),
      contentRevision: rigidGraspState.lockedSnapshot.contentRevision,
      lockRevision: rigidGraspState.lockedSnapshot.lockRevision,
      lockGeneration: rigidGraspState.lockedSnapshot.lockGeneration,
    } : null,
    authoritativeRevision: rigidGraspState.authoritativeRevision,
    lockGeneration: rigidGraspState.lockGeneration,
    rotationConvention: rigidGraspState.rotationConvention,
    units: { ...rigidGraspState.units },
    semantic: rigidGraspState.semantic,
    physicallyCalibrated: rigidGraspState.physicallyCalibrated,
    plannerIntegration: rigidGraspState.plannerIntegration,
    worldCalibrationRevision: rigidGraspState.worldCalibrationRevision,
    selectedFrame: rigidGraspState.selectedFrame,
    gizmoMode: rigidGraspState.gizmoMode,
    requestInFlight: rigidGraspState.requestInFlight,
    alignmentStatus: rigidGraspState.alignmentStatus,
    alignmentResidual: rigidGraspState.alignmentResidual
      ? { ...rigidGraspState.alignmentResidual }
      : null,
    error: rigidGraspState.error,
  };
}

function getRigidGraspConfigurationState() {
  return rigidGraspStateSnapshot();
}

function phase3PlanningAuthorityIdentity() {
  const locked = rigidGraspState.lockedSnapshot;
  if (
    rigidGraspState.status !== "READY"
    || rigidGraspState.state !== "GRASP_LOCKED"
    || rigidGraspState.plannerIntegration !== "PRE_P5_C1_C2_INTEGRATED"
    || !locked
    || !locked.contentRevision
    || !locked.lockRevision
    || !Number.isInteger(locked.lockGeneration)
    || locked.lockGeneration < 1
    || !worldCalibrationState.matches
    || !worldCalibrationState.backendRevision
    || !worldCalibrationState.modelRevision
    || worldCalibrationState.backendRevision !== worldCalibrationState.modelRevision
    || rigidGraspState.worldCalibrationRevision
      !== worldCalibrationState.backendRevision
  ) return null;
  return Object.freeze({
    graspContentRevision: locked.contentRevision,
    lockGeneration: locked.lockGeneration,
    lockRevision: locked.lockRevision,
    calibrationRevision: worldCalibrationState.backendRevision,
    modelCalibrationRevision: worldCalibrationState.modelRevision,
  });
}

function samePhase3PlanningAuthority(left, right) {
  return Boolean(left && right
    && left.graspContentRevision === right.graspContentRevision
    && left.lockGeneration === right.lockGeneration
    && left.lockRevision === right.lockRevision
    && left.calibrationRevision === right.calibrationRevision
    && left.modelCalibrationRevision === right.modelCalibrationRevision);
}

function getObjectGraspRelativeState() {
  const content = currentRigidGraspContent();
  return content ? computeObjectGraspRelativeState(content) : null;
}

function configureOperatorRotationInput(input) {
  if (!input) return;
  const metadata = angleUnitMetadata(angleUnitState.unit);
  input.min = String(metadata.min);
  input.max = String(metadata.max);
  input.step = String(metadata.step);
}

function writeOperatorRotationInput(input, radians) {
  if (!input) return;
  configureOperatorRotationInput(input);
  input.value = String(rotationRadiansToDisplay(radians, angleUnitState.unit));
}

function readOperatorRotationInput(input, label) {
  if (!input || !Number.isFinite(input.valueAsNumber)) {
    throw new TypeError(`${label} must be a finite rotation value`);
  }
  return rotationDisplayToRadians(input.valueAsNumber, angleUnitState.unit);
}

function updateOperatorRotationUnitLabels() {
  const metadata = angleUnitMetadata(angleUnitState.unit);
  document.querySelectorAll("[data-digital-twin-rotation-label]").forEach((element) => {
    const name = element.dataset.digitalTwinRotationLabel || "Rotation";
    element.textContent = `${name} [${metadata.shortLabel}]`;
  });
  const selector = document.getElementById("digitalTwinRotationUnit");
  if (selector) selector.value = angleUnitState.unit;
}

function writeNewObjectWaypointRotationControls() {
  ["Roll", "Pitch", "Yaw"].forEach((axis, index) => {
    const input = document.getElementById(`digitalTwinObjectWaypointNew${axis}`);
    if (input) writeOperatorRotationInput(input, newObjectWaypointDraftRpyRad[index]);
  });
}

function getAngleUnitPresentationState() {
  return {
    unit: angleUnitState.unit,
    metadata: {...angleUnitMetadata(angleUnitState.unit)},
  };
}

function setGlobalAngleUnit(unit, {persist = true} = {}) {
  angleUnitState.unit = normalizeAngleUnit(unit);
  if (persist) {
    try {
      window.localStorage.setItem(ANGLE_UNIT_STORAGE_KEY, angleUnitState.unit);
    } catch (_error) {
      // Presentation preference persistence is optional and must fail safely.
    }
  }
  updateOperatorRotationUnitLabels();
  writeRigidGraspEditorFields();
  if (objectPreviewState.pose) writeObjectPoseControls(objectPreviewState.pose);
  writeNewObjectWaypointRotationControls();
  updateObjectGraspRelativeUi();
  updateApproachWaypointUi();
  renderObjectWaypointRows();
  const fallbackOrientation = document.getElementById("digitalTwinObjectWaypointOrientation");
  if (fallbackOrientation) {
    fallbackOrientation.textContent = objectWaypointPlanningState.fixedOrientationRpyRad
      .map((value) => formatRotationRadians(value, angleUnitState.unit)).join(", ");
  }
  // Presentation-only by design: no planning/Phase-4/Frozen authority invalidation.
  return getAngleUnitPresentationState();
}

function bindGlobalAngleUnitControl() {
  const selector = document.getElementById("digitalTwinRotationUnit");
  if (selector) selector.addEventListener("change", (event) => {
    setGlobalAngleUnit(event.target.value);
  });
  ["Roll", "Pitch", "Yaw"].forEach((axis, index) => {
    const input = document.getElementById(`digitalTwinObjectWaypointNew${axis}`);
    if (!input) return;
    input.addEventListener("input", () => {
      if (Number.isFinite(input.valueAsNumber)) {
        newObjectWaypointDraftRpyRad[index] = rotationDisplayToRadians(
          input.valueAsNumber,
          angleUnitState.unit,
        );
      }
    });
  });
  setGlobalAngleUnit(angleUnitState.unit, {persist: false});
}

function updateObjectGraspRelativeUi() {
  const state = getObjectGraspRelativeState();
  if (!state) {
    for (const prefix of [
      "digitalTwinObjectTLeft",
      "digitalTwinObjectTRight",
      "digitalTwinLeftTRight",
    ]) {
      for (const suffix of ["X", "Y", "Z", "Rx", "Ry", "Rz"]) {
        const element = document.getElementById(`${prefix}${suffix}`);
        if (element) element.textContent = "UNAVAILABLE";
      }
    }
    return;
  }
  const readouts = [
    ["digitalTwinObjectTLeft", state.objectTLeft],
    ["digitalTwinObjectTRight", state.objectTRight],
    ["digitalTwinLeftTRight", state.leftTRight],
  ];
  const format = (value) => (Math.abs(value) < 5e-10 ? 0 : value).toFixed(6);
  readouts.forEach(([prefix, pose]) => {
    const values = [
      ["X", pose.translationM[0]],
      ["Y", pose.translationM[1]],
      ["Z", pose.translationM[2]],
      ["Rx", pose.rpyRad[0]],
      ["Ry", pose.rpyRad[1]],
      ["Rz", pose.rpyRad[2]],
    ];
    values.forEach(([suffix, value]) => {
      const element = document.getElementById(`${prefix}${suffix}`);
      if (element) {
        element.textContent = ["Rx", "Ry", "Rz"].includes(suffix)
          ? formatRotationRadians(value, angleUnitState.unit) : format(value);
      }
    });
  });
}

function writeRigidGraspEditorFields() {
  if (!rigidGraspState.draft) return;
  for (const side of ["left", "right"]) {
    const title = side === "left" ? "Left" : "Right";
    const pose = rigidGraspState.draft[side];
    const values = {
      X: pose.translationM[0],
      Y: pose.translationM[1],
      Z: pose.translationM[2],
      RollDeg: rotationRadiansToDisplay(pose.rpyRad[0], angleUnitState.unit),
      PitchDeg: rotationRadiansToDisplay(pose.rpyRad[1], angleUnitState.unit),
      YawDeg: rotationRadiansToDisplay(pose.rpyRad[2], angleUnitState.unit),
    };
    Object.entries(values).forEach(([suffix, value]) => {
      const element = document.getElementById(`digitalTwin${title}Grasp${suffix}`);
      if (["RollDeg", "PitchDeg", "YawDeg"].includes(suffix)) {
        configureOperatorRotationInput(element);
      }
      if (element && document.activeElement !== element) {
        element.value = String(value);
      }
    });
  }
}

function updateRigidGraspConfigurationUi() {
  const values = {
    digitalTwinGraspConfigurationState: rigidGraspState.state,
    digitalTwinGraspDraftSource: rigidGraspState.draftSource,
    digitalTwinGraspRevision: rigidGraspState.authoritativeRevision
      || rigidGraspState.draftContentRevision || "UNAVAILABLE",
    digitalTwinGraspLockGeneration: String(rigidGraspState.lockGeneration),
    digitalTwinGraspSelectedFrame: rigidGraspState.selectedFrame,
    digitalTwinGraspGizmoMode: rigidGraspState.gizmoMode.toUpperCase(),
    digitalTwinGraspPlannerIntegration: rigidGraspState.plannerIntegration,
    digitalTwinGraspWorldCalibrationRevision: (
      rigidGraspState.worldCalibrationRevision || "UNAVAILABLE"
    ),
    digitalTwinGraspAlignmentStatus: rigidGraspState.alignmentStatus,
    digitalTwinGraspConfigurationError: rigidGraspState.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const editable = rigidGraspState.status === "READY"
    && rigidGraspState.state === "GRASP_UNLOCKED"
    && !rigidGraspState.requestInFlight;
  const locked = rigidGraspState.status === "READY"
    && rigidGraspState.state === "GRASP_LOCKED";
  const lockedSummary = document.getElementById("digitalTwinGraspLockedSummary");
  const unlockedEditor = document.getElementById("digitalTwinGraspUnlockedEditor");
  if (lockedSummary) lockedSummary.hidden = !locked;
  if (unlockedEditor) unlockedEditor.hidden = locked;
  const operatorStatus = document.getElementById("digitalTwinOperatorGraspStatus");
  if (operatorStatus) {
    operatorStatus.textContent = locked ? "✓ Grasp Locked" : "Grasp Setup Required";
  }
  const instruction = document.getElementById("digitalTwinOperatorGraspInstruction");
  if (instruction) {
    instruction.textContent = locked
      ? "This grasp configuration is preserved from the current planning state."
      : "Set both grasp poses, then lock the grasp.";
  }
  for (const side of ["Left", "Right"]) {
    for (const suffix of ["X", "Y", "Z", "RollDeg", "PitchDeg", "YawDeg"]) {
      const element = document.getElementById(`digitalTwin${side}Grasp${suffix}`);
      if (element) element.disabled = !editable;
    }
    const applyButton = document.getElementById(`digitalTwinApply${side}Grasp`);
    if (applyButton) applyButton.disabled = !editable;
  }
  const lockButton = document.getElementById("digitalTwinLockGraspConfiguration");
  if (lockButton) lockButton.disabled = !editable;
  const resetFramesButton = document.getElementById("digitalTwinResetObjectPreview");
  if (resetFramesButton) {
    resetFramesButton.disabled = !editable;
    resetFramesButton.hidden = locked;
    resetFramesButton.title = locked
      ? "Choose Edit Current Grasp or Start New Grasp from Current Robot Pose first."
      : "Snapshot the displayed MAIN left_J6/right_J6 model-tip transforms.";
  }
  const unlockButton = document.getElementById("digitalTwinUnlockGraspConfiguration");
  if (unlockButton) unlockButton.disabled = (
    rigidGraspState.state !== "GRASP_LOCKED" || rigidGraspState.requestInFlight
  );
  const startNewButton = document.getElementById(
    "digitalTwinStartNewGraspFromCurrentRobotPose",
  );
  if (startNewButton) startNewButton.disabled = (
    !locked
    || rigidGraspState.requestInFlight
    || loadState.status !== "READY"
    || !robot
    || !["LIVE JOINT FEEDBACK", "OFFLINE CODE CONFIG"].includes(mainModelState.source)
  );
  const translateButton = document.getElementById("digitalTwinGraspGizmoTranslate");
  const rotateButton = document.getElementById("digitalTwinGraspGizmoRotate");
  const selectedGrasp = rigidGraspState.selectedFrame !== GRASP_SELECTED_FRAMES.CENTER;
  for (const id of ["digitalTwinObjectRoll", "digitalTwinObjectPitch", "digitalTwinObjectYaw"]) {
    const field = document.getElementById(id);
    if (field) field.disabled = !locked;
  }
  const centerRotationReady = rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER
    && rigidGraspState.state === "GRASP_LOCKED";
  if (translateButton) translateButton.disabled = selectedGrasp && !editable;
  if (rotateButton) rotateButton.disabled = selectedGrasp ? !editable : !centerRotationReady;
  if (translateButton) translateButton.setAttribute(
    "aria-pressed", String(rigidGraspState.gizmoMode === "translate"),
  );
  if (rotateButton) rotateButton.setAttribute(
    "aria-pressed", String(rigidGraspState.gizmoMode === "rotate"),
  );
  const frameButtons = {
    digitalTwinSelectObjectFrame: GRASP_SELECTED_FRAMES.CENTER,
    digitalTwinSelectLeftGraspFrame: GRASP_SELECTED_FRAMES.LEFT_GRASP,
    digitalTwinSelectRightGraspFrame: GRASP_SELECTED_FRAMES.RIGHT_GRASP,
  };
  Object.entries(frameButtons).forEach(([id, frame]) => {
    const button = document.getElementById(id);
    if (!button) return;
    button.setAttribute("aria-pressed", String(rigidGraspState.selectedFrame === frame));
    button.disabled = frame !== GRASP_SELECTED_FRAMES.CENTER && !editable;
  });
  writeRigidGraspEditorFields();
  updateObjectGraspRelativeUi();
  updateObjectWaypointPlanningUi();
}

function applyBackendRigidGraspState(nextState) {
  const selectedFrame = rigidGraspState.selectedFrame;
  const gizmoMode = rigidGraspState.gizmoMode;
  Object.assign(rigidGraspState, nextState, {
    status: "READY",
    selectedFrame,
    gizmoMode,
    requestInFlight: false,
    error: null,
  });
  if (rigidGraspState.state === "GRASP_LOCKED"
      && rigidGraspState.selectedFrame !== GRASP_SELECTED_FRAMES.CENTER) {
    rigidGraspState.selectedFrame = GRASP_SELECTED_FRAMES.CENTER;
  }
  if (!initialGraspBackendStateObserved) {
    initialGraspBackendStateObserved = true;
    initialGraspFrameSnapshotPending = true;
    initialGraspFrameSnapshotComplete = false;
  }
  const initializedFromModel =
    initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady();
  if (!initializedFromModel && objectPreviewState.pose) {
    applyObjectPreviewPose(objectPreviewState.pose);
  }
  updateRigidGraspConfigurationUi();
  syncObjectGizmoToCanonicalPose();
  return rigidGraspStateSnapshot();
}

async function loadRigidGraspConfiguration(fetchImpl = fetch) {
  rigidGraspState.status = "LOADING";
  rigidGraspState.error = null;
  updateRigidGraspConfigurationUi();
  try {
    return applyBackendRigidGraspState(await requestGraspConfiguration(fetchImpl));
  } catch (error) {
    rigidGraspState.status = "UNAVAILABLE";
    rigidGraspState.error = error && error.message
      ? error.message : "Rigid grasp configuration is unavailable";
    updateRigidGraspConfigurationUi();
    updateObjectWaypointPlanningUi();
    return rigidGraspStateSnapshot();
  }
}

function invalidatePlanningForGraspChange(reason) {
  objectGlobalPlanState.requestGeneration += 1;
  objectGlobalPlanState.status = "INVALIDATED";
  objectGlobalPlanState.plan = null;
  objectGlobalPlanState.error = reason;
  clearPlannedTrajectory();
  invalidateAllPhase4Validation(reason);
  updateObjectGlobalPlanUi();
  updateObjectWaypointPlanningUi();
}

function updateRigidGraspDraftSide(side, pose, reason = "RIGID GRASP DRAFT CHANGED") {
  if (!["left", "right"].includes(side)) {
    throw new TypeError("grasp side must be left or right");
  }
  if (rigidGraspState.state !== "GRASP_UNLOCKED" || !rigidGraspState.draft) {
    throw new Error("Rigid grasp draft is not editable while locked or unavailable");
  }
  const normalized = normalizeObjectRelativeGraspPose(pose, `${side} grasp`);
  rigidGraspState.draft = {
    ...copyGraspContent(rigidGraspState.draft),
    [side]: copyGraspPose(normalized),
  };
  rigidGraspState.draftSource = "OPERATOR_EDITED_DRAFT";
  rigidGraspState.draftContentRevision = "LOCAL_DRAFT_MODIFIED — LOCK REQUIRED";
  rigidGraspState.authoritativeRevision = null;
  rigidGraspState.alignmentStatus = "EDITED — CURRENT-POSE ALIGNMENT NOT CLAIMED";
  rigidGraspState.alignmentResidual = null;
  rigidGraspState.error = null;
  applyObjectPreviewPose(objectPreviewState.pose || INITIAL_SYNTHETIC_OBJECT_POSE);
  invalidatePlanningForGraspChange(reason);
  updateRigidGraspConfigurationUi();
  return rigidGraspStateSnapshot();
}

function graspPoseFromNumericControls(side) {
  const title = side === "left" ? "Left" : "Right";
  const read = (suffix) => {
    const element = document.getElementById(`digitalTwin${title}Grasp${suffix}`);
    if (!element) throw new Error(`Missing ${side} grasp field ${suffix}`);
    return element.valueAsNumber;
  };
  return normalizeObjectRelativeGraspPose({
    translationM: [read("X"), read("Y"), read("Z")],
    rpyRad: [
      rotationDisplayToRadians(read("RollDeg"), angleUnitState.unit),
      rotationDisplayToRadians(read("PitchDeg"), angleUnitState.unit),
      rotationDisplayToRadians(read("YawDeg"), angleUnitState.unit),
    ],
  }, `${side} grasp`);
}

function applyGraspNumericEditor(side) {
  try {
    return updateRigidGraspDraftSide(
      side,
      graspPoseFromNumericControls(side),
      `${side.toUpperCase()} RIGID GRASP NUMERIC EDIT`,
    );
  } catch (error) {
    rigidGraspState.error = error && error.message
      ? error.message : `Invalid ${side} grasp configuration`;
    updateRigidGraspConfigurationUi();
    return rigidGraspStateSnapshot();
  }
}

async function lockRigidGraspConfiguration(fetchImpl = fetch) {
  if (!rigidGraspState.draft || rigidGraspState.requestInFlight) {
    return rigidGraspStateSnapshot();
  }
  invalidatePlanningForGraspChange("RIGID GRASP LOCK REQUESTED");
  rigidGraspState.requestInFlight = true;
  rigidGraspState.error = null;
  updateRigidGraspConfigurationUi();
  try {
    return applyBackendRigidGraspState(
      await requestLockGraspConfiguration(rigidGraspState.draft, fetchImpl),
    );
  } catch (error) {
    rigidGraspState.requestInFlight = false;
    rigidGraspState.error = error && error.message
      ? error.message : "Rigid grasp Lock failed";
    updateRigidGraspConfigurationUi();
    return rigidGraspStateSnapshot();
  }
}

async function unlockRigidGraspConfiguration(fetchImpl = fetch) {
  if (rigidGraspState.requestInFlight) return rigidGraspStateSnapshot();
  invalidatePlanningForGraspChange("RIGID GRASP UNLOCK REQUESTED");
  rigidGraspState.requestInFlight = true;
  rigidGraspState.error = null;
  updateRigidGraspConfigurationUi();
  try {
    return applyBackendRigidGraspState(
      await requestUnlockGraspConfiguration(fetchImpl),
    );
  } catch (error) {
    rigidGraspState.requestInFlight = false;
    rigidGraspState.error = error && error.message
      ? error.message : "Rigid grasp Unlock failed";
    updateRigidGraspConfigurationUi();
    return rigidGraspStateSnapshot();
  }
}

function setObjectPreviewUiError(message = null) {
  objectPreviewState.error = message;
  const errorElement = document.getElementById("digitalTwinObjectPreviewError");
  if (errorElement) errorElement.textContent = message || "NONE";
}

function applyMatrixToFrame(frame, matrixRows) {
  if (!frame) return;
  const matrix = new THREE.Matrix4();
  matrix.set(...matrixRows.flat());
  frame.matrixAutoUpdate = false;
  frame.matrix.copy(matrix);
  frame.matrixWorldNeedsUpdate = true;
  frame.updateMatrixWorld(true);
}

function createGraspFrame(name, markerColor) {
  const frame = new THREE.Group();
  frame.name = name;
  frame.add(new THREE.AxesHelper(0.18));
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.025, 16, 12),
    new THREE.MeshBasicMaterial({ color: markerColor }),
  );
  marker.name = `${name} origin marker`;
  frame.add(marker);
  return frame;
}

function createWaypointHoverGhostFrame(name, axesSize, markerColor) {
  const frame = new THREE.Group();
  frame.name = name;
  const axes = new THREE.AxesHelper(axesSize);
  axes.name = `${name} axes`;
  if (axes.material) {
    axes.material.transparent = true;
    axes.material.opacity = 0.72;
    axes.material.depthTest = false;
  }
  axes.renderOrder = 30;
  frame.add(axes);
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.021, 14, 10),
    new THREE.MeshBasicMaterial({
      color: markerColor, transparent: true, opacity: 0.42, depthTest: false,
    }),
  );
  marker.name = `${name} ghost origin marker`;
  marker.renderOrder = 31;
  frame.add(marker);
  return frame;
}

function initializeObjectWaypointHoverPreview() {
  objectWaypointHoverGroup = new THREE.Group();
  objectWaypointHoverGroup.name = "Center Waypoint Hover Frame Ghosts — DEBUG ONLY";
  objectWaypointHoverGroup.visible = false;
  objectWaypointHoverFrames.center = createWaypointHoverGhostFrame(
    "Hovered Center Frame Ghost", 0.25, 0xfacc15,
  );
  objectWaypointHoverFrames.left = createWaypointHoverGhostFrame(
    "Hovered Left Frame Ghost", 0.19, 0x22d3ee,
  );
  objectWaypointHoverFrames.right = createWaypointHoverGhostFrame(
    "Hovered Right Frame Ghost", 0.19, 0xf472b6,
  );
  objectWaypointHoverGroup.add(
    objectWaypointHoverFrames.center,
    objectWaypointHoverFrames.left,
    objectWaypointHoverFrames.right,
  );
  scene.add(objectWaypointHoverGroup);
}

function updateObjectWaypointHoverUi() {
  const element = document.getElementById("digitalTwinObjectWaypointHoverPreview");
  if (!element) return;
  element.textContent = objectWaypointHoverState.visible
    ? `${objectWaypointHoverState.identifier} — CENTER / LEFT / RIGHT`
    : "NONE";
}

function getObjectWaypointHoverPreviewState() {
  return { ...objectWaypointHoverState };
}

function hideObjectWaypointHoverPreview() {
  if (objectWaypointHoverGroup) objectWaypointHoverGroup.visible = false;
  objectWaypointHoverState.visible = false;
  objectWaypointHoverState.index = null;
  objectWaypointHoverState.identifier = null;
  objectWaypointHoverState.error = null;
  updateObjectWaypointHoverUi();
  render();
  return getObjectWaypointHoverPreviewState();
}

function showObjectWaypointHoverPreview(index) {
  try {
    if (!objectWaypointHoverGroup) throw new Error("Waypoint hover preview is not initialized");
    const waypoint = objectWaypointPlanningState.waypoints[index];
    if (!waypoint) throw new RangeError("Waypoint hover index is out of range");
    const graspContent = currentRigidGraspContent();
    if (!graspContent) throw new Error("Rigid grasp content is unavailable");
    const pose = {
      translationM: [...waypoint.translation_m],
      rpyRad: waypoint.rpy_rad
        ? [...waypoint.rpy_rad]
        : [...objectWaypointPlanningState.fixedOrientationRpyRad],
    };
    const worldTCenter = matrix4FromTranslationRpy(pose);
    const transforms = computeWorldGraspFrameMatrices(pose, graspContent);
    applyMatrixToFrame(objectWaypointHoverFrames.center, worldTCenter);
    applyMatrixToFrame(objectWaypointHoverFrames.left, transforms.worldTLeft);
    applyMatrixToFrame(objectWaypointHoverFrames.right, transforms.worldTRight);
    objectWaypointHoverGroup.visible = true;
    objectWaypointHoverState.visible = true;
    objectWaypointHoverState.index = index;
    objectWaypointHoverState.identifier = waypoint.identifier;
    objectWaypointHoverState.error = null;
    updateObjectWaypointHoverUi();
    render();
  } catch (error) {
    if (objectWaypointHoverGroup) objectWaypointHoverGroup.visible = false;
    objectWaypointHoverState.visible = false;
    objectWaypointHoverState.index = null;
    objectWaypointHoverState.identifier = null;
    objectWaypointHoverState.error = error && error.message
      ? error.message : "Waypoint hover preview failed";
    updateObjectWaypointHoverUi();
    render();
  }
  return getObjectWaypointHoverPreviewState();
}

function createObjectGraspPreview() {
  objectFrame = new THREE.Group();
  objectFrame.name = "Center / Coordination Frame C";
  objectFrame.visible = false;

  objectFrameAxes = new THREE.AxesHelper(0.24);
  objectFrameAxes.name = "Center / Coordination Frame C axes";
  objectFrame.add(objectFrameAxes);

  leftGraspFrame = createGraspFrame("Left Grasp Frame L", 0x22d3ee);
  rightGraspFrame = createGraspFrame("Right Grasp Frame R", 0xf472b6);
  leftGraspFrame.visible = false;
  rightGraspFrame.visible = false;
  scene.add(objectFrame);
  scene.add(leftGraspFrame);
  scene.add(rightGraspFrame);
  initializeObjectWaypointHoverPreview();
  updateObjectGraspRelativeUi();
}

function objectDragStateSnapshot() {
  return {
    enabled: objectDragState.enabled,
    dragging: objectDragState.dragging,
    mode: objectDragState.mode,
    translationAxes: { x: true, y: true, z: true },
    rotationEnabled: objectDragState.rotationEnabled,
    scaleEnabled: objectDragState.scaleEnabled,
    orientationLocked: rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER
      && rigidGraspState.state !== "GRASP_LOCKED",
    ownership: objectDragState.ownership,
    error: objectDragState.error,
    targetTranslationM: objectGizmoTarget
      ? [
        objectGizmoTarget.position.x,
        objectGizmoTarget.position.y,
        objectGizmoTarget.position.z,
      ]
      : null,
  };
}

function getObjectDragState() {
  return objectDragStateSnapshot();
}

function updateObjectDragUi() {
  const state = objectDragStateSnapshot();
  const values = {
    digitalTwinObjectDragState: state.enabled ? "ENABLED" : "DISABLED",
    digitalTwinObjectDragOwnership: state.ownership,
    digitalTwinObjectDragError: state.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const button = document.getElementById("digitalTwinObjectDragToggle");
  if (button) {
    button.textContent = state.enabled
      ? "Stop 3D Editing"
      : "Move Selected Item in 3D";
    button.setAttribute("aria-pressed", String(state.enabled));
  }
  const centerGizmo = document.getElementById("digitalTwinCenterWaypointGizmoToggle");
  if (centerGizmo) {
    const active = state.enabled
      && rigidGraspState.state === "GRASP_LOCKED"
      && rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER;
    centerGizmo.textContent = active ? "Disable Center Gizmo" : "Enable Center Gizmo";
    centerGizmo.setAttribute("aria-pressed", String(active));
  }
}

function syncObjectGizmoToCanonicalPose(pose = objectPreviewState.pose) {
  if (typeof objectGizmoTarget === "undefined" || !objectGizmoTarget || !pose) return;
  objectGizmoSynchronizing = true;
  let targetMatrixRows = matrix4FromTranslationRpy(pose);
  const content = currentRigidGraspContent();
  if (content && rigidGraspState.selectedFrame !== GRASP_SELECTED_FRAMES.CENTER) {
    const transforms = computeWorldGraspFrameMatrices(pose, content);
    targetMatrixRows = rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.LEFT_GRASP
      ? transforms.worldTLeft : transforms.worldTRight;
  }
  const targetMatrix = new THREE.Matrix4();
  targetMatrix.set(...targetMatrixRows.flat());
  targetMatrix.decompose(
    objectGizmoTarget.position,
    objectGizmoTarget.quaternion,
    objectGizmoTarget.scale,
  );
  objectGizmoTarget.updateMatrixWorld(true);
  objectGizmoSynchronizing = false;
}

function threeMatrixToRows(matrix) {
  const elements = matrix.elements;
  return [
    [elements[0], elements[4], elements[8], elements[12]],
    [elements[1], elements[5], elements[9], elements[13]],
    [elements[2], elements[6], elements[10], elements[14]],
    [elements[3], elements[7], elements[11], elements[15]],
  ];
}

function setRigidGraspSelectedFrame(frame) {
  if (!Object.values(GRASP_SELECTED_FRAMES).includes(frame)) {
    throw new TypeError("Selected grasp frame is invalid");
  }
  if (rigidGraspState.state === "GRASP_LOCKED" && frame !== GRASP_SELECTED_FRAMES.CENTER) {
    rigidGraspState.error = "Left/Right grasp editing is disabled while locked";
    frame = GRASP_SELECTED_FRAMES.CENTER;
  }
  rigidGraspState.selectedFrame = frame;
  if (frame === GRASP_SELECTED_FRAMES.CENTER
      && rigidGraspState.gizmoMode === "rotate"
      && rigidGraspState.state !== "GRASP_LOCKED") {
    rigidGraspState.gizmoMode = "translate";
  }
  if (objectTransformControls) {
    objectTransformControls.setMode(rigidGraspState.gizmoMode);
    objectTransformControls.setSpace(
      frame === GRASP_SELECTED_FRAMES.CENTER && rigidGraspState.gizmoMode === "translate"
        ? "world" : "local",
    );
  }
  const selector = document.getElementById("digitalTwinGraspSelectedFrameControl");
  if (selector) selector.value = frame;
  syncObjectGizmoToCanonicalPose();
  updateRigidGraspConfigurationUi();
  updateObjectDragUi();
  render();
  return rigidGraspStateSnapshot();
}

function setRigidGraspGizmoMode(mode) {
  if (!["translate", "rotate"].includes(mode)) {
    throw new TypeError("Rigid grasp gizmo mode must be translate or rotate");
  }
  if (rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER
      && mode === "rotate"
      && rigidGraspState.state !== "GRASP_LOCKED") {
    throw new Error("Center rotation is available for locked rigid Center waypoints only");
  }
  if (rigidGraspState.selectedFrame !== GRASP_SELECTED_FRAMES.CENTER
      && rigidGraspState.state !== "GRASP_UNLOCKED") {
    throw new Error("Locked grasp frames cannot be edited");
  }
  rigidGraspState.gizmoMode = mode;
  objectDragState.mode = mode;
  objectDragState.rotationEnabled = mode === "rotate";
  if (objectTransformControls) {
    objectTransformControls.setMode(mode);
    objectTransformControls.setSpace(
      rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER && mode === "translate"
        ? "world" : "local",
    );
  }
  updateRigidGraspConfigurationUi();
  updateObjectDragUi();
  return rigidGraspStateSnapshot();
}

function setObjectDragEnabled(enabled) {
  if (typeof enabled !== "boolean") {
    throw new TypeError("Object Drag Mode enabled must be boolean");
  }
  objectDragState.enabled = enabled;
  objectDragState.dragging = false;
  objectDragState.ownership = enabled ? "MANUAL OBJECT DRAG" : "NONE";
  objectDragState.error = null;
  if (objectTransformControls) {
    objectTransformControls.enabled = enabled;
    objectTransformControls.visible = enabled;
  }
  if (controls) controls.enabled = true;
  syncObjectGizmoToCanonicalPose();
  updateObjectDragUi();
  render();
  return objectDragStateSnapshot();
}

function initializeObjectTransformControls() {
  objectGizmoTarget = new THREE.Object3D();
  objectGizmoTarget.name = "Selected Center/Grasp Gizmo Target — OFFLINE";
  scene.add(objectGizmoTarget);

  objectTransformControls = new TransformControls(camera, renderer.domElement);
  objectTransformControls.name = "Center/Rigid Grasp TransformControls — OFFLINE";
  objectTransformControls.setMode("translate");
  objectTransformControls.setSpace("world");
  objectTransformControls.showX = true;
  objectTransformControls.showY = true;
  objectTransformControls.showZ = true;
  objectTransformControls.enabled = false;
  objectTransformControls.visible = false;
  objectTransformControls.attach(objectGizmoTarget);
  objectTransformControls.addEventListener("dragging-changed", (event) => {
    objectDragState.dragging = Boolean(event.value);
    if (controls) controls.enabled = !event.value;
    if (event.value) {
      pauseObjectTrajectoryForManualPreview();
      objectDragState.ownership = "MANUAL OBJECT DRAG";
    }
    updateObjectDragUi();
  });
  objectTransformControls.addEventListener("objectChange", () => {
    if (objectGizmoSynchronizing || !objectDragState.enabled) return;
    const currentPose = objectPreviewState.pose || INITIAL_SYNTHETIC_OBJECT_POSE;
    try {
      if (rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER) {
        objectGizmoTarget.updateMatrixWorld(true);
        const centerPose = poseFromRigidMatrix4(threeMatrixToRows(objectGizmoTarget.matrixWorld));
        applyObjectPreviewPose({
          translationM: [...centerPose.translationM],
          rpyRad: [...centerPose.rpyRad],
        });
        invalidateAllPhase4Validation(
          rigidGraspState.gizmoMode === "rotate"
            ? "CENTER ORIENTATION CHANGED" : "OBJECT TRANSLATION CHANGED",
        );
      } else {
        if (rigidGraspState.state !== "GRASP_UNLOCKED") {
          throw new Error("Rigid grasp gizmo editing is disabled while locked");
        }
        objectGizmoTarget.updateMatrixWorld(true);
        const worldTGrasp = threeMatrixToRows(objectGizmoTarget.matrixWorld);
        const worldTObject = matrix4FromTranslationRpy(currentPose);
        const objectTGrasp = multiplyMatrix4(
          inverseRigidMatrix4(worldTObject),
          worldTGrasp,
        );
        const side = rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.LEFT_GRASP
          ? "left" : "right";
        updateRigidGraspDraftSide(
          side,
          poseFromRigidMatrix4(objectTGrasp),
          `${side.toUpperCase()} RIGID GRASP GIZMO EDIT`,
        );
      }
      objectDragState.error = null;
    } catch (error) {
      objectDragState.error = error && error.message
        ? error.message : "Object drag failed";
      syncObjectGizmoToCanonicalPose(currentPose);
    }
    updateObjectDragUi();
  });
  objectTransformControls.addEventListener("change", render);
  scene.add(objectTransformControls);
  syncObjectGizmoToCanonicalPose();
  updateObjectDragUi();
}

function applyObjectPreviewPose(pose) {
  if (!objectFrame || !leftGraspFrame || !rightGraspFrame) {
    throw new Error("Object/grasp preview scene is not initialized");
  }
  const normalizedPose = normalizeObjectPreviewPose(pose);
  const worldTObject = matrix4FromTranslationRpy(normalizedPose);
  applyMatrixToFrame(objectFrame, worldTObject);
  objectFrame.visible = true;
  if (objectFrameAxes) objectFrameAxes.visible = objectPreviewState.objectFrameVisible;
  const graspContent = currentRigidGraspContent();
  if (graspContent) {
    const transforms = computeWorldGraspFrameMatrices(normalizedPose, graspContent);
    applyMatrixToFrame(leftGraspFrame, transforms.worldTLeft);
    applyMatrixToFrame(rightGraspFrame, transforms.worldTRight);
    leftGraspFrame.visible = objectPreviewState.graspFramesVisible;
    rightGraspFrame.visible = objectPreviewState.graspFramesVisible;
  } else {
    leftGraspFrame.visible = false;
    rightGraspFrame.visible = false;
  }
  objectPreviewState.pose = copyObjectPose(normalizedPose);
  if (typeof syncObjectGizmoToCanonicalPose === "function") {
    syncObjectGizmoToCanonicalPose(normalizedPose);
  }
  writeObjectPoseControls(normalizedPose);
  setObjectPreviewUiError(null);
  render();
  return getObjectPreviewState();
}

function objectPoseFromControls() {
  const value = (id) => {
    const element = document.getElementById(id);
    if (!element) throw new Error(`Missing object pose control: ${id}`);
    return element.valueAsNumber;
  };
  const currentPose = objectPreviewState.pose || INITIAL_SYNTHETIC_OBJECT_POSE;
  return normalizeObjectPreviewPose({
    translationM: [
      value("digitalTwinObjectX"),
      value("digitalTwinObjectY"),
      value("digitalTwinObjectZ"),
    ],
    rpyRad: [
      rotationDisplayToRadians(value("digitalTwinObjectRoll"), angleUnitState.unit),
      rotationDisplayToRadians(value("digitalTwinObjectPitch"), angleUnitState.unit),
      rotationDisplayToRadians(value("digitalTwinObjectYaw"), angleUnitState.unit),
    ],
  });
}

function applyObjectPoseFromControls() {
  pauseObjectTrajectoryForManualPreview();
  try {
    const state = applyObjectPreviewPose(objectPoseFromControls());
    invalidateAllPhase4Validation("OBJECT POSE INPUT CHANGED");
    return state;
  } catch (error) {
    const message = error && error.message ? error.message : "Invalid object pose";
    setObjectPreviewUiError(message);
    return getObjectPreviewState();
  }
}

function writeObjectPoseControls(pose) {
  const values = [
    ["digitalTwinObjectX", pose.translationM[0]],
    ["digitalTwinObjectY", pose.translationM[1]],
    ["digitalTwinObjectZ", pose.translationM[2]],
    ["digitalTwinObjectRoll", pose.rpyRad[0], true],
    ["digitalTwinObjectPitch", pose.rpyRad[1], true],
    ["digitalTwinObjectYaw", pose.rpyRad[2], true],
  ];
  values.forEach(([id, value, rotation = false]) => {
    const element = document.getElementById(id);
    if (!element) return;
    if (rotation) writeOperatorRotationInput(element, value);
    else element.value = String(value);
  });
}

function currentDisplayedModelTipWorldMatrices() {
  if (loadState.status !== "READY" || !robot || !robot.links) {
    throw new Error("Digital Twin model pose is not ready");
  }
  if (!["LIVE JOINT FEEDBACK", "OFFLINE CODE CONFIG"].includes(mainModelState.source)) {
    throw new Error("No accepted Live or offline Planning Start model pose is available");
  }
  if (mainModelState.source === "LIVE JOINT FEEDBACK"
      && !hasValidLiveFeedbackOwnership(Date.now())) {
    throw new Error("Live model pose is no longer fresh enough for a frame snapshot");
  }
  if (mainModelState.source === "OFFLINE CODE CONFIG"
      && planningStartState.status !== "READY") {
    throw new Error("Offline Planning Start model pose is no longer accepted");
  }
  const leftTip = robot.links[MODEL_TCP_LINKS.left];
  const rightTip = robot.links[MODEL_TCP_LINKS.right];
  if (!leftTip || !rightTip || !leftTip.matrixWorld || !rightTip.matrixWorld) {
    throw new Error("Digital Twin left_J6/right_J6 model-tip transforms are unavailable");
  }
  robot.updateMatrixWorld(true);
  return {
    left: threeMatrixToRows(leftTip.matrixWorld),
    right: threeMatrixToRows(rightTip.matrixWorld),
    source: mainModelState.source,
    configurationSource: mainModelState.configurationSource,
  };
}

function resetGraspFramesToCurrentRobotPose({
  invalidatePlanning = true,
  automaticInitialization = false,
  sessionAction = "RESET",
} = {}) {
  if (rigidGraspState.status !== "READY"
      || rigidGraspState.state !== "GRASP_UNLOCKED"
      || !rigidGraspState.draft) {
    throw new Error("Edit/Unlock the grasp before resetting frames");
  }
  const tips = currentDisplayedModelTipWorldMatrices();
  const resetGeometry = deriveGraspFrameResetFromWorldTips(tips.left, tips.right);
  const alignment = verifyGraspFrameAlignment({
    centerPose: resetGeometry.centerPose,
    draft: resetGeometry.draft,
    worldTLeft: tips.left,
    worldTRight: tips.right,
  });
  if (!alignment.ok) {
    const message = (
      "Grasp frame alignment self-check failed: "
      + `translation residual ${alignment.maxTranslationM.toExponential(3)} m, `
      + `orientation residual ${alignment.maxOrientationRad.toExponential(3)} rad`
    );
    rigidGraspState.alignmentStatus = "ALIGNMENT ERROR";
    rigidGraspState.alignmentResidual = {
      maxTranslationM: alignment.maxTranslationM,
      maxOrientationRad: alignment.maxOrientationRad,
    };
    rigidGraspState.error = message;
    setObjectPreviewUiError(message);
    updateRigidGraspConfigurationUi();
    throw new Error(message);
  }
  rigidGraspState.draft = copyGraspContent(resetGeometry.draft);
  rigidGraspState.draftSource = `CURRENT DIGITAL TWIN MODEL TIPS — ${tips.source}`;
  rigidGraspState.draftContentRevision = automaticInitialization
    ? "LOCAL DRAFT INITIALIZED FROM CURRENT MODEL TIPS — LOCK REQUIRED"
    : sessionAction === "NEW_GRASP"
      ? "LOCAL NEW GRASP FROM CURRENT MODEL TIPS — LOCK REQUIRED"
      : "LOCAL DRAFT RESET FROM CURRENT MODEL TIPS — LOCK REQUIRED";
  rigidGraspState.authoritativeRevision = null;
  rigidGraspState.alignmentStatus = "SNAPSHOTTED MAIN J6 ALIGNMENT VERIFIED";
  rigidGraspState.alignmentResidual = {
    maxTranslationM: alignment.maxTranslationM,
    maxOrientationRad: alignment.maxOrientationRad,
  };
  rigidGraspState.error = null;
  pauseObjectTrajectoryForManualPreview();
  setObjectPreviewVisibility({
    objectVisible: false,
    objectFrameVisible: true,
    graspFramesVisible: true,
  });
  applyObjectPreviewPose(resetGeometry.centerPose);
  writeRigidGraspEditorFields();
  if (invalidatePlanning) {
    invalidatePlanningForGraspChange(sessionAction === "NEW_GRASP"
      ? "NEW GRASP STARTED FROM CURRENT ROBOT POSE"
      : "GRASP FRAMES RESET FROM CURRENT ROBOT POSE");
  }
  updateRigidGraspConfigurationUi();
  return getObjectPreviewState();
}

function initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady() {
  if (!initialGraspFrameSnapshotPending || initialGraspFrameSnapshotComplete) {
    return false;
  }
  if (rigidGraspState.status !== "READY"
      || loadState.status !== "READY"
      || !robot
      || !["LIVE JOINT FEEDBACK", "OFFLINE CODE CONFIG"].includes(mainModelState.source)) {
    return false;
  }
  try {
    // A locked snapshot remains byte-for-byte authoritative.  Only place its
    // Center reference at the current tip midpoint; never derive/replace its
    // saved Center-relative Left/Right grasp transforms on page load.
    if (rigidGraspState.state === "GRASP_LOCKED") {
      const tips = currentDisplayedModelTipWorldMatrices();
      const resetGeometry = deriveGraspFrameResetFromWorldTips(tips.left, tips.right);
      setObjectPreviewVisibility({
        objectVisible: false,
        objectFrameVisible: true,
        graspFramesVisible: true,
      });
      applyObjectPreviewPose(resetGeometry.centerPose);
      initialGraspFrameSnapshotPending = false;
      initialGraspFrameSnapshotComplete = true;
      return true;
    }
    if (rigidGraspState.state !== "GRASP_UNLOCKED" || !rigidGraspState.draft) {
      return false;
    }
    resetGraspFramesToCurrentRobotPose({
      invalidatePlanning: false,
      automaticInitialization: true,
    });
    initialGraspFrameSnapshotPending = false;
    initialGraspFrameSnapshotComplete = true;
    return true;
  } catch (error) {
    console.debug(
      "[DualArmDigitalTwin] current-model grasp-frame initialization is waiting",
      error,
    );
    return false;
  }
}

function resetObjectPreview() {
  return resetGraspFramesToCurrentRobotPose();
}

// Preserve the backend-provided numeric grasp draft. This is intentionally a
// plain Unlock and must never imply a reset to the displayed robot pose.
function editCurrentGrasp(fetchImpl = fetch) {
  return unlockRigidGraspConfiguration(fetchImpl);
}

async function startNewGraspFromCurrentRobotPose(fetchImpl = fetch) {
  if (rigidGraspState.status !== "READY"
      || rigidGraspState.state !== "GRASP_LOCKED"
      || rigidGraspState.requestInFlight) {
    const message = "A locked grasp is required before starting a new grasp setup";
    rigidGraspState.error = message;
    setObjectPreviewUiError(message);
    updateRigidGraspConfigurationUi();
    return rigidGraspStateSnapshot();
  }

  const unlockedState = await unlockRigidGraspConfiguration(fetchImpl);
  if (unlockedState.state !== "GRASP_UNLOCKED"
      || rigidGraspState.state !== "GRASP_UNLOCKED") {
    const message = rigidGraspState.error
      || "Could not unlock the current grasp before starting a new setup";
    setObjectPreviewUiError(message);
    updateRigidGraspConfigurationUi();
    return rigidGraspStateSnapshot();
  }

  try {
    resetGraspFramesToCurrentRobotPose({ sessionAction: "NEW_GRASP" });
    objectWaypointPlanningState.approachWaypoints = [];
    updateApproachWaypointUi();
  } catch (error) {
    const message = error && error.message
      ? error.message : "New grasp alignment from current robot pose failed";
    rigidGraspState.error = message;
    setObjectPreviewUiError(message);
    updateRigidGraspConfigurationUi();
  }
  return rigidGraspStateSnapshot();
}

function setObjectPreviewVisibility({
  objectVisible = objectPreviewState.objectVisible,
  objectFrameVisible = objectPreviewState.objectFrameVisible,
  graspFramesVisible = objectPreviewState.graspFramesVisible,
} = {}) {
  if (![objectVisible, objectFrameVisible, graspFramesVisible].every(
    (value) => typeof value === "boolean",
  )) {
    throw new TypeError("Object preview visibility values must be boolean");
  }
  // Preserve the public state shape, but the removed synthetic mesh can never
  // be made visible in the normal planning scene.
  objectPreviewState.objectVisible = false;
  objectPreviewState.objectFrameVisible = objectFrameVisible;
  objectPreviewState.graspFramesVisible = graspFramesVisible;
  if (objectFrameAxes) objectFrameAxes.visible = objectFrameVisible;
  const hasGrasp = currentRigidGraspContent() !== null;
  if (leftGraspFrame) leftGraspFrame.visible = graspFramesVisible && hasGrasp;
  if (rightGraspFrame) rightGraspFrame.visible = graspFramesVisible && hasGrasp;
  render();
  return getObjectPreviewState();
}

function syncObjectPreviewVisibilityFromControls() {
  const checked = (id) => {
    const element = document.getElementById(id);
    return element ? element.checked : true;
  };
  return setObjectPreviewVisibility({
    objectVisible: false,
    objectFrameVisible: checked("digitalTwinShowObjectFrame"),
    graspFramesVisible: checked("digitalTwinShowGraspFrames"),
  });
}

function bindObjectPreviewControls() {
  const applyButton = document.getElementById("digitalTwinApplyObjectPose");
  const resetButton = document.getElementById("digitalTwinResetObjectPreview");
  if (applyButton) applyButton.addEventListener("click", applyObjectPoseFromControls);
  if (resetButton) resetButton.addEventListener("click", () => {
    try {
      resetObjectPreview();
      setObjectPreviewUiError(null);
    } catch (error) {
      setObjectPreviewUiError(
        error && error.message ? error.message : "Frame reset failed",
      );
    }
  });
  const dragButton = document.getElementById("digitalTwinObjectDragToggle");
  if (dragButton) dragButton.addEventListener("click", () => {
    setObjectDragEnabled(!objectDragState.enabled);
  });
  for (const id of [
    "digitalTwinShowObjectFrame",
    "digitalTwinShowGraspFrames",
  ]) {
    const element = document.getElementById(id);
    if (element) element.addEventListener("change", syncObjectPreviewVisibilityFromControls);
  }
}

function bindRigidGraspConfigurationControls() {
  const selector = document.getElementById("digitalTwinGraspSelectedFrameControl");
  if (selector) selector.addEventListener("change", (event) => {
    setRigidGraspSelectedFrame(event.target.value);
  });
  const frameButtons = {
    digitalTwinSelectObjectFrame: GRASP_SELECTED_FRAMES.CENTER,
    digitalTwinSelectLeftGraspFrame: GRASP_SELECTED_FRAMES.LEFT_GRASP,
    digitalTwinSelectRightGraspFrame: GRASP_SELECTED_FRAMES.RIGHT_GRASP,
  };
  Object.entries(frameButtons).forEach(([id, frame]) => {
    const button = document.getElementById(id);
    if (button) button.addEventListener("click", () => {
      setRigidGraspSelectedFrame(frame);
      setObjectDragEnabled(true);
    });
  });
  const translate = document.getElementById("digitalTwinGraspGizmoTranslate");
  if (translate) translate.addEventListener("click", () => {
    setRigidGraspGizmoMode("translate");
  });
  const rotate = document.getElementById("digitalTwinGraspGizmoRotate");
  if (rotate) rotate.addEventListener("click", () => {
    setRigidGraspGizmoMode("rotate");
  });
  const applyLeft = document.getElementById("digitalTwinApplyLeftGrasp");
  if (applyLeft) applyLeft.addEventListener("click", () => applyGraspNumericEditor("left"));
  const applyRight = document.getElementById("digitalTwinApplyRightGrasp");
  if (applyRight) applyRight.addEventListener("click", () => applyGraspNumericEditor("right"));
  for (const side of ["Left", "Right"]) {
    for (const suffix of ["X", "Y", "Z", "RollDeg", "PitchDeg", "YawDeg"]) {
      const field = document.getElementById(`digitalTwin${side}Grasp${suffix}`);
      if (field) field.addEventListener("change", () => (
        applyGraspNumericEditor(side.toLowerCase())
      ));
    }
  }
  const lock = document.getElementById("digitalTwinLockGraspConfiguration");
  if (lock) lock.addEventListener("click", () => lockRigidGraspConfiguration());
  const unlock = document.getElementById("digitalTwinUnlockGraspConfiguration");
  if (unlock) unlock.addEventListener("click", () => editCurrentGrasp());
  const startNew = document.getElementById(
    "digitalTwinStartNewGraspFromCurrentRobotPose",
  );
  if (startNew) startNew.addEventListener("click", () => (
    startNewGraspFromCurrentRobotPose()
  ));
}

function disposeWaypointObject(object) {
  object.traverse((child) => {
    if (child.geometry && typeof child.geometry.dispose === "function") {
      child.geometry.dispose();
    }
    if (child.material && typeof child.material.dispose === "function") {
      child.material.dispose();
    }
  });
}

function initializeObjectWaypointVisualization() {
  objectWaypointVisualizationGroup = new THREE.Group();
  objectWaypointVisualizationGroup.name = (
    "Object Waypoint Markers — PRE-PLAN ONLY"
  );
  scene.add(objectWaypointVisualizationGroup);
  syncObjectWaypointVisualization();
}

function syncObjectWaypointVisualization() {
  if (!objectWaypointVisualizationGroup) return;
  for (const child of [...objectWaypointVisualizationGroup.children]) {
    objectWaypointVisualizationGroup.remove(child);
    disposeWaypointObject(child);
  }
  objectWaypointPath = null;
  const points = objectWaypointPlanningState.waypoints.map(
    (waypoint) => new THREE.Vector3(...waypoint.translation_m),
  );
  objectWaypointPlanningState.waypoints.forEach((waypoint, index) => {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.035, 16, 12),
      new THREE.MeshBasicMaterial({
        color: new THREE.Color().setHSL((0.12 + index * 0.17) % 1, 0.9, 0.62),
        depthTest: true,
      }),
    );
    marker.name = `Object Waypoint ${waypoint.identifier} — PRE-PLAN ONLY`;
    marker.position.copy(points[index]);
    marker.userData = {
      waypointIdentifier: waypoint.identifier,
      waypointIndex: index,
      semantic: objectWaypointVisualizationState.semantic,
    };
    objectWaypointVisualizationGroup.add(marker);
  });
  if (points.length >= 2) {
    objectWaypointPath = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: 0xfacc15, linewidth: 2 }),
    );
    objectWaypointPath.name = objectWaypointVisualizationState.semantic;
    objectWaypointVisualizationGroup.add(objectWaypointPath);
  }
  objectWaypointVisualizationState.markerCount = points.length;
  objectWaypointVisualizationState.pathOrder = (
    objectWaypointPlanningState.waypoints.map((waypoint) => waypoint.identifier)
  );
  render();
}

function getObjectWaypointVisualizationState() {
  return {
    semantic: objectWaypointVisualizationState.semantic,
    markerCount: objectWaypointVisualizationState.markerCount,
    pathOrder: [...objectWaypointVisualizationState.pathOrder],
    markerPositionsM: objectWaypointPlanningState.waypoints.map((waypoint) => ({
      identifier: waypoint.identifier,
      translation_m: [...waypoint.translation_m],
      rpy_rad: waypoint.rpy_rad ? [...waypoint.rpy_rad] : null,
      speed_percent: waypoint.speed_percent,
      acceleration_percent: waypoint.acceleration_percent,
    })),
    pathVisible: objectWaypointPlanningState.waypoints.length >= 2,
  };
}

function planningStartStateSnapshot() {
  return {
    left: planningStartState.left ? [...planningStartState.left] : null,
    right: planningStartState.right ? [...planningStartState.right] : null,
    jointNames: [...planningStartState.jointNames],
    units: planningStartState.units,
    source: planningStartState.source,
    status: planningStartState.status,
    error: planningStartState.error,
    offline: true,
    readOnly: true,
    liveRobotStateClaimed: false,
  };
}

function getPlanningStartState() {
    return planningStartStateSnapshot();
}

function setNextPlanningStartOverride(values, source) {
  const normalized = normalizePlanningStartState(values);
  if (typeof source !== "string" || !source.trim()) {
    throw new TypeError("Planning start override source must be non-empty");
  }
  nextPlanningStartOverride = {
    initialJointStateRad: {
      left: [...normalized.left],
      right: [...normalized.right],
    },
    planningStartStateSource: source.trim(),
  };
  return {
    armed: true,
    source: nextPlanningStartOverride.planningStartStateSource,
    initial_joint_state_rad: {
      left: [...nextPlanningStartOverride.initialJointStateRad.left],
      right: [...nextPlanningStartOverride.initialJointStateRad.right],
    },
    semantic: "ONE-SHOT NEXT GLOBAL PLAN OVERRIDE — NO MOTION",
  };
}

function normalizePlanningStartStateResponse(payload) {
  if (!payload || typeof payload !== "object" || payload.ok !== true) {
    throw new TypeError("Planning Start State response must be available");
  }
  const pose = normalizePlannedDualArmPose({
    left: payload.left,
    right: payload.right,
  });
  if (!Array.isArray(payload.joint_names)
      || payload.joint_names.length !== EXPECTED_JOINTS.length
      || payload.joint_names.some((name, index) => name !== EXPECTED_JOINTS[index])) {
    throw new RangeError("Planning Start State joint ordering is not canonical");
  }
  if (payload.units !== "radian") {
    throw new RangeError("Planning Start State units must be radian");
  }
  if (typeof payload.source !== "string" || !payload.source.trim()) {
    throw new TypeError("Planning Start State source must be non-empty");
  }
  return {
    left: pose.left,
    right: pose.right,
    jointNames: [...payload.joint_names],
    units: "radian",
    source: payload.source.trim(),
  };
}

async function loadPlanningStartState(fetchImpl = fetch) {
  const previousIdentity = planningStartState.status === "READY"
    ? JSON.stringify({
      left: planningStartState.left,
      right: planningStartState.right,
      source: planningStartState.source,
    })
    : null;
  planningStartState.status = "LOADING";
  planningStartState.error = null;
  try {
    const payload = await requestPlanningStartState(fetchImpl);
    const normalized = normalizePlanningStartStateResponse(payload);
    planningStartState.left = normalized.left;
    planningStartState.right = normalized.right;
    planningStartState.jointNames = normalized.jointNames;
    planningStartState.units = normalized.units;
    planningStartState.source = normalized.source;
    planningStartState.status = "READY";
    planningStartState.error = null;
    applyPlanningStartStateToMainWhenOffline(Date.now());
  } catch (error) {
    planningStartState.left = null;
    planningStartState.right = null;
    planningStartState.jointNames = [];
    planningStartState.source = "UNAVAILABLE";
    planningStartState.status = "UNAVAILABLE";
    planningStartState.error = error && error.message
      ? error.message : "Planning Start State is unavailable";
  }
  const currentIdentity = planningStartState.status === "READY"
    ? JSON.stringify({
      left: planningStartState.left,
      right: planningStartState.right,
      source: planningStartState.source,
    })
    : null;
  if (
    previousIdentity !== null
    && previousIdentity !== currentIdentity
    && objectGlobalPlanState.plan
  ) {
    invalidatePlanningForGraspChange("PLANNING START STATE CHANGED");
  }
  updateMirrorUi();
  updateObjectGlobalPlanUi();
  return planningStartStateSnapshot();
}

function objectWaypointPlanningStateSnapshot() {
  return {
    waypoints: objectWaypointPlanningState.waypoints.map((waypoint) => ({
      identifier: waypoint.identifier,
      translation_m: [...waypoint.translation_m],
      rpy_rad: waypoint.rpy_rad ? [...waypoint.rpy_rad] : null,
      speed_percent: waypoint.speed_percent,
      acceleration_percent: waypoint.acceleration_percent,
    })),
    approachWaypoints: objectWaypointPlanningState.approachWaypoints.map((waypoint) => ({
      identifier: waypoint.identifier,
      left: {translation_m: [...waypoint.left.translation_m], rpy_rad: [...waypoint.left.rpy_rad]},
      right: {translation_m: [...waypoint.right.translation_m], rpy_rad: [...waypoint.right.rpy_rad]},
    })),
    fixedOrientationRpyRad: [...objectWaypointPlanningState.fixedOrientationRpyRad],
    orientationSource: objectWaypointPlanningState.orientationSource,
    segmentDurationS: objectWaypointPlanningState.segmentDurationS,
    samplesPerSegment: objectWaypointPlanningState.samplesPerSegment,
    candidateAttemptsPerArm: objectWaypointPlanningState.candidateAttemptsPerArm,
    status: objectWaypointPlanningState.status,
    error: objectWaypointPlanningState.error,
    planning: objectWaypointPlanningState.planning,
  };
}

function getObjectWaypointPlanningState() {
  return objectWaypointPlanningStateSnapshot();
}

function updateOperatorPlanStatus() {
  const element = document.getElementById("digitalTwinOperatorPlanStatus");
  if (!element) return;
  const authorityReady = phase3PlanningAuthorityIdentity() !== null;
  if (objectWaypointPlanningState.planning || objectGlobalPlanState.status === "PLANNING") {
    element.textContent = "PLANNING…";
  } else if (objectGlobalPlanState.status === "READY" && objectGlobalPlanState.plan) {
    element.textContent = "PLAN READY";
  } else if (objectGlobalPlanState.status === "BLOCKED" && !authorityReady) {
    element.textContent = "LOCK GRASP TO CONTINUE";
  } else if (["FAILED", "BLOCKED"].includes(objectGlobalPlanState.status)) {
    element.textContent = "FAILED";
  } else if (!authorityReady) {
    element.textContent = "LOCK GRASP TO CONTINUE";
  } else if (objectWaypointPlanningState.waypoints.length < 2) {
    element.textContent = "ADD AT LEAST 2 WAYPOINTS";
  } else {
    element.textContent = "READY TO PLAN";
  }
}

function setIndependentApproachWaypoints(nextWaypoints, reason = "INDEPENDENT APPROACH WAYPOINTS CHANGED") {
  if (rigidGraspState.state !== "GRASP_UNLOCKED" || rigidGraspState.status !== "READY") {
    rigidGraspState.error = "Approach waypoints can only be edited before Lock Grasp";
    updateRigidGraspConfigurationUi();
    return objectWaypointPlanningStateSnapshot();
  }
  try {
    const normalized = normalizeApproachWaypoints(nextWaypoints);
    const identifiers = normalized.map((waypoint) => waypoint.identifier.toLocaleLowerCase());
    if (new Set(identifiers).size !== identifiers.length) {
      throw new RangeError("Approach waypoint identifiers must be unique");
    }
    objectWaypointPlanningState.approachWaypoints = normalized.map((waypoint) => ({
      identifier: waypoint.identifier,
      left: {
        translation_m: [...waypoint.left.translation_m],
        rpy_rad: [...waypoint.left.rpy_rad],
      },
      right: {
        translation_m: [...waypoint.right.translation_m],
        rpy_rad: [...waypoint.right.rpy_rad],
      },
    }));
    rigidGraspState.error = null;
    invalidatePlanningForGraspChange(reason);
  } catch (error) {
    rigidGraspState.error = error && error.message
      ? error.message : "Invalid Independent Approach waypoint";
    updateRigidGraspConfigurationUi();
  }
  updateApproachWaypointUi();
  return objectWaypointPlanningStateSnapshot();
}

function updateApproachWaypointUi() {
  const count = document.getElementById("digitalTwinApproachWaypointCount");
  if (count) count.textContent = String(objectWaypointPlanningState.approachWaypoints.length);
  const list = document.getElementById("digitalTwinApproachWaypointList");
  const editable = rigidGraspState.state === "GRASP_UNLOCKED"
    && rigidGraspState.status === "READY";
  if (list) {
    list.replaceChildren();
    objectWaypointPlanningState.approachWaypoints.forEach((waypoint, index) => {
      const card = document.createElement("div");
      card.className = "digital-twin-approach-waypoint-card";
      card.dataset.approachWaypointIndex = String(index);

      const header = document.createElement("div");
      header.className = "digital-twin-approach-waypoint-header";
      const idInput = document.createElement("input");
      idInput.value = waypoint.identifier;
      idInput.disabled = !editable;
      idInput.setAttribute("aria-label", `Approach waypoint ${index + 1} identifier`);
      header.append(idInput);

      const actions = [
        ["↑", () => moveIndependentApproachWaypoint(index, -1), index === 0],
        ["↓", () => moveIndependentApproachWaypoint(index, 1), index === objectWaypointPlanningState.approachWaypoints.length - 1],
        ["Delete", () => deleteIndependentApproachWaypoint(index), false],
      ];
      actions.forEach(([label, handler, boundaryDisabled]) => {
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = label;
        button.disabled = !editable || boundaryDisabled;
        button.addEventListener("click", handler);
        header.append(button);
      });
      card.append(header);

      const poses = document.createElement("div");
      poses.className = "digital-twin-approach-pose-columns";
      const poseInputs = {};
      for (const side of ["left", "right"]) {
        const panel = document.createElement("div");
        panel.className = "digital-twin-approach-pose-panel";
        const title = document.createElement("strong");
        title.textContent = side === "left" ? "Left Target" : "Right Target";
        panel.append(title);
        const grid = document.createElement("div");
        grid.className = "digital-twin-approach-pose-grid";
        const rotationUnit = angleUnitMetadata(angleUnitState.unit);
        const values = [
          ["X [m]", waypoint[side].translation_m[0], "translation_m", 0],
          ["Y [m]", waypoint[side].translation_m[1], "translation_m", 1],
          ["Z [m]", waypoint[side].translation_m[2], "translation_m", 2],
          [`Roll [${rotationUnit.shortLabel}]`, waypoint[side].rpy_rad[0], "rpy_rad", 0],
          [`Pitch [${rotationUnit.shortLabel}]`, waypoint[side].rpy_rad[1], "rpy_rad", 1],
          [`Yaw [${rotationUnit.shortLabel}]`, waypoint[side].rpy_rad[2], "rpy_rad", 2],
        ];
        poseInputs[side] = { translation_m: [], rpy_rad: [] };
        values.forEach(([labelText, value, group, axis]) => {
          const label = document.createElement("label");
          label.textContent = labelText;
          const input = document.createElement("input");
          input.type = "number";
          input.step = group === "translation_m" ? "0.001" : String(rotationUnit.step);
          if (group === "rpy_rad") writeOperatorRotationInput(input, value);
          else input.value = String(value);
          input.disabled = !editable;
          input.setAttribute(
            "aria-label",
            `${waypoint.identifier} ${side} ${labelText}`,
          );
          poseInputs[side][group][axis] = input;
          label.append(input);
          grid.append(label);
        });
        panel.append(grid);
        poses.append(panel);
      }
      card.append(poses);

      const commit = () => {
        const next = objectWaypointPlanningState.approachWaypoints.map((item) => ({
          identifier: item.identifier,
          left: {
            translation_m: [...item.left.translation_m],
            rpy_rad: [...item.left.rpy_rad],
          },
          right: {
            translation_m: [...item.right.translation_m],
            rpy_rad: [...item.right.rpy_rad],
          },
        }));
        next[index] = {
          identifier: idInput.value,
          left: {
            translation_m: poseInputs.left.translation_m.map((input) => input.valueAsNumber),
            rpy_rad: poseInputs.left.rpy_rad.map((input, axis) => (
              readOperatorRotationInput(input, `Left approach RPY axis ${axis}`)
            )),
          },
          right: {
            translation_m: poseInputs.right.translation_m.map((input) => input.valueAsNumber),
            rpy_rad: poseInputs.right.rpy_rad.map((input, axis) => (
              readOperatorRotationInput(input, `Right approach RPY axis ${axis}`)
            )),
          },
        };
        setIndependentApproachWaypoints(next, "INDEPENDENT APPROACH WAYPOINT EDITED");
      };
      idInput.addEventListener("change", commit);
      for (const side of ["left", "right"]) {
        for (const group of ["translation_m", "rpy_rad"]) {
          poseInputs[side][group].forEach((input) => input.addEventListener("change", commit));
        }
      }
      list.append(card);
    });
  }
  for (const id of ["digitalTwinApproachWaypointAddCurrent", "digitalTwinApproachWaypointClear"]) {
    const button = document.getElementById(id);
    if (button) button.disabled = !editable;
  }
  const clear = document.getElementById("digitalTwinApproachWaypointClear");
  if (clear) clear.disabled = !editable || objectWaypointPlanningState.approachWaypoints.length === 0;
}

function moveIndependentApproachWaypoint(index, offset) {
  const next = objectWaypointPlanningState.approachWaypoints.map((item) => ({
    identifier: item.identifier,
    left: { translation_m: [...item.left.translation_m], rpy_rad: [...item.left.rpy_rad] },
    right: { translation_m: [...item.right.translation_m], rpy_rad: [...item.right.rpy_rad] },
  }));
  const target = index + offset;
  if (target < 0 || target >= next.length) return objectWaypointPlanningStateSnapshot();
  [next[index], next[target]] = [next[target], next[index]];
  return setIndependentApproachWaypoints(next, "INDEPENDENT APPROACH WAYPOINT ORDER CHANGED");
}

function deleteIndependentApproachWaypoint(index) {
  const next = objectWaypointPlanningState.approachWaypoints.filter(
    (_waypoint, waypointIndex) => waypointIndex !== index,
  );
  return setIndependentApproachWaypoints(next, "INDEPENDENT APPROACH WAYPOINT DELETED");
}

function addCurrentIndependentApproachWaypoint(identifierValue = null) {
  if (rigidGraspState.state !== "GRASP_UNLOCKED" || !rigidGraspState.draft) {
    rigidGraspState.error = "Approach waypoints are captured before Lock Grasp";
    updateRigidGraspConfigurationUi();
    return objectWaypointPlanningStateSnapshot();
  }
  const pose = objectPreviewState.pose || INITIAL_SYNTHETIC_OBJECT_POSE;
  const transforms = computeWorldGraspFrameMatrices(pose, rigidGraspState.draft);
  const left = poseFromRigidMatrix4(transforms.worldTLeft);
  const right = poseFromRigidMatrix4(transforms.worldTRight);
  const sequence = objectWaypointPlanningState.approachWaypoints.length;
  const input = document.getElementById("digitalTwinApproachWaypointNewId");
  const identifier = identifierValue || (input && input.value.trim()) || `A${sequence}`;
  const next = [
    ...objectWaypointPlanningState.approachWaypoints,
    {
      identifier,
      left: {translation_m: [...left.translationM], rpy_rad: [...left.rpyRad]},
      right: {translation_m: [...right.translationM], rpy_rad: [...right.rpyRad]},
    },
  ];
  const state = setIndependentApproachWaypoints(
    next,
    "INDEPENDENT APPROACH WAYPOINTS CHANGED",
  );
  if (input && rigidGraspState.error === null) input.value = `A${sequence + 1}`;
  return state;
}

function clearIndependentApproachWaypoints() {
  return setIndependentApproachWaypoints(
    [],
    "INDEPENDENT APPROACH WAYPOINTS CLEARED",
  );
}

function centerPathLibrarySnapshot() {
  return {
    paths: centerPathLibraryState.paths.map((item) => ({...item})),
    selectedName: centerPathLibraryState.selectedName,
    loadedName: centerPathLibraryState.loadedName,
    dirty: centerPathLibraryState.dirty,
    status: centerPathLibraryState.status,
    error: centerPathLibraryState.error,
    requestInFlight: centerPathLibraryState.requestInFlight,
  };
}

function currentCenterPathPayload() {
  syncObjectPlanningConfigurationFromControls();
  return {
    waypoints: objectWaypointPlanningState.waypoints.map((waypoint, index) => ({
      identifier: `W${index}`,
      translation_m: [...waypoint.translation_m],
      rpy_rad: waypoint.rpy_rad
        ? [...waypoint.rpy_rad]
        : [...objectWaypointPlanningState.fixedOrientationRpyRad],
      speed_percent: waypoint.speed_percent,
      acceleration_percent: waypoint.acceleration_percent,
    })),
    fixed_orientation_rpy_rad: [...objectWaypointPlanningState.fixedOrientationRpyRad],
    orientation_source: objectWaypointPlanningState.orientationSource,
    segment_duration_s: objectWaypointPlanningState.segmentDurationS,
    samples_per_segment: objectWaypointPlanningState.samplesPerSegment,
    candidate_attempts_per_arm: objectWaypointPlanningState.candidateAttemptsPerArm,
  };
}

function writeObjectPlanningConfigurationControls() {
  const values = {
    digitalTwinObjectWaypointSegmentDuration: objectWaypointPlanningState.segmentDurationS,
    digitalTwinObjectWaypointSamplesPerSegment: objectWaypointPlanningState.samplesPerSegment,
    digitalTwinGlobalPlanCandidateAttemptsInput: objectWaypointPlanningState.candidateAttemptsPerArm,
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.value = String(value);
  });
}

function updateCenterPathLibraryUi() {
  const select = document.getElementById("digitalTwinCenterPathSelect");
  if (select) {
    const currentValue = centerPathLibraryState.selectedName || "";
    select.replaceChildren();
    const placeholder = document.createElement("option");
    placeholder.value = "";
    placeholder.textContent = centerPathLibraryState.paths.length
      ? "Select saved path…" : "No saved paths yet";
    select.append(placeholder);
    centerPathLibraryState.paths.forEach((item) => {
      const option = document.createElement("option");
      option.value = item.name;
      option.textContent = `${item.name} (${item.waypoint_count} WPs)`;
      select.append(option);
    });
    select.value = centerPathLibraryState.paths.some((item) => item.name === currentValue)
      ? currentValue : "";
  }
  const status = document.getElementById("digitalTwinCenterPathStatus");
  if (status) {
    if (centerPathLibraryState.error) status.textContent = `ERROR — ${centerPathLibraryState.error}`;
    else if (centerPathLibraryState.requestInFlight) status.textContent = centerPathLibraryState.status;
    else if (centerPathLibraryState.loadedName && centerPathLibraryState.dirty) {
      status.textContent = `MODIFIED — ${centerPathLibraryState.loadedName}`;
    } else if (centerPathLibraryState.loadedName) {
      status.textContent = `SAVED — ${centerPathLibraryState.loadedName}`;
    } else if (objectWaypointPlanningState.waypoints.length) status.textContent = "UNSAVED PATH";
    else status.textContent = "NO PATH LOADED";
  }
  const busy = centerPathLibraryState.requestInFlight || objectWaypointPlanningState.planning;
  const hasWaypoints = objectWaypointPlanningState.waypoints.length > 0;
  const hasSelection = Boolean(centerPathLibraryState.selectedName);
  const controls = {
    digitalTwinCenterPathSave: busy || !hasWaypoints,
    digitalTwinCenterPathSaveAs: busy || !hasWaypoints,
    digitalTwinCenterPathLoad: busy || !hasSelection,
    digitalTwinCenterPathRename: busy || !hasSelection,
    digitalTwinCenterPathDelete: busy || !hasSelection,
  };
  Object.entries(controls).forEach(([id, disabled]) => {
    const element = document.getElementById(id);
    if (element) element.disabled = disabled;
  });
}

async function refreshCenterPathLibrary(fetchImpl = fetch) {
  centerPathLibraryState.requestInFlight = true;
  centerPathLibraryState.status = "REFRESHING PATH LIBRARY…";
  centerPathLibraryState.error = null;
  updateCenterPathLibraryUi();
  try {
    const response = await listCenterPaths(fetchImpl);
    centerPathLibraryState.paths = Array.isArray(response.paths) ? response.paths : [];
    if (!centerPathLibraryState.paths.some((item) => item.name === centerPathLibraryState.selectedName)) {
      centerPathLibraryState.selectedName = null;
    }
    centerPathLibraryState.status = "READY";
  } catch (error) {
    centerPathLibraryState.error = error && error.message ? error.message : "Path Library refresh failed";
    centerPathLibraryState.status = "ERROR";
  } finally {
    centerPathLibraryState.requestInFlight = false;
    updateCenterPathLibraryUi();
  }
  return centerPathLibrarySnapshot();
}

async function saveCurrentCenterPath({saveAs = false, fetchImpl = fetch} = {}) {
  const nameInput = document.getElementById("digitalTwinCenterPathName");
  const typedName = nameInput ? nameInput.value.trim() : "";
  const name = saveAs
    ? typedName
    : (centerPathLibraryState.loadedName || typedName);
  if (!name) {
    centerPathLibraryState.error = "Enter a path name first";
    updateCenterPathLibraryUi();
    return centerPathLibrarySnapshot();
  }
  if (!objectWaypointPlanningState.waypoints.length) {
    centerPathLibraryState.error = "Add at least one Center waypoint before saving";
    updateCenterPathLibraryUi();
    return centerPathLibrarySnapshot();
  }
  let succeeded = false;
  centerPathLibraryState.requestInFlight = true;
  centerPathLibraryState.status = saveAs ? "SAVING NEW PATH…" : "SAVING PATH…";
  centerPathLibraryState.error = null;
  updateCenterPathLibraryUi();
  try {
    const response = await saveCenterPath(name, currentCenterPathPayload(), !saveAs, fetchImpl);
    centerPathLibraryState.loadedName = response.name;
    centerPathLibraryState.selectedName = response.name;
    centerPathLibraryState.dirty = false;
    if (nameInput) nameInput.value = response.name;
    centerPathLibraryState.status = "SAVED";
    succeeded = true;
  } catch (error) {
    centerPathLibraryState.error = error && error.message ? error.message : "Path save failed";
    centerPathLibraryState.status = "ERROR";
  } finally {
    centerPathLibraryState.requestInFlight = false;
    if (succeeded) await refreshCenterPathLibrary(fetchImpl);
    else updateCenterPathLibraryUi();
  }
  return centerPathLibrarySnapshot();
}

async function loadSelectedCenterPath(fetchImpl = fetch) {
  const name = centerPathLibraryState.selectedName;
  if (!name) return centerPathLibrarySnapshot();
  centerPathLibraryState.requestInFlight = true;
  centerPathLibraryState.status = "LOADING PATH…";
  centerPathLibraryState.error = null;
  updateCenterPathLibraryUi();
  try {
    const response = await loadCenterPath(name, fetchImpl);
    const stored = response && response.document && response.document.path;
    if (!stored || !Array.isArray(stored.waypoints)) throw new Error("Saved path payload is invalid");
    centerPathLibraryApplying = true;
    objectWaypointPlanningState.segmentDurationS = Number(stored.segment_duration_s);
    objectWaypointPlanningState.samplesPerSegment = Number(stored.samples_per_segment);
    objectWaypointPlanningState.candidateAttemptsPerArm = Number(stored.candidate_attempts_per_arm);
    setObjectPlanningWaypoints(stored.waypoints, {
      fixedOrientationRpyRad: stored.fixed_orientation_rpy_rad,
      orientationSource: stored.orientation_source,
    });
    writeObjectPlanningConfigurationControls();
    centerPathLibraryState.loadedName = response.name;
    centerPathLibraryState.selectedName = response.name;
    centerPathLibraryState.dirty = false;
    centerPathLibraryState.status = "LOADED";
    const nameInput = document.getElementById("digitalTwinCenterPathName");
    if (nameInput) nameInput.value = response.name;
  } catch (error) {
    centerPathLibraryState.error = error && error.message ? error.message : "Path load failed";
    centerPathLibraryState.status = "ERROR";
  } finally {
    centerPathLibraryApplying = false;
    centerPathLibraryState.requestInFlight = false;
    updateCenterPathLibraryUi();
  }
  return centerPathLibrarySnapshot();
}

async function renameSelectedCenterPath(fetchImpl = fetch) {
  const oldName = centerPathLibraryState.selectedName;
  const input = document.getElementById("digitalTwinCenterPathName");
  const newName = input ? input.value.trim() : "";
  if (!oldName || !newName) {
    centerPathLibraryState.error = "Select a saved path and enter its new name";
    updateCenterPathLibraryUi();
    return centerPathLibrarySnapshot();
  }
  let succeeded = false;
  centerPathLibraryState.requestInFlight = true;
  centerPathLibraryState.status = "RENAMING PATH…";
  centerPathLibraryState.error = null;
  updateCenterPathLibraryUi();
  try {
    const response = await renameCenterPath(oldName, newName, fetchImpl);
    if (centerPathLibraryState.loadedName === oldName) centerPathLibraryState.loadedName = response.name;
    centerPathLibraryState.selectedName = response.name;
    if (input) input.value = response.name;
    centerPathLibraryState.status = "RENAMED";
    succeeded = true;
  } catch (error) {
    centerPathLibraryState.error = error && error.message ? error.message : "Path rename failed";
    centerPathLibraryState.status = "ERROR";
  } finally {
    centerPathLibraryState.requestInFlight = false;
    if (succeeded) await refreshCenterPathLibrary(fetchImpl);
    else updateCenterPathLibraryUi();
  }
  return centerPathLibrarySnapshot();
}

async function deleteSelectedCenterPath(fetchImpl = fetch) {
  const name = centerPathLibraryState.selectedName;
  if (!name) return centerPathLibrarySnapshot();
  if (typeof window !== "undefined" && typeof window.confirm === "function") {
    if (!window.confirm(`Delete saved Center path "${name}"?`)) return centerPathLibrarySnapshot();
  }
  let succeeded = false;
  centerPathLibraryState.requestInFlight = true;
  centerPathLibraryState.status = "DELETING PATH…";
  centerPathLibraryState.error = null;
  updateCenterPathLibraryUi();
  try {
    await deleteCenterPath(name, fetchImpl);
    if (centerPathLibraryState.loadedName === name) {
      centerPathLibraryState.loadedName = null;
      centerPathLibraryState.dirty = objectWaypointPlanningState.waypoints.length > 0;
    }
    centerPathLibraryState.selectedName = null;
    centerPathLibraryState.status = "DELETED";
    succeeded = true;
  } catch (error) {
    centerPathLibraryState.error = error && error.message ? error.message : "Path delete failed";
    centerPathLibraryState.status = "ERROR";
  } finally {
    centerPathLibraryState.requestInFlight = false;
    if (succeeded) await refreshCenterPathLibrary(fetchImpl);
    else updateCenterPathLibraryUi();
  }
  return centerPathLibrarySnapshot();
}

function updateObjectWaypointPlanningUi() {
  const state = objectWaypointPlanningStateSnapshot();
  const values = {
    digitalTwinObjectWaypointState: state.status,
    digitalTwinObjectWaypointCount: String(state.waypoints.length),
    digitalTwinObjectWaypointOrientation: state.fixedOrientationRpyRad
      .map((value) => (
        angleUnitState.unit === "radians" ? value : value * 180 / Math.PI
      ).toFixed(angleUnitState.unit === "radians" ? 10 : 8)).join(", "),
    digitalTwinObjectWaypointOrientationSource: state.orientationSource,
    digitalTwinObjectWaypointError: state.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
  const planButton = document.getElementById("digitalTwinPlanObjectGlobal");
  if (planButton) {
    const authorityReady = phase3PlanningAuthorityIdentity() !== null;
    const prerequisitesReady = state.status !== "INVALID"
      && authorityReady
      && state.waypoints.length >= 2
      && Number.isFinite(state.segmentDurationS)
      && state.segmentDurationS > 0
      && Number.isInteger(state.samplesPerSegment)
      && state.samplesPerSegment >= 2;
    planButton.disabled = state.planning || !prerequisitesReady;
    planButton.textContent = state.planning
      ? "Planning…"
      : "Generate Trajectory";
  }
  const locked = rigidGraspState.state === "GRASP_LOCKED";
  const centerMove = document.getElementById("digitalTwinCenterWaypointMove");
  const centerRotate = document.getElementById("digitalTwinCenterWaypointRotate");
  const centerGizmo = document.getElementById("digitalTwinCenterWaypointGizmoToggle");
  if (centerMove) {
    centerMove.disabled = !locked;
    centerMove.setAttribute("aria-pressed", String(
      locked && rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER
        && rigidGraspState.gizmoMode === "translate"
    ));
  }
  if (centerRotate) {
    centerRotate.disabled = !locked;
    centerRotate.setAttribute("aria-pressed", String(
      locked && rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER
        && rigidGraspState.gizmoMode === "rotate"
    ));
  }
  if (centerGizmo) {
    centerGizmo.disabled = !locked;
    centerGizmo.setAttribute("aria-pressed", String(
      locked && rigidGraspState.selectedFrame === GRASP_SELECTED_FRAMES.CENTER
        && objectDragState.enabled
    ));
    centerGizmo.textContent = objectDragState.enabled ? "Disable Center Gizmo" : "Enable Center Gizmo";
  }
  const nextIdentifierInput = document.getElementById("digitalTwinObjectWaypointNewId");
  if (nextIdentifierInput) {
    nextIdentifierInput.value = `W${state.waypoints.length}`;
    nextIdentifierInput.readOnly = true;
    nextIdentifierInput.title = "Waypoint identifiers are numbered automatically";
  }
  for (const id of [
    "digitalTwinObjectWaypointAdd",
    "digitalTwinObjectWaypointAddCurrent",
    "digitalTwinObjectWaypointDemo",
  ]) {
    const button = document.getElementById(id);
    if (button) button.disabled = !locked;
  }
  updateOperatorPlanStatus();
  updateApproachWaypointUi();
  syncObjectWaypointVisualization();
  renderObjectWaypointRows();
  updateCenterPathLibraryUi();
}

function setObjectPlanningWaypoints(waypoints, {
  fixedOrientationRpyRad = objectWaypointPlanningState.fixedOrientationRpyRad,
  orientationSource = objectWaypointPlanningState.orientationSource,
} = {}) {
  try {
    objectWaypointPlanningState.waypoints = renumberObjectPlanningWaypoints(waypoints);
    objectWaypointPlanningState.fixedOrientationRpyRad = [...fixedOrientationRpyRad];
    objectWaypointPlanningState.orientationSource = orientationSource;
    objectWaypointPlanningState.status = waypoints.length > 0 ? "READY" : "EMPTY";
    objectWaypointPlanningState.error = null;
    if (!centerPathLibraryApplying) {
      centerPathLibraryState.dirty = true;
      centerPathLibraryState.error = null;
    }
  } catch (error) {
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = error.message;
  }
  invalidatePlanningForGraspChange("OBJECT WAYPOINTS CHANGED");
  updateObjectWaypointPlanningUi();
  return objectWaypointPlanningStateSnapshot();
}

function addObjectPlanningWaypointToState(waypoint) {
  try {
    return setObjectPlanningWaypoints([
      ...objectWaypointPlanningState.waypoints,
      waypoint,
    ]);
  } catch (error) {
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = error.message;
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
}

function addCurrentObjectPlanningWaypoint(identifierValue = null) {
  if (rigidGraspState.state !== "GRASP_LOCKED") {
    objectWaypointPlanningState.error = "Lock the rigid grasp before saving a Center waypoint";
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
  const pose = objectPreviewState.pose || INITIAL_SYNTHETIC_OBJECT_POSE;
  let sequence = objectWaypointPlanningState.waypoints.length;
  const names = new Set(objectWaypointPlanningState.waypoints.map(
    (waypoint) => waypoint.identifier.toLocaleLowerCase(),
  ));
  while (names.has(`w${sequence}`)) sequence += 1;
  const nextIdentifier = identifierValue || `W${sequence}`;
  objectWaypointPlanningState.fixedOrientationRpyRad = [...pose.rpyRad];
  objectWaypointPlanningState.orientationSource = "CURRENT CENTER PREVIEW ORIENTATION";
  return addObjectPlanningWaypointToState({
    identifier: nextIdentifier,
    translation_m: [...pose.translationM],
    rpy_rad: [...pose.rpyRad],
  });
}

function deleteObjectPlanningWaypointFromState(index) {
  try {
    return setObjectPlanningWaypoints(deleteObjectPlanningWaypoint(
      objectWaypointPlanningState.waypoints,
      index,
    ));
  } catch (error) {
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = error.message;
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
}

function moveObjectPlanningWaypointInState(index, offset) {
  try {
    return setObjectPlanningWaypoints(moveObjectPlanningWaypoint(
      objectWaypointPlanningState.waypoints,
      index,
      offset,
    ));
  } catch (error) {
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = error.message;
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
}

function duplicateObjectPlanningWaypointInState(index) {
  try {
    return setObjectPlanningWaypoints(duplicateObjectPlanningWaypoint(
      objectWaypointPlanningState.waypoints,
      index,
    ));
  } catch (error) {
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = error.message;
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
}

function reorderObjectPlanningWaypointInState(index, insertionIndex) {
  try {
    return setObjectPlanningWaypoints(reorderObjectPlanningWaypoint(
      objectWaypointPlanningState.waypoints,
      index,
      insertionIndex,
    ));
  } catch (error) {
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = error.message;
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
}

function moveCenterPreviewToObjectWaypoint(index) {
  if (rigidGraspState.state !== "GRASP_LOCKED") {
    objectWaypointPlanningState.error = "Lock the rigid grasp before moving Center to a waypoint";
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
  const waypoint = objectWaypointPlanningState.waypoints[index];
  if (!waypoint) {
    objectWaypointPlanningState.error = "Waypoint index is out of range";
    updateObjectWaypointPlanningUi();
    return objectWaypointPlanningStateSnapshot();
  }
  pauseObjectTrajectoryForManualPreview();
  setRigidGraspSelectedFrame(GRASP_SELECTED_FRAMES.CENTER);
  applyObjectPreviewPose({
    translationM: [...waypoint.translation_m],
    rpyRad: waypoint.rpy_rad
      ? [...waypoint.rpy_rad]
      : [...objectWaypointPlanningState.fixedOrientationRpyRad],
  });
  objectWaypointPlanningState.error = null;
  updateObjectWaypointPlanningUi();
  return objectWaypointPlanningStateSnapshot();
}

function clearObjectPlanningWaypoints() {
  return setObjectPlanningWaypoints([]);
}

function loadObjectPlanningDemoFixture() {
  const fixtureWaypoints = PHASE3_DEMO_OBJECT_WAYPOINTS.map((waypoint) => ({
    identifier: waypoint.identifier,
    translation_m: [...waypoint.translation_m],
    rpy_rad: [...PHASE3_DEMO_FIXED_ORIENTATION_RPY_RAD],
  }));
  applyObjectPreviewPose({
    translationM: [...fixtureWaypoints[0].translation_m],
    rpyRad: [...PHASE3_DEMO_FIXED_ORIENTATION_RPY_RAD],
  });
  return setObjectPlanningWaypoints(fixtureWaypoints, {
    fixedOrientationRpyRad: [...PHASE3_DEMO_FIXED_ORIENTATION_RPY_RAD],
    orientationSource: "PHASE 1F.2 SYNTHETIC DEMO FIXTURE",
  });
}

function renderObjectWaypointRows() {
  const list = document.getElementById("digitalTwinObjectWaypointList");
  if (!list) return;
  hideObjectWaypointHoverPreview();
  const clearDropIndicators = () => {
    list.querySelectorAll(".digital-twin-object-waypoint-row").forEach((candidate) => {
      candidate.classList.remove("is-dragging", "drop-before", "drop-after");
    });
  };
  list.replaceChildren();
  objectWaypointPlanningState.waypoints.forEach((waypoint, index) => {
    const isFinalWaypoint = index === objectWaypointPlanningState.waypoints.length - 1;
    const row = document.createElement("div");
    row.className = "digital-twin-object-waypoint-row";
    row.dataset.waypointIndex = String(index);
    const makeInput = (value, label, step = null) => {
      const input = document.createElement("input");
      input.value = String(value);
      input.setAttribute("aria-label", label);
      if (step !== null) {
        input.type = "number";
        input.step = step;
      }
      return input;
    };

    const idCell = document.createElement("div");
    idCell.className = "digital-twin-waypoint-id-cell";
    const dragHandle = document.createElement("span");
    dragHandle.className = "digital-twin-waypoint-drag-handle";
    dragHandle.textContent = "⠿";
    dragHandle.draggable = true;
    dragHandle.tabIndex = 0;
    dragHandle.title = `Drag ${waypoint.identifier} to reorder`;
    dragHandle.setAttribute("role", "button");
    dragHandle.setAttribute("aria-label", `Drag ${waypoint.identifier} to reorder`);
    const idInput = makeInput(waypoint.identifier, `Waypoint ${index + 1} identifier`);
    idInput.readOnly = true;
    idInput.title = "Waypoint identifiers are numbered automatically";
    idCell.append(dragHandle, idInput);

    const coordinateInputs = waypoint.translation_m.map((value, axis) => (
      makeInput(value, `Waypoint ${index + 1} ${["X", "Y", "Z"][axis]} meters`, "0.01")
    ));
    const waypointRpy = waypoint.rpy_rad || objectWaypointPlanningState.fixedOrientationRpyRad;
    const orientationInputs = waypointRpy.map((value, axis) => (
      makeInput(
        rotationRadiansToDisplay(value, angleUnitState.unit),
        `Waypoint ${index + 1} ${["Roll", "Pitch", "Yaw"][axis]} ${angleUnitMetadata(angleUnitState.unit).spokenLabel}`,
        String(angleUnitMetadata(angleUnitState.unit).step),
      )
    ));
    orientationInputs.forEach(configureOperatorRotationInput);

    const makeProfileControl = (profileName, value) => {
      const cell = document.createElement("div");
      cell.className = "digital-twin-waypoint-profile-control";
      const range = document.createElement("input");
      range.type = "range";
      range.min = "0.1";
      range.max = "100";
      range.step = "0.1";
      range.value = String(value);
      range.setAttribute(
        "aria-label",
        `${waypoint.identifier} outgoing ${profileName} percent slider`,
      );
      const numeric = document.createElement("input");
      numeric.type = "number";
      numeric.min = "0.1";
      numeric.max = "100";
      numeric.step = "0.1";
      numeric.value = String(value);
      numeric.setAttribute(
        "aria-label",
        `${waypoint.identifier} outgoing ${profileName} percent numeric value`,
      );
      const percent = document.createElement("span");
      percent.textContent = "%";
      if (isFinalWaypoint) {
        range.disabled = true;
        numeric.disabled = true;
        cell.classList.add("is-final-waypoint-profile");
        cell.title = `${waypoint.identifier} is final and has no outgoing segment`;
      }
      range.addEventListener("input", () => {
        numeric.value = range.value;
      });
      numeric.addEventListener("input", () => {
        const candidate = numeric.valueAsNumber;
        if (Number.isFinite(candidate) && candidate > 0 && candidate <= 100) {
          range.value = numeric.value;
        }
      });
      cell.append(range, numeric, percent);
      if (isFinalWaypoint) {
        const note = document.createElement("small");
        note.textContent = "Final — no outgoing segment";
        cell.append(note);
      }
      return {cell, range, numeric};
    };
    const speedControl = makeProfileControl("Speed", waypoint.speed_percent);
    const accelerationControl = makeProfileControl(
      "Acceleration",
      waypoint.acceleration_percent,
    );
    const commit = () => {
      const next = objectWaypointPlanningState.waypoints.map((item) => ({
        identifier: item.identifier,
        translation_m: [...item.translation_m],
        rpy_rad: item.rpy_rad ? [...item.rpy_rad] : null,
        speed_percent: item.speed_percent,
        acceleration_percent: item.acceleration_percent,
      }));
      next[index] = {
        identifier: waypoint.identifier,
        translation_m: coordinateInputs.map((input) => input.valueAsNumber),
        rpy_rad: orientationInputs.map((input, axis) => readOperatorRotationInput(
          input,
          `${waypoint.identifier} ${["Roll", "Pitch", "Yaw"][axis]}`,
        )),
        speed_percent: speedControl.numeric.valueAsNumber,
        acceleration_percent: accelerationControl.numeric.valueAsNumber,
      };
      setObjectPlanningWaypoints(next);
    };
    coordinateInputs.forEach((input) => input.addEventListener("change", commit));
    orientationInputs.forEach((input) => input.addEventListener("change", commit));
    for (const control of [speedControl, accelerationControl]) {
      control.range.addEventListener("change", commit);
      control.numeric.addEventListener("change", commit);
    }

    row.addEventListener("pointerenter", () => {
      showObjectWaypointHoverPreview(index);
    });
    row.addEventListener("pointerleave", () => {
      hideObjectWaypointHoverPreview();
    });

    dragHandle.addEventListener("dragstart", (event) => {
      hideObjectWaypointHoverPreview();
      clearDropIndicators();
      row.classList.add("is-dragging");
      if (event.dataTransfer) {
        event.dataTransfer.effectAllowed = "move";
        event.dataTransfer.setData("text/plain", String(index));
      }
    });
    dragHandle.addEventListener("dragend", clearDropIndicators);
    row.addEventListener("dragover", (event) => {
      event.preventDefault();
      clearDropIndicators();
      const rect = row.getBoundingClientRect();
      const after = event.clientY > rect.top + (rect.height / 2);
      row.classList.add(after ? "drop-after" : "drop-before");
      if (event.dataTransfer) event.dataTransfer.dropEffect = "move";
    });
    row.addEventListener("drop", (event) => {
      event.preventDefault();
      const rawSource = event.dataTransfer
        ? event.dataTransfer.getData("text/plain") : "";
      const sourceIndex = Number(rawSource);
      const rect = row.getBoundingClientRect();
      const after = event.clientY > rect.top + (rect.height / 2);
      const insertionIndex = index + (after ? 1 : 0);
      clearDropIndicators();
      if (Number.isInteger(sourceIndex)) {
        reorderObjectPlanningWaypointInState(sourceIndex, insertionIndex);
      }
    });

    row.append(
      idCell,
      ...coordinateInputs,
      ...orientationInputs,
      speedControl.cell,
      accelerationControl.cell,
    );
    const actions = [
      ["Move Center Here", () => moveCenterPreviewToObjectWaypoint(index)],
      ["Duplicate", () => duplicateObjectPlanningWaypointInState(index)],
      ["Delete", () => deleteObjectPlanningWaypointFromState(index)],
    ];
    actions.forEach(([label, handler]) => {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = label;
      button.addEventListener("click", handler);
      row.append(button);
    });
    list.append(row);
  });
}

function objectWaypointFromAddControls() {
  const read = (id) => {
    const element = document.getElementById(id);
    if (!element) throw new Error(`Missing Object Waypoint control: ${id}`);
    return element;
  };
  return {
    identifier: read("digitalTwinObjectWaypointNewId").value,
    translation_m: [
      read("digitalTwinObjectWaypointNewX").valueAsNumber,
      read("digitalTwinObjectWaypointNewY").valueAsNumber,
      read("digitalTwinObjectWaypointNewZ").valueAsNumber,
    ],
    rpy_rad: [
      readOperatorRotationInput(read("digitalTwinObjectWaypointNewRoll"), "New waypoint Roll"),
      readOperatorRotationInput(read("digitalTwinObjectWaypointNewPitch"), "New waypoint Pitch"),
      readOperatorRotationInput(read("digitalTwinObjectWaypointNewYaw"), "New waypoint Yaw"),
    ],
    speed_percent: 100.0,
    acceleration_percent: 100.0,
  };
}

function syncObjectPlanningConfigurationFromControls() {
  const duration = document.getElementById("digitalTwinObjectWaypointSegmentDuration");
  const samples = document.getElementById("digitalTwinObjectWaypointSamplesPerSegment");
  const candidateAttempts = document.getElementById(
    "digitalTwinGlobalPlanCandidateAttemptsInput",
  );
  if (duration) objectWaypointPlanningState.segmentDurationS = duration.valueAsNumber;
  if (samples) objectWaypointPlanningState.samplesPerSegment = samples.valueAsNumber;
  if (candidateAttempts) {
    objectWaypointPlanningState.candidateAttemptsPerArm = (
      candidateAttempts.valueAsNumber
    );
  }
}

function loadObjectGlobalPlanPreview(response) {
  const plan = normalizeObjectGlobalPlanResponse(response);
  invalidateAllPhase4Validation("NEW GLOBAL PLAN LOADED");
  if (!plan.ok) {
    objectGlobalPlanState.status = plan.planner_status;
    objectGlobalPlanState.plan = plan;
    objectGlobalPlanState.error = plan.failure && plan.failure.message
      ? plan.failure.message
      : "Global planning failed";
    // Never leave an older (possibly discontinuous) Planned trajectory
    // playable after the replacement plan has failed its continuity gate.
    clearPlannedTrajectory();
    updateObjectGlobalPlanUi();
    return globalPlanStateSnapshot();
  }
  cancelObjectTrajectoryAnimation();
  objectTrajectoryPreviewState.playing = false;
  if (objectTrajectoryPreviewState.trajectory) {
    objectTrajectoryPreviewState.status = "INTEGRATED_PLAN_OVERRIDE";
    updateObjectTrajectoryPreviewUi();
  }
  objectGlobalPlanState.status = "READY";
  objectGlobalPlanState.plan = plan;
  objectGlobalPlanState.error = null;
  integratedPlanPreviewState.active = true;
  integratedPlanPreviewState.owner = "INTEGRATED GLOBAL PLAN CLOCK";
  integratedPlanPreviewState.plan = plan;
  integratedPlanPreviewState.error = null;
  loadPlannedTrajectory(
    globalPlanToPlannedTrajectory(plan),
    PHASE3_WEB_PLANNER_SOURCE,
    { preserveIntegratedPlan: true },
  );
  updateObjectGlobalPlanUi();
  updatePhase4UnifiedValidationUi();
  return globalPlanStateSnapshot();
}

async function planObjectGlobal(fetchImpl = fetch) {
  // Refresh read-only authority snapshots before the initial planning gate.
  try {
    await loadWorldCalibrationState(fetchImpl);
    applyBackendRigidGraspState(await requestGraspConfiguration(fetchImpl));
  } catch (error) {
    const message = error && error.message
      ? `Planning authority refresh failed: ${error.message}`
      : "Planning authority refresh failed";
    objectWaypointPlanningState.status = "BLOCKED";
    objectWaypointPlanningState.error = message;
    objectGlobalPlanState.status = "BLOCKED";
    objectGlobalPlanState.plan = null;
    objectGlobalPlanState.error = message;
    clearPlannedTrajectory();
    invalidateAllPhase4Validation(message);
    return globalPlanStateSnapshot();
  }
  let authorityAtRequest = phase3PlanningAuthorityIdentity();
  if (!authorityAtRequest) {
    const message = (
      "Phase-3 requires GRASP_LOCKED plus matching backend/Web model calibration revisions"
    );
    objectWaypointPlanningState.status = "BLOCKED";
    objectWaypointPlanningState.error = message;
    objectGlobalPlanState.status = "BLOCKED";
    objectGlobalPlanState.plan = null;
    objectGlobalPlanState.error = message;
    clearPlannedTrajectory();
    invalidateAllPhase4Validation(message);
    return globalPlanStateSnapshot();
  }
  const requestGeneration = objectGlobalPlanState.requestGeneration + 1;
  objectGlobalPlanState.requestGeneration = requestGeneration;
  syncObjectPlanningConfigurationFromControls();
  const currentPose = objectPreviewState.pose || INITIAL_SYNTHETIC_OBJECT_POSE;
  objectWaypointPlanningState.fixedOrientationRpyRad = [...currentPose.rpyRad];
  objectWaypointPlanningState.orientationSource = "CURRENT CENTER PREVIEW ORIENTATION";
  try {
    objectWaypointPlanningState.planning = true;
    objectWaypointPlanningState.status = "CHECKING_AUTHORITY";
    objectWaypointPlanningState.error = null;
    objectGlobalPlanState.status = "CHECKING_AUTHORITY";
    objectGlobalPlanState.error = null;
    updateObjectWaypointPlanningUi();
    updateObjectGlobalPlanUi();

    // The rigid-grasp lock is intentionally backend-owned and resets safely
    // when the backend restarts. Refresh it before every plan so a browser that
    // stayed open across that restart cannot submit a stale lock identity.
    applyBackendRigidGraspState(await requestGraspConfiguration(fetchImpl));
    authorityAtRequest = phase3PlanningAuthorityIdentity();
    if (!authorityAtRequest) {
      const message = (
        "Backend planning authority changed or the rigid grasp is unlocked. "
        + "Lock the grasp again before Generate Trajectory."
      );
      objectWaypointPlanningState.status = "BLOCKED";
      objectWaypointPlanningState.error = message;
      objectGlobalPlanState.status = "BLOCKED";
      objectGlobalPlanState.plan = null;
      objectGlobalPlanState.error = message;
      clearPlannedTrajectory();
      invalidateAllPhase4Validation(message);
      updateObjectGlobalPlanUi();
      return globalPlanStateSnapshot();
    }

    const request = buildObjectGlobalPlanRequest({
      name: "Object_Global_Plan",
      waypoints: objectWaypointPlanningState.waypoints,
      approachWaypoints: objectWaypointPlanningState.approachWaypoints,
      fixedOrientationRpyRad: objectWaypointPlanningState.fixedOrientationRpyRad,
      segmentDurationS: objectWaypointPlanningState.segmentDurationS,
      samplesPerSegment: objectWaypointPlanningState.samplesPerSegment,
      candidateAttemptsPerArm: objectWaypointPlanningState.candidateAttemptsPerArm,
      initialJointStateRad: nextPlanningStartOverride
        ? nextPlanningStartOverride.initialJointStateRad : null,
      planningStartStateSource: nextPlanningStartOverride
        ? nextPlanningStartOverride.planningStartStateSource : null,
      expectedGraspContentRevision: authorityAtRequest.graspContentRevision,
      expectedLockGeneration: authorityAtRequest.lockGeneration,
      expectedLockRevision: authorityAtRequest.lockRevision,
      expectedCalibrationRevision: authorityAtRequest.calibrationRevision,
      expectedModelCalibrationRevision: authorityAtRequest.modelCalibrationRevision,
    });
    // The explicit Replan From Current capture applies to exactly this global
    // planner request. Every later request falls back to code config unless the
    // operator explicitly captures fresh Actual feedback again.
    nextPlanningStartOverride = null;
    objectWaypointPlanningState.status = "PLANNING";
    objectGlobalPlanState.status = "PLANNING";
    updateObjectWaypointPlanningUi();
    updateObjectGlobalPlanUi();
    const response = await requestObjectGlobalPlan(request, fetchImpl);
    const authorityNow = phase3PlanningAuthorityIdentity();
    const responseAuthorityMatches = response.ok !== true || (
      response.grasp.content_revision === authorityAtRequest.graspContentRevision
      && response.grasp.lock_generation === authorityAtRequest.lockGeneration
      && response.grasp.lock_revision === authorityAtRequest.lockRevision
      && response.calibration.revision === authorityAtRequest.calibrationRevision
      && response.calibration.model_revision
        === authorityAtRequest.modelCalibrationRevision
    );
    if (
      requestGeneration !== objectGlobalPlanState.requestGeneration
      || !samePhase3PlanningAuthority(authorityAtRequest, authorityNow)
      || !responseAuthorityMatches
    ) {
      const staleMessage = (
        "STALE PLANNING RESPONSE — grasp or calibration authority changed in flight"
      );
      objectWaypointPlanningState.status = "STALE";
      objectWaypointPlanningState.error = staleMessage;
      objectGlobalPlanState.status = "STALE";
      objectGlobalPlanState.plan = null;
      objectGlobalPlanState.error = staleMessage;
      clearPlannedTrajectory();
      invalidateAllPhase4Validation(staleMessage);
      return globalPlanStateSnapshot();
    }
    objectWaypointPlanningState.status = response.ok ? "READY" : "FAILED";
    return loadObjectGlobalPlanPreview(response);
  } catch (error) {
    const message = error && error.message ? error.message : "Global planning failed";
    objectWaypointPlanningState.status = "INVALID";
    objectWaypointPlanningState.error = message;
    objectGlobalPlanState.status = "FAILED";
    objectGlobalPlanState.error = message;
    deactivateIntegratedPlanPreview("NONE");
    updateObjectGlobalPlanUi();
    return globalPlanStateSnapshot();
  } finally {
    objectWaypointPlanningState.planning = false;
    updateObjectWaypointPlanningUi();
  }
}

function bindObjectWaypointPlanningControls() {
  const centerMove = document.getElementById("digitalTwinCenterWaypointMove");
  if (centerMove) centerMove.addEventListener("click", () => {
    setRigidGraspSelectedFrame(GRASP_SELECTED_FRAMES.CENTER);
    setRigidGraspGizmoMode("translate");
    setObjectDragEnabled(true);
  });
  const centerRotate = document.getElementById("digitalTwinCenterWaypointRotate");
  if (centerRotate) centerRotate.addEventListener("click", () => {
    setRigidGraspSelectedFrame(GRASP_SELECTED_FRAMES.CENTER);
    setRigidGraspGizmoMode("rotate");
    setObjectDragEnabled(true);
  });
  const centerGizmo = document.getElementById("digitalTwinCenterWaypointGizmoToggle");
  if (centerGizmo) centerGizmo.addEventListener("click", () => {
    setRigidGraspSelectedFrame(GRASP_SELECTED_FRAMES.CENTER);
    setObjectDragEnabled(!objectDragState.enabled);
  });
  const pathSelect = document.getElementById("digitalTwinCenterPathSelect");
  if (pathSelect) pathSelect.addEventListener("change", (event) => {
    const selected = String(event.target.value || "").trim();
    centerPathLibraryState.selectedName = selected || null;
    centerPathLibraryState.error = null;
    const nameInput = document.getElementById("digitalTwinCenterPathName");
    if (nameInput && selected) nameInput.value = selected;
    updateCenterPathLibraryUi();
  });
  const bindings = {
    digitalTwinCenterPathSave: () => saveCurrentCenterPath({saveAs: false}),
    digitalTwinCenterPathSaveAs: () => saveCurrentCenterPath({saveAs: true}),
    digitalTwinCenterPathLoad: () => loadSelectedCenterPath(),
    digitalTwinCenterPathRename: () => renameSelectedCenterPath(),
    digitalTwinCenterPathDelete: () => deleteSelectedCenterPath(),
    digitalTwinObjectWaypointAdd: () => addObjectPlanningWaypointToState(
      objectWaypointFromAddControls(),
    ),
    digitalTwinApproachWaypointAddCurrent: () => addCurrentIndependentApproachWaypoint(),
    digitalTwinApproachWaypointClear: () => clearIndependentApproachWaypoints(),
    digitalTwinObjectWaypointAddCurrent: () => addCurrentObjectPlanningWaypoint(),
    digitalTwinObjectWaypointDemo: loadObjectPlanningDemoFixture,
    digitalTwinObjectWaypointClear: clearObjectPlanningWaypoints,
    digitalTwinPlanObjectGlobal: () => planObjectGlobal(),
    digitalTwinGlobalPlanPlay: playPlannedTrajectory,
    digitalTwinGlobalPlanPause: pausePlannedTrajectory,
    digitalTwinGlobalPlanReset: stopPlannedTrajectory,
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });
  const scrubber = document.getElementById("digitalTwinGlobalPlanScrubber");
  if (scrubber) scrubber.addEventListener("input", (event) => {
    setTrajectoryTime(Number(event.target.value));
  });
  const rate = document.getElementById("digitalTwinGlobalPlanPlaybackRate");
  if (rate) rate.addEventListener("change", (event) => {
    setTrajectoryPlaybackRate(Number(event.target.value));
  });
  for (const id of [
    "digitalTwinObjectWaypointSegmentDuration",
    "digitalTwinObjectWaypointSamplesPerSegment",
    "digitalTwinGlobalPlanCandidateAttemptsInput",
  ]) {
    const element = document.getElementById(id);
    if (element) element.addEventListener("change", () => {
      centerPathLibraryState.dirty = true;
      centerPathLibraryState.error = null;
      invalidatePlanningForGraspChange("GLOBAL PLANNING CONFIGURATION CHANGED");
      updateCenterPathLibraryUi();
    });
  }
  refreshCenterPathLibrary();
}

function setIntegratedPlanTime(timeSeconds) {
  if (integratedPlanPreviewState.active) setTrajectoryTime(timeSeconds);
  return integratedPlanStateSnapshot();
}

function executionTargetPoseMatchesTrajectory(target, sampled) {
  const tolerance = 1e-9;
  return ["left", "right"].every((side) => (
    Array.isArray(target.pose[side])
    && target.pose[side].length === 6
    && target.pose[side].every((joint, index) => (
      typeof joint === "number"
      && Number.isFinite(joint)
      && Math.abs(joint - sampled[side][index]) <= tolerance
    ))
  ));
}

function setExecutionWaypointTarget(target) {
  // Visual-only Phase-5 target mode. This touches only the translucent Planned
  // model and its existing preview clock; MAIN remains owned by Live Joint
  // Feedback/offline configuration.
  pausePlannedTrajectory();
  if (!trajectoryPreviewState.trajectory) {
    return { applied: false, reason: "FROZEN PLANNED TRAJECTORY UNAVAILABLE" };
  }
  if (!target || typeof target !== "object" || !target.pose
      || typeof target.timeS !== "number" || !Number.isFinite(target.timeS)) {
    return { applied: false, reason: "EXECUTION WAYPOINT TARGET IS INVALID" };
  }
  const durationS = trajectoryDurationSeconds(trajectoryPreviewState.trajectory);
  if (target.timeS < 0 || target.timeS > durationS) {
    return { applied: false, reason: "EXECUTION WAYPOINT TARGET TIME IS OUT OF RANGE" };
  }
  const sampled = sampleTrajectoryAtTime(
    trajectoryPreviewState.trajectory,
    target.timeS,
  );
  if (!executionTargetPoseMatchesTrajectory(target, sampled)) {
    return {
      applied: false,
      reason: "EXECUTION WAYPOINT TARGET DOES NOT MATCH FROZEN TRAJECTORY",
    };
  }

  const planned = setPlannedJointValues(target.pose, trajectoryPreviewState.source);
  if (!planned || planned.visible !== true) {
    return { applied: false, reason: "PLANNED GHOST MODEL COULD NOT SHOW TARGET" };
  }
  trajectoryPreviewState.currentTimeS = sampled.time_from_start_s;
  trajectoryPreviewState.currentSegmentIndex = sampled.segmentIndex;
  trajectoryPreviewState.currentPointIndex = sampled.segmentIndex;
  if (sampled.alpha === 1 && sampled.time_from_start_s === durationS) {
    trajectoryPreviewState.currentPointIndex = trajectoryPreviewState.trajectory.points.length - 1;
  }
  trajectoryPreviewState.currentAlpha = sampled.alpha;
  trajectoryPreviewState.status = "EXECUTION_WAYPOINT_TARGET";
  trajectoryPreviewState.validationError = null;
  if (integratedPlanPreviewState.active) {
    applyIntegratedPlanTime(target.timeS);
    integratedPlanPreviewState.owner = "PHASE 5 NEXT RIGID WAYPOINT TARGET";
  }
  updateTrajectoryPreviewUi();
  updateObjectGlobalPlanUi();
  return {
    applied: true,
    reason: null,
    waypointIndex: target.waypointIndex,
    identifier: target.identifier,
    timeS: target.timeS,
  };
}

function hideExecutionWaypointTarget() {
  // Preserve the loaded frozen trajectory/source identity for later manual
  // review; only the Planned model's visibility changes.
  pausePlannedTrajectory();
  return hidePlannedModel();
}

function playIntegratedPlan() {
  if (integratedPlanPreviewState.active) playPlannedTrajectory();
  return integratedPlanStateSnapshot();
}

function pauseIntegratedPlan() {
  if (integratedPlanPreviewState.active) pausePlannedTrajectory();
  return integratedPlanStateSnapshot();
}

function resetIntegratedPlan() {
  if (integratedPlanPreviewState.active) stopPlannedTrajectory();
  return integratedPlanStateSnapshot();
}

function objectTrajectoryStateSnapshot() {
  const trajectory = objectTrajectoryPreviewState.trajectory;
  return {
    status: objectTrajectoryPreviewState.status,
    trajectoryName: trajectory ? trajectory.name : "NONE",
    sampleCount: trajectory ? trajectory.sampleCount : 0,
    currentSampleIndex: objectTrajectoryPreviewState.currentSampleIndex,
    currentTimeS: objectTrajectoryPreviewState.currentTimeS,
    durationS: trajectory ? trajectory.durationS : 0,
    alpha: trajectory
      ? trajectory.samples[objectTrajectoryPreviewState.currentSampleIndex].alpha
      : 0,
    playing: objectTrajectoryPreviewState.playing,
    playbackRate: objectTrajectoryPreviewState.playbackRate,
    error: objectTrajectoryPreviewState.error,
  };
}

function getObjectTrajectoryPreviewState() {
  return objectTrajectoryStateSnapshot();
}

function updateObjectTrajectoryPreviewUi() {
  const state = objectTrajectoryStateSnapshot();
  const sampleLabel = state.sampleCount > 0
    ? `${state.currentSampleIndex + 1}/${state.sampleCount}`
    : "NONE";
  const values = {
    digitalTwinObjectTrajectoryState: state.status,
    digitalTwinObjectTrajectoryName: state.trajectoryName,
    digitalTwinObjectTrajectorySampleCount: String(state.sampleCount),
    digitalTwinObjectTrajectoryCurrentSample: sampleLabel,
    digitalTwinObjectTrajectoryTime: `${state.currentTimeS.toFixed(2)} s`,
    digitalTwinObjectTrajectoryDuration: `${state.durationS.toFixed(2)} s`,
    digitalTwinObjectTrajectoryAlpha: state.alpha.toFixed(2),
    digitalTwinObjectTrajectoryRate: `${state.playbackRate.toFixed(2)}x`,
    digitalTwinObjectTrajectoryError: state.error || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });

  const scrubber = document.getElementById("digitalTwinObjectTrajectoryScrubber");
  if (scrubber) {
    scrubber.max = String(state.durationS);
    scrubber.value = String(state.currentTimeS);
    scrubber.disabled = state.sampleCount === 0;
    scrubber.setAttribute("aria-valuenow", state.currentTimeS.toFixed(2));
  }
  const scrubberTime = document.getElementById(
    "digitalTwinObjectTrajectoryScrubberTime",
  );
  if (scrubberTime) scrubberTime.textContent = `${state.currentTimeS.toFixed(2)} s`;
  const rateSelector = document.getElementById(
    "digitalTwinObjectTrajectoryPlaybackRate",
  );
  if (rateSelector && rateSelector.value !== String(state.playbackRate)) {
    rateSelector.value = String(state.playbackRate);
  }
}

function cancelObjectTrajectoryAnimation() {
  if (objectTrajectoryPreviewState.animationFrameId !== null) {
    cancelAnimationFrame(objectTrajectoryPreviewState.animationFrameId);
    objectTrajectoryPreviewState.animationFrameId = null;
  }
  objectTrajectoryPreviewState.previousFrameTimeMs = null;
}

function applyObjectTrajectorySample(
  sampleIndex,
  nextStatus = null,
  currentTimeS = null,
) {
  const trajectory = objectTrajectoryPreviewState.trajectory;
  if (!trajectory) return false;
  const sample = objectTrajectorySampleAtIndex(trajectory, sampleIndex);
  applyObjectPreviewPose(sample.objectPose);
  objectTrajectoryPreviewState.currentSampleIndex = sample.index;
  objectTrajectoryPreviewState.currentTimeS = currentTimeS === null
    ? sample.timeFromStartS
    : currentTimeS;
  objectTrajectoryPreviewState.error = null;
  if (nextStatus) objectTrajectoryPreviewState.status = nextStatus;
  updateObjectTrajectoryPreviewUi();
  return true;
}

function loadSyntheticObjectTrajectory() {
  if (integratedPlanPreviewState.active) {
    pausePlannedTrajectory();
    deactivateIntegratedPlanPreview("STANDALONE OBJECT TRAJECTORY");
  }
  cancelObjectTrajectoryAnimation();
  objectTrajectoryPreviewState.trajectory = (
    SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY
  );
  objectTrajectoryPreviewState.currentTimeS = 0;
  objectTrajectoryPreviewState.currentSampleIndex = 0;
  objectTrajectoryPreviewState.playing = false;
  objectTrajectoryPreviewState.error = null;
  applyObjectTrajectorySample(0, "READY");
  return objectTrajectoryStateSnapshot();
}

function setObjectTrajectorySampleIndex(index) {
  if (!objectTrajectoryPreviewState.trajectory) {
    objectTrajectoryPreviewState.status = "EMPTY";
    objectTrajectoryPreviewState.error = "No object trajectory is loaded";
    updateObjectTrajectoryPreviewUi();
    return objectTrajectoryStateSnapshot();
  }
  try {
    const sample = objectTrajectorySampleAtIndex(
      objectTrajectoryPreviewState.trajectory,
      index,
    );
    cancelObjectTrajectoryAnimation();
    objectTrajectoryPreviewState.playing = false;
    applyObjectTrajectorySample(sample.index, "PAUSED");
  } catch (error) {
    objectTrajectoryPreviewState.status = "INVALID";
    objectTrajectoryPreviewState.error = error && error.message
      ? error.message
      : "Invalid object trajectory sample";
    updateObjectTrajectoryPreviewUi();
  }
  return objectTrajectoryStateSnapshot();
}

function setObjectTrajectoryTime(timeSeconds) {
  const trajectory = objectTrajectoryPreviewState.trajectory;
  if (!trajectory) {
    objectTrajectoryPreviewState.status = "EMPTY";
    objectTrajectoryPreviewState.error = "No object trajectory is loaded";
    updateObjectTrajectoryPreviewUi();
    return objectTrajectoryStateSnapshot();
  }
  try {
    if (typeof timeSeconds !== "number" || !Number.isFinite(timeSeconds)) {
      throw new TypeError("Object trajectory time must be a finite number");
    }
    const clampedRequestedTimeS = Math.max(
      0,
      Math.min(trajectory.durationS, timeSeconds),
    );
    const sample = nearestObjectTrajectorySample(
      trajectory,
      clampedRequestedTimeS,
    );
    cancelObjectTrajectoryAnimation();
    objectTrajectoryPreviewState.playing = false;
    // Preserve continuous requested time while applying only the nearest
    // discrete sample pose. No Cartesian interpolation occurs here.
    applyObjectTrajectorySample(
      sample.index,
      "PAUSED",
      clampedRequestedTimeS,
    );
  } catch (error) {
    objectTrajectoryPreviewState.status = "INVALID";
    objectTrajectoryPreviewState.error = error && error.message
      ? error.message
      : "Invalid object trajectory time";
    updateObjectTrajectoryPreviewUi();
  }
  return objectTrajectoryStateSnapshot();
}

function scheduleObjectTrajectoryFrame() {
  if (
    !objectTrajectoryPreviewState.playing
    || objectTrajectoryPreviewState.animationFrameId !== null
  ) return;
  objectTrajectoryPreviewState.animationFrameId = requestAnimationFrame(
    advanceObjectTrajectoryPlayback,
  );
}

function advanceObjectTrajectoryPlayback(frameTimeMs) {
  objectTrajectoryPreviewState.animationFrameId = null;
  const trajectory = objectTrajectoryPreviewState.trajectory;
  if (!objectTrajectoryPreviewState.playing || !trajectory) return;
  const previousTimeMs = objectTrajectoryPreviewState.previousFrameTimeMs;
  objectTrajectoryPreviewState.previousFrameTimeMs = frameTimeMs;
  const elapsedSeconds = previousTimeMs === null
    ? 0
    : Math.max(0, (frameTimeMs - previousTimeMs) / 1000);
  const nextTimeS = objectTrajectoryPreviewState.currentTimeS
    + elapsedSeconds * objectTrajectoryPreviewState.playbackRate;
  const clampedTimeS = Math.min(nextTimeS, trajectory.durationS);
  const sample = nearestObjectTrajectorySample(trajectory, clampedTimeS);

  if (nextTimeS >= trajectory.durationS) {
    applyObjectTrajectorySample(sample.index, "FINISHED", trajectory.durationS);
    objectTrajectoryPreviewState.playing = false;
    objectTrajectoryPreviewState.previousFrameTimeMs = null;
    updateObjectTrajectoryPreviewUi();
    return;
  }

  applyObjectTrajectorySample(sample.index, "PLAYING", clampedTimeS);
  scheduleObjectTrajectoryFrame();
}

function playObjectTrajectory() {
  const trajectory = objectTrajectoryPreviewState.trajectory;
  if (!trajectory) {
    objectTrajectoryPreviewState.status = "EMPTY";
    objectTrajectoryPreviewState.error = "No object trajectory is loaded";
    updateObjectTrajectoryPreviewUi();
    return objectTrajectoryStateSnapshot();
  }
  if (objectTrajectoryPreviewState.playing) return objectTrajectoryStateSnapshot();
  if (objectTrajectoryPreviewState.currentTimeS >= trajectory.durationS) {
    applyObjectTrajectorySample(0, "READY");
  }
  objectTrajectoryPreviewState.playing = true;
  objectTrajectoryPreviewState.status = "PLAYING";
  objectTrajectoryPreviewState.error = null;
  objectTrajectoryPreviewState.previousFrameTimeMs = performance.now();
  updateObjectTrajectoryPreviewUi();
  scheduleObjectTrajectoryFrame();
  return objectTrajectoryStateSnapshot();
}

function pauseObjectTrajectory() {
  if (!objectTrajectoryPreviewState.trajectory) {
    return objectTrajectoryStateSnapshot();
  }
  cancelObjectTrajectoryAnimation();
  objectTrajectoryPreviewState.playing = false;
  objectTrajectoryPreviewState.status = "PAUSED";
  updateObjectTrajectoryPreviewUi();
  return objectTrajectoryStateSnapshot();
}

function pauseObjectTrajectoryForManualPreview() {
  if (objectTrajectoryPreviewState.trajectory) {
    cancelObjectTrajectoryAnimation();
    objectTrajectoryPreviewState.playing = false;
    objectTrajectoryPreviewState.status = "MANUAL_OVERRIDE";
    objectTrajectoryPreviewState.error = null;
    updateObjectTrajectoryPreviewUi();
  }
  if (integratedPlanPreviewState.active) {
    pausePlannedTrajectory();
    integratedPlanPreviewState.owner = "MANUAL OBJECT PREVIEW";
    updateObjectGlobalPlanUi();
  }
}

function stopObjectTrajectory() {
  if (!objectTrajectoryPreviewState.trajectory) {
    return objectTrajectoryStateSnapshot();
  }
  cancelObjectTrajectoryAnimation();
  objectTrajectoryPreviewState.playing = false;
  applyObjectTrajectorySample(0, "READY");
  return objectTrajectoryStateSnapshot();
}

function clearObjectTrajectory() {
  cancelObjectTrajectoryAnimation();
  objectTrajectoryPreviewState.status = "EMPTY";
  objectTrajectoryPreviewState.trajectory = null;
  objectTrajectoryPreviewState.currentTimeS = 0;
  objectTrajectoryPreviewState.currentSampleIndex = 0;
  objectTrajectoryPreviewState.playing = false;
  objectTrajectoryPreviewState.error = null;
  // Preserve the last visible Object/Grasp pose; only trajectory ownership ends.
  updateObjectTrajectoryPreviewUi();
  return objectTrajectoryStateSnapshot();
}

function setObjectTrajectoryPlaybackRate(rate) {
  if (![0.25, 0.5, 1.0, 2.0].includes(rate)) {
    objectTrajectoryPreviewState.error = (
      "Object trajectory playback rate must be 0.25x, 0.5x, 1.0x, or 2.0x"
    );
    updateObjectTrajectoryPreviewUi();
    return objectTrajectoryStateSnapshot();
  }
  objectTrajectoryPreviewState.playbackRate = rate;
  objectTrajectoryPreviewState.error = null;
  if (objectTrajectoryPreviewState.playing) {
    objectTrajectoryPreviewState.previousFrameTimeMs = performance.now();
  }
  updateObjectTrajectoryPreviewUi();
  return objectTrajectoryStateSnapshot();
}

function bindObjectTrajectoryPreviewControls() {
  const bindings = {
    digitalTwinLoadSyntheticObjectTrajectory: loadSyntheticObjectTrajectory,
    digitalTwinObjectTrajectoryPrevious: () => setObjectTrajectorySampleIndex(
      Math.max(objectTrajectoryPreviewState.currentSampleIndex - 1, 0),
    ),
    digitalTwinObjectTrajectoryNext: () => setObjectTrajectorySampleIndex(
      Math.min(
        objectTrajectoryPreviewState.currentSampleIndex + 1,
        objectTrajectoryPreviewState.trajectory
          ? objectTrajectoryPreviewState.trajectory.sampleCount - 1
          : 0,
      ),
    ),
    digitalTwinObjectTrajectoryPlay: playObjectTrajectory,
    digitalTwinObjectTrajectoryPause: pauseObjectTrajectory,
    digitalTwinObjectTrajectoryStop: stopObjectTrajectory,
    digitalTwinObjectTrajectoryClear: clearObjectTrajectory,
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });

  const scrubber = document.getElementById("digitalTwinObjectTrajectoryScrubber");
  if (scrubber) {
    scrubber.addEventListener("input", (event) => {
      setObjectTrajectoryTime(Number(event.target.value));
    });
  }
  const rateSelector = document.getElementById(
    "digitalTwinObjectTrajectoryPlaybackRate",
  );
  if (rateSelector) {
    rateSelector.addEventListener("change", (event) => {
      setObjectTrajectoryPlaybackRate(Number(event.target.value));
    });
  }
}

const publicApi = {
  resetCamera,
  fitModel: () => fitModel(false),
  toggleGrid,
  toggleAxes,
  getAngleUnitPresentationState,
  setGlobalAngleUnit,
  setJointValues,
  getLatestActualPose,
  getLoadState,
  ingestStatusSnapshot,
  setMirrorEnabled,
  getMirrorState,
  resetToStaticPose,
  getModelTcpState,
  setModelTcpFeedbackStatus,
  setPlannedJointValues,
  showPlannedModel,
  hidePlannedModel,
  clearPlannedPreview,
  getPlannedPreviewState,
  captureActualAsPlanned,
  loadPlannedTrajectory,
  clearPlannedTrajectory,
  setTrajectoryTime,
  setTrajectoryPointIndex,
  playPlannedTrajectory,
  pausePlannedTrajectory,
  stopPlannedTrajectory,
  setTrajectoryPlaybackRate,
  getTrajectoryPreviewState,
  validateLoadedTrajectory,
  validateLoadedTrajectoryJointLimits,
  clearTrajectoryValidation,
  getTrajectoryValidationState,
  applyObjectPreviewPose: (pose) => {
    const state = applyObjectPreviewPose(pose);
    invalidateAllPhase4Validation("PUBLIC OBJECT POSE CHANGED");
    return state;
  },
  resetObjectPreview: () => {
    return resetObjectPreview();
  },
  resetGraspFramesToCurrentRobotPose,
  setObjectPreviewVisibility,
  getObjectPreviewState,
  getObjectGraspRelativeState,
  getRigidGraspConfigurationState,
  loadRigidGraspConfiguration,
  updateRigidGraspDraftSide,
  lockRigidGraspConfiguration,
  unlockRigidGraspConfiguration,
  editCurrentGrasp,
  startNewGraspFromCurrentRobotPose,
  setRigidGraspSelectedFrame,
  setRigidGraspGizmoMode,
  getObjectDragState,
  setObjectDragEnabled,
  getObjectWaypointVisualizationState,
  getObjectWaypointHoverPreviewState,
  showObjectWaypointHoverPreview,
  hideObjectWaypointHoverPreview,
  getPlanningStartState,
  setNextPlanningStartOverride,
  loadSyntheticObjectTrajectory,
  setObjectTrajectorySampleIndex,
  setObjectTrajectoryTime,
  playObjectTrajectory,
  pauseObjectTrajectory,
  stopObjectTrajectory,
  clearObjectTrajectory,
  setObjectTrajectoryPlaybackRate,
  getObjectTrajectoryPreviewState,
  getObjectWaypointPlanningState,
  getObjectWaypointState: getObjectWaypointPlanningState,
  getCenterPathLibraryState: centerPathLibrarySnapshot,
  refreshCenterPathLibrary,
  saveCurrentCenterPath,
  loadSelectedCenterPath,
  renameSelectedCenterPath,
  deleteSelectedCenterPath,
  setObjectPlanningWaypoints,
  addObjectPlanningWaypoint: addObjectPlanningWaypointToState,
  addCurrentIndependentApproachWaypoint,
  clearIndependentApproachWaypoints,
  addCurrentObjectPlanningWaypoint,
  deleteObjectPlanningWaypoint: deleteObjectPlanningWaypointFromState,
  moveObjectPlanningWaypoint: moveObjectPlanningWaypointInState,
  duplicateObjectPlanningWaypoint: duplicateObjectPlanningWaypointInState,
  reorderObjectPlanningWaypoint: reorderObjectPlanningWaypointInState,
  moveCenterToObjectWaypoint: moveCenterPreviewToObjectWaypoint,
  clearObjectPlanningWaypoints,
  loadObjectPlanningDemoFixture,
  planObjectGlobal,
  loadObjectGlobalPlanPreview,
  getObjectGlobalPlanState,
  getIntegratedPlanPreviewState,
  setIntegratedPlanTime,
  setExecutionWaypointTarget,
  hideExecutionWaypointTarget,
  playIntegratedPlan,
  pauseIntegratedPlan,
  resetIntegratedPlan,
  getPhase4TrajectoryValidationState,
  validateCurrentGlobalPlanPhase4,
  setPhase4ValidationStartState,
  clearPhase4ValidationStartState,
  getPhase4CollisionValidationState,
  validateCurrentGlobalPlanPhase4Collision,
  clearPhase4CollisionValidation,
  getPhase4UnifiedValidationState,
  validateCurrentGlobalPlanPhase4Unified,
  clearPhase4UnifiedValidation,
  getExecutionGateState,
  getWorldCalibrationState,
};

function handleResize() {
  if (!container || !camera || !renderer) return;
  const width = Math.max(container.clientWidth, 1);
  const height = Math.max(container.clientHeight, 1);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  render();
}

function bindControls() {
  const bindings = {
    digitalTwinResetCamera: resetCamera,
    digitalTwinFitModel: () => fitModel(false),
    digitalTwinToggleGrid: toggleGrid,
    digitalTwinToggleAxes: toggleAxes,
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });
}

function ingestOfflineMockPose(snapshot, label) {
  ingestStatusSnapshot(snapshot, Date.now(), label);
}

function bindMirrorControls() {
  const bindings = {
    digitalTwinEnableMirror: () => setMirrorEnabled(true),
    digitalTwinDisableMirror: () => setMirrorEnabled(false),
    digitalTwinResetStaticPose: resetToStaticPose,
    digitalTwinMockPoseA: () => ingestOfflineMockPose(
      MOCK_POSE_A,
      "OFFLINE MOCK A",
    ),
    digitalTwinMockPoseB: () => ingestOfflineMockPose(
      MOCK_POSE_B,
      "OFFLINE MOCK B",
    ),
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });
}

function bindPlannedPreviewControls() {
  const bindings = {
    digitalTwinCaptureActualAsPlanned: captureActualAsPlanned,
    digitalTwinPlannedMockA: () => setPlannedJointValues(
      PLANNED_MOCK_POSE_A,
      "PLANNED MOCK A",
    ),
    digitalTwinPlannedMockB: () => setPlannedJointValues(
      PLANNED_MOCK_POSE_B,
      "PLANNED MOCK B",
    ),
    digitalTwinShowPlanned: showPlannedModel,
    digitalTwinHidePlanned: hidePlannedModel,
    digitalTwinClearPlanned: clearPlannedPreview,
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });
}

function bindTrajectoryPreviewControls() {
  const bindings = {
    digitalTwinTrajectoryMockA: () => loadPlannedTrajectory(
      MOCK_TRAJECTORY_A,
      "OFFLINE MOCK TRAJECTORY A",
    ),
    digitalTwinTrajectoryMockB: () => loadPlannedTrajectory(
      MOCK_TRAJECTORY_B,
      "OFFLINE MOCK TRAJECTORY B",
    ),
    digitalTwinTrajectoryPrevious: () => setTrajectoryPointIndex(
      Math.max(trajectoryPreviewState.currentPointIndex - 1, 0),
    ),
    digitalTwinTrajectoryNext: () => setTrajectoryPointIndex(
      Math.min(
        trajectoryPreviewState.currentPointIndex + 1,
        trajectoryPreviewState.trajectory
          ? trajectoryPreviewState.trajectory.points.length - 1
          : 0,
      ),
    ),
    digitalTwinTrajectoryPlay: playPlannedTrajectory,
    digitalTwinTrajectoryPause: pausePlannedTrajectory,
    digitalTwinTrajectoryStop: stopPlannedTrajectory,
    digitalTwinTrajectoryClear: clearPlannedTrajectory,
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });

  const scrubber = document.getElementById("digitalTwinTrajectoryScrubber");
  if (scrubber) {
    scrubber.addEventListener("input", (event) => {
      setTrajectoryTime(Number(event.target.value));
    });
  }
  const timeInput = document.getElementById("digitalTwinTrajectoryTimeInput");
  if (timeInput) {
    const applyTimeInput = () => {
      const rawValue = String(timeInput.value).trim();
      const timeSeconds = rawValue === "" ? NaN : Number(rawValue);
      if (!Number.isFinite(timeSeconds)) {
        updateTrajectoryPreviewUi();
        return;
      }
      setTrajectoryTime(timeSeconds);
    };
    timeInput.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        applyTimeInput();
      }
    });
    timeInput.addEventListener("change", applyTimeInput);
    timeInput.addEventListener("blur", applyTimeInput);
  }
  const rateSelector = document.getElementById("digitalTwinTrajectoryPlaybackRate");
  if (rateSelector) {
    rateSelector.addEventListener("change", (event) => {
      setTrajectoryPlaybackRate(Number(event.target.value));
    });
  }
}

function bindTrajectoryValidationControls() {
  const bindings = {
    digitalTwinValidateStoredPoints: () => validateLoadedTrajectory(),
    digitalTwinValidateJointLimits: validateLoadedTrajectoryJointLimits,
    digitalTwinClearValidation: clearTrajectoryValidation,
    digitalTwinInvalidLimitTest: () => loadPlannedTrajectory(
      MOCK_INVALID_LIMIT_TRAJECTORY,
      "OFFLINE VALIDATION TEST",
    ),
    digitalTwinKnownCollisionTest: () => loadPlannedTrajectory(
      KNOWN_MOVEIT_COLLISION_TRAJECTORY,
      "OFFLINE MOVEIT TEST — NOT FOR ROBOT EXECUTION",
    ),
    digitalTwinSampledPathMockTest: () => loadPlannedTrajectory(
      SAMPLED_PATH_MOCK_TRAJECTORY,
      "SAMPLED-PATH MOCK TEST — NOT PHYSICALLY CONFIRMED",
    ),
    digitalTwinConfirmedSampledCollisionTest: () => loadPlannedTrajectory(
      CONFIRMED_SAMPLED_COLLISION_TRAJECTORY,
      "CONFIRMED MOVEIT RUNTIME FIXTURE — NOT FOR ROBOT EXECUTION",
    ),
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });
}

function countVisualMeshes(model) {
  let count = 0;
  const countedMeshes = new Set();
  model.traverse((object) => {
    const geometry = object.geometry;
    const positions = geometry && geometry.attributes
      ? geometry.attributes.position
      : null;
    if (
      object.isMesh &&
      positions &&
      positions.count > 0 &&
      !countedMeshes.has(object)
    ) {
      countedMeshes.add(object);
      count += 1;
    }
  });
  return count;
}

function inspectUrdfModelReadiness(model, role) {
  const missingJoints = EXPECTED_JOINTS.filter(
    (jointName) => !model.joints[jointName],
  );
  model.updateMatrixWorld(true);
  const diagnostic = {
    ready: false,
    role,
    expectedJointCount: EXPECTED_JOINTS.length,
    foundJointCount: EXPECTED_JOINTS.length - missingJoints.length,
    missingJoints,
    expectedVisualCount: EXPECTED_VISUAL_COUNT,
    foundVisualCount: countVisualMeshes(model),
    boundsEmpty: new THREE.Box3().setFromObject(model).isEmpty(),
  };
  diagnostic.ready = (
    diagnostic.missingJoints.length === 0 &&
    diagnostic.foundVisualCount === diagnostic.expectedVisualCount &&
    !diagnostic.boundsEmpty
  );
  return diagnostic;
}

function formatUrdfValidationError(diagnostic) {
  const missing = diagnostic.missingJoints.length > 0
    ? diagnostic.missingJoints.join(", ")
    : "none";
  return `${diagnostic.role} URDF validation failed: expected `
    + `${diagnostic.expectedJointCount} joints and `
    + `${diagnostic.expectedVisualCount} visual meshes, found `
    + `${diagnostic.foundJointCount} joints and `
    + `${diagnostic.foundVisualCount} visual meshes; `
    + `missing joints: ${missing}; bounds empty: ${diagnostic.boundsEmpty}`;
}

function makePlannedMaterialsTransparent(model) {
  const cloneMaterial = (material) => {
    const ghostMaterial = material.clone();
    ghostMaterial.transparent = true;
    ghostMaterial.opacity = PLANNED_GHOST_OPACITY;
    ghostMaterial.depthWrite = false;
    ghostMaterial.needsUpdate = true;
    return ghostMaterial;
  };

  model.traverse((object) => {
    if (!object.isMesh || !object.material) return;
    object.material = Array.isArray(object.material)
      ? object.material.map(cloneMaterial)
      : cloneMaterial(object.material);
  });
}

function failPlannedPreview(message, error = null) {
  const detail = error && error.message ? `${message}: ${error.message}` : message;
  plannedLoadState.status = "ERROR";
  plannedLoadState.error = detail;
  plannedPreviewState.status = "UNAVAILABLE";
  plannedPreviewState.visible = false;
  plannedPreviewState.validationError = detail;
  if (plannedRobot) plannedRobot.visible = false;
  updatePlannedPreviewUi();
  console.error(`[DualArmDigitalTwin:Planned] ${detail}`, error || "");
}

function createUrdfLoadLifecycle({
  role,
  onValidated,
  onFailure,
  onProgress = null,
}) {
  const state = {
    role,
    urdfParsed: false,
    assetsLoaded: false,
    finalized: false,
    failedAssets: new Set(),
    loadTimeout: null,
    completionTimer: null,
    readinessTimer: null,
    activityGeneration: 0,
    manager: new THREE.LoadingManager(),
    model: null,
    lastDiagnostic: null,
    readinessLogSignature: null,
  };

  function clearLifecycleTimers() {
    if (state.loadTimeout !== null) {
      window.clearTimeout(state.loadTimeout);
      state.loadTimeout = null;
    }
    if (state.completionTimer !== null) {
      window.clearTimeout(state.completionTimer);
      state.completionTimer = null;
    }
    if (state.readinessTimer !== null) {
      window.clearTimeout(state.readinessTimer);
      state.readinessTimer = null;
    }
  }

  function failLifecycle(message, error = null) {
    if (state.finalized) return;
    state.finalized = true;
    clearLifecycleTimers();
    onFailure(message, error);
  }

  function finalizeReadyModel(diagnostic) {
    state.finalized = true;
    clearLifecycleTimers();
    console.info(
      `[DualArmDigitalTwin][${role}] model ready: `
      + `joints=${diagnostic.foundJointCount} `
      + `visuals=${diagnostic.foundVisualCount}`,
    );
    onValidated(state.model, diagnostic);
  }

  function waitForModelReadiness() {
    if (
      state.finalized ||
      !state.urdfParsed ||
      !state.assetsLoaded ||
      !state.model
    ) return;

    if (state.failedAssets.size > 0) {
      failLifecycle(
        `Failed to load ${role} asset(s): ${Array.from(state.failedAssets).join(", ")}`,
      );
      return;
    }

    state.model.updateMatrixWorld(true);
    const diagnostic = inspectUrdfModelReadiness(state.model, role);
    state.lastDiagnostic = diagnostic;
    if (diagnostic.ready) {
      finalizeReadyModel(diagnostic);
      return;
    }

    const signature = [
      diagnostic.foundJointCount,
      diagnostic.foundVisualCount,
      diagnostic.boundsEmpty,
    ].join(":");
    if (signature !== state.readinessLogSignature) {
      state.readinessLogSignature = signature;
      console.info(
        `[DualArmDigitalTwin][${role}] waiting for visual attachment: `
        + `joints=${diagnostic.foundJointCount} `
        + `visuals=${diagnostic.foundVisualCount} `
        + `boundsEmpty=${diagnostic.boundsEmpty}`,
      );
    }
    state.readinessTimer = window.setTimeout(() => {
      state.readinessTimer = null;
      waitForModelReadiness();
    }, MODEL_READINESS_RETRY_MS);
  }

  state.manager.onStart = () => {
    if (!state.urdfParsed || state.finalized) return;
    state.assetsLoaded = false;
    state.activityGeneration += 1;
  };
  state.manager.onProgress = (url, loaded = 0, total = 0) => {
    if (state.urdfParsed && !state.finalized) {
      state.activityGeneration += 1;
    }
    if (onProgress) onProgress(url, loaded, total);
  };
  state.manager.onError = (url) => {
    state.failedAssets.add(String(url || `unknown ${role} asset`));
  };
  state.manager.onLoad = () => {
    if (!state.urdfParsed || state.finalized) {
      console.debug(
        `[DualArmDigitalTwin][${role}] ignoring pre-parse manager completion`,
      );
      return;
    }

    // Defer one task so any mesh items registered immediately after parsing
    // can invalidate this completion through activityGeneration.
    const completionGeneration = state.activityGeneration;
    if (state.completionTimer !== null) {
      window.clearTimeout(state.completionTimer);
    }
    state.completionTimer = window.setTimeout(() => {
      state.completionTimer = null;
      if (
        state.finalized ||
        !state.urdfParsed ||
        completionGeneration !== state.activityGeneration
      ) return;
      state.assetsLoaded = true;
      console.info(`[DualArmDigitalTwin][${role}] resource activity complete`);
      waitForModelReadiness();
    }, 0);
  };

  state.loadTimeout = window.setTimeout(() => {
    if (state.finalized) return;
    const failedAssets = Array.from(state.failedAssets);
    const diagnostic = state.lastDiagnostic;
    const readinessDetail = diagnostic
      ? formatUrdfValidationError(diagnostic)
      : "model readiness was never inspectable";
    failLifecycle(
      `Timed out waiting for ${role} model readiness: `
      + `parsed=${state.urdfParsed}, assetsLoaded=${state.assetsLoaded}, `
      + `failed assets=${failedAssets.length > 0 ? failedAssets.join(", ") : "none"}; `
      + readinessDetail,
    );
  }, LOAD_TIMEOUT_MS);

  function markParsed(model) {
    if (state.finalized) return;
    state.model = model;
    state.urdfParsed = true;
    state.assetsLoaded = false;
    state.activityGeneration += 1;
    console.info(`[DualArmDigitalTwin][${role}] URDF parsed; waiting for visuals`);
    waitForModelReadiness();
  }

  return {
    state,
    manager: state.manager,
    markParsed,
    fail: failLifecycle,
  };
}

function loadRobot() {
  const lifecycle = createUrdfLoadLifecycle({
    role: "Actual",
    onValidated: (_model, validation) => {
      if (!fitModel(true)) return;
      createModelTcpFrames();
      loadState.status = "READY";
      loadState.loadedMovableJoints = validation.foundJointCount;
      loadState.loadedVisuals = validation.foundVisualCount;
      loadState.error = null;
      setStatus(
        `READY — ${validation.foundJointCount} MOVABLE JOINTS LOADED`
        + ` — ${validation.foundVisualCount} VISUALS`,
        "ready",
      );
      if (!applyLatestMirrorSnapshotIfPossible(Date.now())) {
        applyPlanningStartStateToMainWhenOffline(Date.now());
        updateMirrorUi();
      }
    },
    onFailure: (message, error) => fail(message, error),
    onProgress: (_url, loaded, total) => {
      const percent = total > 0 ? Math.round((loaded / total) * 100) : 0;
      setStatus(`LOADING ASSETS ${loaded}/${total} (${percent}%)`);
    },
  });

  const loader = new URDFLoader(lifecycle.manager);
  loader.parseCollision = false;

  loadState.status = "LOADING";
  setStatus("LOADING DUAL JAKA A12 URDF");

  loader.load(
    MODEL_URL,
    (loadedRobot) => {
      const missingJoints = EXPECTED_JOINTS.filter(
        (jointName) => !loadedRobot.joints[jointName],
      );

      if (missingJoints.length > 0) {
        lifecycle.fail(
          `Actual URDF is missing expected joints: ${missingJoints.join(", ")}`,
        );
        return;
      }

      robot = loadedRobot;

      EXPECTED_JOINTS.forEach((jointName) => {
        robot.joints[jointName].setJointValue(0);
      });
      latestActualPose = {
        left: [...ZERO_JOINT_VALUES.left],
        right: [...ZERO_JOINT_VALUES.right],
      };
      updatePlannedDelta();
      updatePlannedPreviewUi();

      scene.add(robot);
      robot.updateMatrixWorld(true);
      setStatus("URDF PARSED — WAITING FOR VISUAL MESHES");
      lifecycle.markParsed(robot);
    },
    (event) => {
      if (event && event.lengthComputable && event.total > 0) {
        const percent = Math.round((event.loaded / event.total) * 100);
        setStatus(`LOADING URDF ${percent}%`);
      }
    },
    (error) => {
      lifecycle.fail("Failed to load Actual URDF", error);
    },
  );
}

function loadPlannedRobot() {
  if (plannedLoadStarted) return;
  plannedLoadStarted = true;
  plannedLoadState.status = "LOADING";

  const lifecycle = createUrdfLoadLifecycle({
    role: "Planned",
    onValidated: (_model, validation) => {
      makePlannedMaterialsTransparent(plannedRobot);
      plannedRobot.visible = false;
      plannedLoadState.status = "READY";
      plannedLoadState.loadedMovableJoints = validation.foundJointCount;
      plannedLoadState.loadedVisuals = validation.foundVisualCount;
      plannedLoadState.error = null;
      plannedPreviewState.status = "READY";
      plannedPreviewState.visible = false;
      plannedPreviewState.validationError = null;

      if (plannedPreviewState.latestPose) {
        applyLatestPlannedPose();
      } else {
        updatePlannedPreviewUi();
        render();
      }
    },
    onFailure: (message, error) => failPlannedPreview(message, error),
  });

  const loader = new URDFLoader(lifecycle.manager);
  loader.parseCollision = false;
  loader.load(
    MODEL_URL,
    (loadedRobot) => {
      const missingJoints = EXPECTED_JOINTS.filter(
        (jointName) => !loadedRobot.joints[jointName],
      );
      if (missingJoints.length > 0) {
        lifecycle.fail(
          `Planned URDF is missing expected joints: ${missingJoints.join(", ")}`,
        );
        return;
      }

      plannedRobot = loadedRobot;
      EXPECTED_JOINTS.forEach((jointName) => {
        plannedRobot.joints[jointName].setJointValue(0);
      });
      plannedRobot.visible = false;
      scene.add(plannedRobot);
      plannedRobot.updateMatrixWorld(true);
      lifecycle.markParsed(plannedRobot);
    },
    undefined,
    (error) => {
      lifecycle.fail("Failed to load Planned URDF", error);
    },
  );
}

function initialize() {
  container = document.getElementById("digitalTwinViewer");
  statusElement = document.getElementById("digitalTwinStatus");
  if (!container) {
    fail("Digital Twin viewer container is missing");
    return;
  }

  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1120);
  camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);

  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  } catch (error) {
    fail("WebGL initialization failed", error);
    return;
  }
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.replaceChildren(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.enableZoom = false;
  renderer.domElement.addEventListener("wheel", handleViewerWheel, { passive: false });
  controls.addEventListener("change", render);

  scene.add(new THREE.AmbientLight(0xffffff, 1.6));
  const directionalLight = new THREE.DirectionalLight(0xffffff, 2.2);
  directionalLight.position.set(3, -4, 6);
  scene.add(directionalLight);

  grid = new THREE.GridHelper(6, 30, 0x475569, 0x273449);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  axes = new THREE.AxesHelper(0.75);
  axes.name = "World Frame W axes";
  scene.add(axes);

  createObjectGraspPreview();
  initializeObjectTransformControls();
  initializeObjectWaypointVisualization();

  bindGlobalAngleUnitControl();
  bindControls();
  bindObjectPreviewControls();
  bindRigidGraspConfigurationControls();
  bindObjectTrajectoryPreviewControls();
  bindObjectWaypointPlanningControls();
  bindMirrorControls();
  bindPlannedPreviewControls();
  bindTrajectoryPreviewControls();
  bindTrajectoryValidationControls();
  bindPhase4TrajectoryValidationControls();
  bindPhase4CollisionValidationControls();
  bindPhase4UnifiedValidationControls();
  bindPhase5ExecutionControls();
  updateMirrorUi();
  updatePlannedPreviewUi();
  updateTrajectoryPreviewUi();
  updateTrajectoryValidationUi();
  updatePhase4TrajectoryValidationUi();
  updatePhase4CollisionValidationUi();
  updatePhase4UnifiedValidationUi();
  updateObjectTrajectoryPreviewUi();
  updateObjectDragUi();
  updateObjectWaypointPlanningUi();
  updateObjectGlobalPlanUi();
  updateModelTcpUi();
  updateWorldCalibrationUi();
  updateRigidGraspConfigurationUi();
  window.setInterval(() => {
    updateMirrorStaleness(Date.now());
  }, 250);
  window.addEventListener("resize", handleResize);
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(handleResize).observe(container);
  }
  handleResize();

  const animate = (frameTimeMs = 0) => {
    requestAnimationFrame(animate);
    applySmoothedMirrorVisualFrame(frameTimeMs, Date.now());
    if (controls) controls.update();
    render();
  };
  animate();
  loadRobot();
  loadPlannedRobot();
  loadPlanningStartState();
  loadWorldCalibrationState();
  loadRigidGraspConfiguration();
}

if (window[INITIALIZATION_FLAG]) {
  console.warn("[DualArmDigitalTwin] Duplicate module initialization ignored");
} else {
  window[INITIALIZATION_FLAG] = true;
  window.dualArmDigitalTwin = publicApi;
  initialize();
}
