// Phase 1B/1C.1 pinned browser dependencies:
// Three.js 0.160.0 and urdf-loader 0.12.5 (resolved by index.html import map).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";
import {
  DEFAULT_STALE_TIMEOUT_MS,
  isNormalizedSnapshotStale,
  normalizeDualArmStatusSnapshot,
} from "./digital_twin_status_adapter.js";
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

const MODEL_URL = "/digital-twin/assets/dual_jaka_a12_web.urdf";
const LOAD_TIMEOUT_MS = 20000;
const MODEL_READINESS_RETRY_MS = 40;
const EXPECTED_VISUAL_COUNT = 14;
const PLANNED_GHOST_OPACITY = 0.38;
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

const loadState = {
  status: "INITIALIZING",
  loadedMovableJoints: 0,
  loadedVisuals: 0,
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
  previousFrameTimeMs: null,
  animationFrameId: null,
};

let container = null;
let statusElement = null;
let scene = null;
let camera = null;
let renderer = null;
let controls = null;
let grid = null;
let axes = null;
let robot = null;
let plannedRobot = null;
let latestActualPose = null;
let plannedLoadStarted = false;
let homeCameraPosition = null;
let homeCameraTarget = null;

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
  const halfFovRadians = THREE.MathUtils.degToRad(camera.fov * 0.5);
  const distance = (maxDimension * 0.65) / Math.tan(halfFovRadians);
  const viewDirection = new THREE.Vector3(1, -1, 0.7).normalize();

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
  return true;
}

function loadPlannedTrajectory(
  trajectory,
  sourceLabel = "LOCAL TRAJECTORY",
) {
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
    digitalTwinMirrorError: mirrorState.validationError || "NONE",
  };
  Object.entries(values).forEach(([id, value]) => {
    const element = document.getElementById(id);
    if (element) element.textContent = value;
  });
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
    updateMirrorUi();
  }
  return stale;
}

function applyLatestMirrorSnapshotIfPossible(nowMs = Date.now()) {
  if (!mirrorState.enabled) return false;
  if (!mirrorState.latestValidSnapshot) {
    mirrorState.mode = MIRROR_MODES.MIRROR_READY;
    updateMirrorUi();
    return false;
  }
  if (updateMirrorStaleness(nowMs)) return false;
  if (loadState.status !== "READY" || !robot) {
    mirrorState.mode = MIRROR_MODES.MIRROR_READY;
    updateMirrorUi();
    return false;
  }

  setJointValues({
    left: mirrorState.latestValidSnapshot.left,
    right: mirrorState.latestValidSnapshot.right,
  });
  mirrorState.mode = MIRROR_MODES.LIVE_MIRROR;
  mirrorState.lastAppliedSnapshotMs = nowMs;
  mirrorState.validationError = null;
  updateMirrorUi();
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
    mirrorState.validationError = error && error.message
      ? error.message
      : "Invalid status snapshot";
    updateMirrorUi();
    return mirrorStateSnapshot();
  }

  mirrorState.latestValidSnapshot = normalized;
  mirrorState.lastAcceptedSnapshotMs = normalized.receivedAtMs;
  mirrorState.validationError = null;
  mirrorState.updateSource = (
    typeof sourceLabel === "string" && sourceLabel.trim().length > 0
  )
    ? sourceLabel.trim()
    : "LOCAL STATUS SNAPSHOT";

  if (!mirrorState.enabled) {
    mirrorState.mode = MIRROR_MODES.STATIC;
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
    updateMirrorUi();
    return mirrorStateSnapshot();
  }

  mirrorState.validationError = null;
  if (!mirrorState.latestValidSnapshot) {
    mirrorState.mode = MIRROR_MODES.MIRROR_READY;
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
  if (loadState.status === "READY" && robot) {
    setJointValues({
      left: [...ZERO_JOINT_VALUES.left],
      right: [...ZERO_JOINT_VALUES.right],
    });
  }
  mirrorState.enabled = false;
  mirrorState.mode = MIRROR_MODES.STATIC;
  mirrorState.lastAppliedSnapshotMs = null;
  mirrorState.validationError = null;
  mirrorState.updateSource = "STATIC RESET";
  updateMirrorUi();
  return mirrorStateSnapshot();
}

const publicApi = {
  resetCamera,
  fitModel: () => fitModel(false),
  toggleGrid,
  toggleAxes,
  setJointValues,
  getLoadState,
  ingestStatusSnapshot,
  setMirrorEnabled,
  getMirrorState,
  resetToStaticPose,
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
      loadState.status = "READY";
      loadState.loadedMovableJoints = validation.foundJointCount;
      loadState.loadedVisuals = validation.foundVisualCount;
      loadState.error = null;
      setStatus(
        `READY — ${validation.foundJointCount} MOVABLE JOINTS LOADED`
        + ` — ${validation.foundVisualCount} VISUALS`,
        "ready",
      );
      applyLatestMirrorSnapshotIfPossible(Date.now());
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
  controls.addEventListener("change", render);

  scene.add(new THREE.AmbientLight(0xffffff, 1.6));
  const directionalLight = new THREE.DirectionalLight(0xffffff, 2.2);
  directionalLight.position.set(3, -4, 6);
  scene.add(directionalLight);

  grid = new THREE.GridHelper(6, 30, 0x475569, 0x273449);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  axes = new THREE.AxesHelper(0.75);
  scene.add(axes);

  bindControls();
  bindMirrorControls();
  bindPlannedPreviewControls();
  bindTrajectoryPreviewControls();
  updateMirrorUi();
  updatePlannedPreviewUi();
  updateTrajectoryPreviewUi();
  window.setInterval(() => {
    updateMirrorStaleness(Date.now());
  }, 250);
  window.addEventListener("resize", handleResize);
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(handleResize).observe(container);
  }
  handleResize();

  const animate = () => {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    render();
  };
  animate();
  loadRobot();
  loadPlannedRobot();
}

if (window[INITIALIZATION_FLAG]) {
  console.warn("[DualArmDigitalTwin] Duplicate module initialization ignored");
} else {
  window[INITIALIZATION_FLAG] = true;
  window.dualArmDigitalTwin = publicApi;
  initialize();
}
