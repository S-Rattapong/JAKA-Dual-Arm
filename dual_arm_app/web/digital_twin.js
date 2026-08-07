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
  updateMirrorUi();
  updatePlannedPreviewUi();
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
