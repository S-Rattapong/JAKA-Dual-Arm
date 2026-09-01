"use strict";

import {
  EXECUTION_GHOST_ACTUAL_FRESHNESS_MS,
  createRigidWaypointTargetCursor,
  updateRigidWaypointTargetCursor,
} from "./digital_twin_phase5_waypoint_target.js";

// P5.8-P5.15 operator-only execution controls and read-only driver feedback.
// Module initialization performs
// read-only GET polling only; no POST endpoint is called automatically.
const STATUS_ENDPOINT = "/api/digital-twin/phase5/execution";
const ARTIFACT_ENDPOINT = "/api/digital-twin/phase5/execution-artifact";
const PREPARE_ENDPOINT = "/api/digital-twin/phase5/prepare";
const EXECUTE_ENDPOINT = "/api/digital-twin/phase5/execute";
const REPLAN_FROM_CURRENT_ENDPOINT = "/api/digital-twin/phase5/replan-from-current";
const MOVE_TO_INITIAL_ENDPOINT = "/api/digital-twin/phase5/move-to-initial";
const FROZEN_PREVIEW_SOURCE_PREFIX = "PHASE 5 FROZEN ARTIFACT PREVIEW";
const ACTIVE_STATES = new Set(["PREPARING", "ARMED", "RUNNING", "ABORT_REQUESTED"]);
const EXECUTION_TARGET_STATES = new Set(["ARMED", "RUNNING"]);
const TERMINAL_GHOST_HIDE_STATES = new Set(["COMPLETED", "ABORTED", "FAILED"]);
const STATUS_POLL_INTERVAL_MS = 100;

let controlsBound = false;
let statusPollId = null;

const state = {
  operatorAuthorityPrepared: false,
  preparedStopGeneration: null,
  stopGeneration: null,
  requestInFlight: false,
  statusRequestInFlight: false,
  previewRequestInFlight: false,
  previewFingerprint: null,
  previewGeneration: null,
  previewName: null,
  previewSampleCount: null,
  previewDurationS: null,
  previewStatus: "NOT_LOADED",
  previewError: null,
  executionGhostHidden: false,
  executionTargetCursor: null,
  manualPreviewOverride: false,
  artifactFingerprint: null,
  artifactGeneration: null,
  lastOperatorAction: "READY FOR OPERATOR",
};

function element(id) {
  return document.getElementById(id);
}

function setText(id, value) {
  const target = element(id);
  if (target) target.textContent = String(value);
}

function setOperatorAction(message) {
  state.lastOperatorAction = String(message);
  setText("digitalTwinPhase5OperatorAction", state.lastOperatorAction);
}

function errorText(payload, fallback) {
  if (!payload || typeof payload !== "object") return fallback;
  const blockers = Array.isArray(payload.blocking_reasons)
    ? payload.blocking_reasons.join(", ") : "";
  return [payload.error, payload.detail, blockers].filter(Boolean).join(": ") || fallback;
}

async function requestJson(url, options = {}) {
  const response = await fetch(url, { cache: "no-store", ...options });
  const payload = await response.json();
  if (!response.ok) throw new Error(errorText(payload, `HTTP ${response.status}`));
  return payload;
}

function frozenPreviewSource(fingerprint) {
  return `${FROZEN_PREVIEW_SOURCE_PREFIX} — ${fingerprint}`;
}

function clearOwnedFrozenPreview() {
  const digitalTwin = window.dualArmDigitalTwin;
  if (
    !digitalTwin
    || typeof digitalTwin.getTrajectoryPreviewState !== "function"
    || typeof digitalTwin.clearPlannedTrajectory !== "function"
  ) return;
  const preview = digitalTwin.getTrajectoryPreviewState();
  if (
    preview
    && typeof preview.source === "string"
    && preview.source.startsWith(FROZEN_PREVIEW_SOURCE_PREFIX)
  ) {
    digitalTwin.clearPlannedTrajectory();
  }
}

function currentFrozenPreviewLoaded() {
  if (!state.artifactFingerprint || state.artifactGeneration === null) return false;
  if (
    state.previewFingerprint !== state.artifactFingerprint
    || state.previewGeneration !== state.artifactGeneration
    || state.previewStatus !== "LOADED"
  ) return false;
  const digitalTwin = window.dualArmDigitalTwin;
  if (
    !digitalTwin
    || typeof digitalTwin.getTrajectoryPreviewState !== "function"
    || typeof digitalTwin.getPlannedPreviewState !== "function"
  ) return false;
  const trajectory = digitalTwin.getTrajectoryPreviewState();
  const planned = digitalTwin.getPlannedPreviewState();
  const expectedSource = frozenPreviewSource(state.artifactFingerprint);
  return Boolean(
    trajectory
    && trajectory.status !== "INVALID"
    && trajectory.source === expectedSource
    && trajectory.pointCount === state.previewSampleCount
    && Math.abs(Number(trajectory.durationS) - Number(state.previewDurationS)) < 1e-9
    && planned
    && planned.source === expectedSource
    && planned.loadState
    && planned.loadState.status === "READY"
  );
}

function currentFrozenPreviewReady() {
  if (!currentFrozenPreviewLoaded()) return false;
  return window.dualArmDigitalTwin.getPlannedPreviewState().visible === true;
}

function frozenArtifactTrajectory(payload, expectedArtifact) {
  if (
    !payload
    || payload.ok !== true
    || payload.available !== true
    || payload.status !== "FROZEN"
    || !payload.artifact
  ) throw new Error("Frozen Phase-5 artifact is unavailable");
  const artifact = payload.artifact;
  if (
    artifact.artifact_fingerprint !== expectedArtifact.artifact_fingerprint
    || Number(payload.generation) !== Number(expectedArtifact.generation)
  ) throw new Error("Frozen artifact changed while loading the 3D Ghost preview");
  const trajectory = artifact.trajectory || {};
  const times = trajectory.common_timestamps_s;
  const left = (trajectory.left || {}).positions_rad;
  const right = (trajectory.right || {}).positions_rad;
  if (
    !Array.isArray(times)
    || !Array.isArray(left)
    || !Array.isArray(right)
    || times.length < 2
    || left.length !== times.length
    || right.length !== times.length
  ) throw new Error("Frozen artifact trajectory shape is invalid for 3D preview");
  return {
    fingerprint: artifact.artifact_fingerprint,
    generation: Number(payload.generation),
    name: trajectory.name || "Frozen Phase-5 Trajectory",
    sampleCount: times.length,
    durationS: Number(trajectory.duration_s),
    trajectory: {
      name: trajectory.name || "Frozen Phase-5 Trajectory",
      points: times.map((time, index) => ({
        time_from_start_s: Number(time),
        left: [...left[index]],
        right: [...right[index]],
      })),
    },
  };
}

function renderFrozenPreviewState(executionState = "IDLE") {
  const ready = currentFrozenPreviewReady();
  const loaded = currentFrozenPreviewLoaded();
  let label = state.previewStatus;
  if (state.previewRequestInFlight) label = "LOADING FROZEN ARTIFACT…";
  else if (ready) label = "READY — MATCHES FROZEN ARTIFACT";
  else if (loaded) label = "LOADED — GHOST HIDDEN AFTER EXECUTION";
  else if (state.previewError) label = `BLOCKED — ${state.previewError}`;
  else if (state.previewStatus === "LOADED") label = "WAITING FOR PLANNED GHOST MODEL";
  setText("digitalTwinPhase5PreviewStatus", label);
  setText(
    "digitalTwinPhase5PreviewTrajectory",
    state.previewName
      ? `${state.previewName}: ${state.previewSampleCount} samples, ${state.previewDurationS} s`
      : "UNAVAILABLE",
  );
  setText(
    "digitalTwinPhase5PreviewInterlock",
    ready ? "READY — PREPARE/EXECUTE MAY PROCEED" : "BLOCKED — VERIFIED GHOST REQUIRED",
  );
  const active = ACTIVE_STATES.has(executionState);
  for (const id of ["digitalTwinPhase5PreviewPlay", "digitalTwinPhase5PreviewReset"]) {
    const button = element(id);
    if (button) button.disabled = !loaded || active;
  }
}

async function ensureFrozenArtifactPreview(artifact, { force = false } = {}) {
  state.artifactFingerprint = artifact && artifact.available
    ? artifact.artifact_fingerprint : null;
  state.artifactGeneration = artifact && artifact.available
    ? Number(artifact.generation) : null;
  if (!state.artifactFingerprint || state.artifactGeneration === null) {
    state.previewStatus = "NOT_AVAILABLE";
    state.previewError = "NO FROZEN ARTIFACT";
    state.previewFingerprint = null;
    state.previewGeneration = null;
    state.executionGhostHidden = false;
    state.executionTargetCursor = null;
    state.manualPreviewOverride = false;
    clearOwnedFrozenPreview();
    renderFrozenPreviewState();
    return false;
  }
  const sameIdentity = (
    state.previewFingerprint === state.artifactFingerprint
    && state.previewGeneration === state.artifactGeneration
  );
  if (!sameIdentity) {
    state.executionGhostHidden = false;
    state.executionTargetCursor = null;
    state.manualPreviewOverride = false;
  }
  if (!force && sameIdentity && state.previewStatus === "LOADED") {
    renderFrozenPreviewState();
    return currentFrozenPreviewReady();
  }
  if (state.previewRequestInFlight) return false;
  state.previewRequestInFlight = true;
  state.previewStatus = "LOADING";
  state.previewError = null;
  renderFrozenPreviewState();
  try {
    const loaded = frozenArtifactTrajectory(
      await requestJson(ARTIFACT_ENDPOINT), artifact,
    );
    const digitalTwin = window.dualArmDigitalTwin;
    if (!digitalTwin || typeof digitalTwin.loadPlannedTrajectory !== "function") {
      throw new Error("Digital Twin planned-trajectory renderer is unavailable");
    }
    const preview = digitalTwin.loadPlannedTrajectory(
      loaded.trajectory,
      frozenPreviewSource(loaded.fingerprint),
      { preserveIntegratedPlan: true, syncIntegratedPlan: false },
    );
    if (!preview || preview.status === "INVALID") {
      throw new Error((preview && preview.validationError) || "Frozen Ghost trajectory rejected");
    }
    state.previewFingerprint = loaded.fingerprint;
    state.previewGeneration = loaded.generation;
    state.previewName = loaded.name;
    state.previewSampleCount = loaded.sampleCount;
    state.previewDurationS = loaded.durationS;
    state.previewStatus = "LOADED";
    state.previewError = null;
  } catch (error) {
    state.previewStatus = "ERROR";
    state.previewError = error && error.message
      ? error.message : "Frozen Ghost preview failed closed";
    clearOwnedFrozenPreview();
    clearPendingOperatorAuthority("FROZEN GHOST PREVIEW INVALID");
  } finally {
    state.previewRequestInFlight = false;
    renderFrozenPreviewState();
  }
  return currentFrozenPreviewReady();
}

function playFrozenArtifactPreview() {
  if (!currentFrozenPreviewLoaded()) return;
  const digitalTwin = window.dualArmDigitalTwin;
  if (!digitalTwin) return;
  state.executionGhostHidden = false;
  state.manualPreviewOverride = true;
  digitalTwin.stopPlannedTrajectory();
  digitalTwin.setTrajectoryPlaybackRate(1.0);
  digitalTwin.playPlannedTrajectory();
  setText("digitalTwinPhase5Error", "PLAYING FROZEN GHOST — VISUALIZATION ONLY / NO MOTION");
}

function resetFrozenArtifactPreview() {
  if (!currentFrozenPreviewLoaded()) return;
  const digitalTwin = window.dualArmDigitalTwin;
  if (!digitalTwin) return;
  state.executionGhostHidden = false;
  state.manualPreviewOverride = true;
  digitalTwin.stopPlannedTrajectory();
  setText("digitalTwinPhase5Error", "FROZEN GHOST RESET TO START — NO MOTION");
}

function clearPendingOperatorAuthority(reason = "NO PENDING AUTHORITY") {
  state.operatorAuthorityPrepared = false;
  state.preparedStopGeneration = null;
  setOperatorAction(reason);
}

function finiteNumber(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function percent(value) {
  const number = finiteNumber(value);
  return number === null ? "N/A" : `${(number * 100).toFixed(1)}%`;
}

function renderCommonProgress(value, previousArtifact = false) {
  const number = finiteNumber(value);
  const clamped = number === null ? 0 : Math.min(1, Math.max(0, number));
  setText(
    "digitalTwinPhase5CommonProgress",
    previousArtifact ? `PREVIOUS ARTIFACT — ${percent(value)}` : percent(value),
  );
  const progress = element("digitalTwinPhase5ProgressBar");
  if (progress) {
    progress.value = clamped * 100;
    progress.dataset.executionArtifact = previousArtifact ? "PREVIOUS" : "CURRENT";
  }
}

function formatJointArray(side) {
  return side && side.valid === true && Array.isArray(side.joint)
    ? `[${side.joint.map((value) => Number(value).toFixed(5)).join(", ")}]`
    : "UNAVAILABLE";
}

function formatJointCache(side) {
  if (!side || side.valid !== true) return "valid=false; received=N/A; age=N/A";
  return `valid=true; received=${side.received_at_ms} ms; age=${side.age_ms} ms`;
}

function formatTcpSide(side) {
  if (!side || side.valid !== true || !Array.isArray(side.tcp)) return "UNAVAILABLE";
  return `[${side.tcp.map((value) => Number(value).toFixed(6)).join(", ")}]`;
}

function renderReusedActualTcp() {
  const source = window.dualArmDigitalTwinTcpSource;
  const tcpState = source && typeof source.getTcpSourceState === "function"
    ? source.getTcpSourceState() : null;
  const snapshot = tcpState ? tcpState.latestSnapshot : null;
  setText("digitalTwinPhase5ActualTcpStatus", tcpState ? tcpState.status : "UNAVAILABLE");
  setText("digitalTwinPhase5ActualLeftTcp", formatTcpSide(snapshot && snapshot.left));
  setText("digitalTwinPhase5ActualRightTcp", formatTcpSide(snapshot && snapshot.right));
}

function renderDriverSide(sideName, side, previousArtifact = false) {
  const title = sideName[0].toUpperCase() + sideName.slice(1);
  const prefix = previousArtifact ? "PREVIOUS ARTIFACT — " : "";
  const stateLabel = side && side.state
    ? side.state : (side && side.feedback_status ? side.feedback_status : "UNAVAILABLE");
  setText(
    `digitalTwinPhase5${title}Driver`,
    `${prefix}${stateLabel} / ${percent(side && side.progress_0_to_1)}`,
  );
  setText(
    `digitalTwinPhase5${title}Reason`,
    `${prefix}${(side && (side.terminal_reason || side.error)) || "NONE"}`,
  );
}

function formatNumber(value, digits = 6) {
  const number = finiteNumber(value);
  return number === null ? "N/A" : number.toFixed(digits);
}

function formatTrackingVector(values) {
  return Array.isArray(values) && values.length === 6
    ? `[${values.map((value) => formatNumber(value, 5)).join(", ")}]`
    : "UNAVAILABLE";
}

function renderPhase6JointTracking(monitor) {
  const tracking = monitor || {};
  const left = tracking.left || {};
  const right = tracking.right || {};
  const combined = tracking.combined || {};
  setText("digitalTwinP6JointTrackingStatus", tracking.status || "NOT EVALUATED");
  setText("digitalTwinP6JointTrackingSource", tracking.actual_source || "UNAVAILABLE");
  setText("digitalTwinP6JointTrackingCombinedMax", formatNumber(combined.max_abs_error_rad));
  setText("digitalTwinP6JointTrackingCombinedJoint", combined.max_error_joint_name || "N/A");
  setText("digitalTwinP6JointTrackingLeftMax", formatNumber(left.max_abs_error_rad));
  setText("digitalTwinP6JointTrackingLeftJoint", left.max_error_joint_name || "N/A");
  setText("digitalTwinP6JointTrackingRightMax", formatNumber(right.max_abs_error_rad));
  setText("digitalTwinP6JointTrackingRightJoint", right.max_error_joint_name || "N/A");
  setText("digitalTwinP6JointTrackingLeftVector", formatTrackingVector(left.error_rad));
  setText("digitalTwinP6JointTrackingRightVector", formatTrackingVector(right.error_rad));
  setText("digitalTwinP6JointTrackingTiming", tracking.time_alignment || "LATEST COMMAND VS LATEST ACTUAL");
  setText(
    "digitalTwinP6JointTrackingAuthority",
    tracking.monitoring_only === true && tracking.threshold_applied !== true
      ? "OBSERVE ONLY — NO THRESHOLD / STOP" : "UNAVAILABLE",
  );
}

function formatQuality(side) {
  const quality = side && side.motion_quality;
  if (!quality) return "UNAVAILABLE";
  return `samples=${quality.sample_count}; `
    + `|v|max=${formatNumber(quality.max_abs_velocity_rad_s)} rad/s; `
    + `|a|max=${formatNumber(quality.max_abs_acceleration_rad_s2)} rad/s²; `
    + `|j|max=${formatNumber(quality.max_abs_jerk_rad_s3)} rad/s³; `
    + `RMS=${formatNumber(quality.rms_velocity_rad_s)}/`
    + `${formatNumber(quality.rms_acceleration_rad_s2)}/`
    + `${formatNumber(quality.rms_jerk_rad_s3)}`;
}

function formatTiming(side) {
  const timing = side && side.dispatch_timing;
  if (!timing) return "UNAVAILABLE";
  const call = side.servo_j_call_duration || {};
  const stream = side.servo_stream || {};
  const startNs = Number(side.start_time_unix_ns);
  const firstDispatchNs = Number(stream.first_dispatch_unix_ns);
  const firstReturnNs = Number(stream.first_servo_return_unix_ns);
  const lastDispatchNs = Number(stream.last_dispatch_unix_ns);
  const lastReturnNs = Number(stream.last_servo_return_unix_ns);
  const offsetMs = (valueNs) => (
    Number.isFinite(valueNs) && Number.isFinite(startNs) && valueNs > 0 && startNs > 0
      ? (valueNs - startNs) / 1e6
      : null
  );
  return `step=${stream.servo_step_num ?? "N/A"}; `
    + `period=${formatNumber(stream.command_period_ms, 3)} ms; `
    + `telemetry=${stream.telemetry_mode || "UNAVAILABLE"}; `
    + `telemetry polls suppressed=${stream.telemetry_suppressed_poll_count ?? 0}; `
    + `first dispatch/return offset=${formatNumber(offsetMs(firstDispatchNs), 3)}/`
    + `${formatNumber(offsetMs(firstReturnNs), 3)} ms; `
    + `last dispatch/return offset=${formatNumber(offsetMs(lastDispatchNs), 3)}/`
    + `${formatNumber(offsetMs(lastReturnNs), 3)} ms; `
    + `guard observed/limit=${formatNumber(stream.guard_observed_max_rad_s)}/`
    + `${formatNumber(stream.guard_limit_rad_s)} rad/s; `
    + `dispatch samples=${timing.sample_count}; late mean/p95/p99/max=`
    + `${formatNumber(timing.mean_abs_lateness_ms, 3)}/`
    + `${formatNumber(timing.p95_abs_lateness_ms, 3)}/`
    + `${formatNumber(timing.p99_abs_lateness_ms, 3)}/`
    + `${formatNumber(timing.max_abs_lateness_ms, 3)} ms; jitter=`
    + `${formatNumber(timing.mean_abs_jitter_ms, 3)}/`
    + `${formatNumber(timing.p95_abs_jitter_ms, 3)}/`
    + `${formatNumber(timing.p99_abs_jitter_ms, 3)}/`
    + `${formatNumber(timing.max_abs_jitter_ms, 3)} ms; `
    + `missed=${timing.missed_cycle_count}; servo_j call samples=${call.sample_count ?? 0}; `
    + `overruns=${call.overrun_count ?? 0}; mean/p95/p99/max=${formatNumber(call.mean_ms, 3)}/`
    + `${formatNumber(call.p95_ms, 3)}/`
    + `${formatNumber(call.p99_ms, 3)}/`
    + `${formatNumber(call.max_ms, 3)} ms`;
}

function formatFilter(filter) {
  if (!filter) return "UNAVAILABLE";
  if (filter.mode === "LEGACY_FORESIGHT") {
    return `LEGACY_FORESIGHT max_buf=${filter.legacy_max_buf}, kp=${filter.legacy_kp}`;
  }
  if (filter.mode === "LPF") return `LPF cutoff=${filter.lpf_cutoff_hz} Hz`;
  if (filter.mode === "NLF") {
    return `NLF v/a/j=${filter.nlf_max_velocity_deg_s}/`
      + `${filter.nlf_max_acceleration_deg_s2}/`
      + `${filter.nlf_max_jerk_deg_s3} deg units`;
  }
  return filter.mode || "LEGACY_FORESIGHT";
}

function executionMatchesFrozenArtifact(execution, artifact) {
  const executionGeneration = Number(execution.artifact_generation);
  const artifactGeneration = Number(artifact.generation);
  return Boolean(
    execution.trajectory_id
    && artifact.available === true
    && execution.artifact_fingerprint
    && execution.artifact_fingerprint === artifact.artifact_fingerprint
    && execution.plan_fingerprint
    && execution.plan_fingerprint === artifact.plan_fingerprint
    && execution.artifact_generation !== null
    && execution.artifact_generation !== undefined
    && artifact.generation !== null
    && artifact.generation !== undefined
    && Number.isFinite(executionGeneration)
    && Number.isFinite(artifactGeneration)
    && executionGeneration === artifactGeneration
  );
}

function hideExecutionTargetGhost(digitalTwin) {
  if (!digitalTwin) return;
  if (typeof digitalTwin.hideExecutionWaypointTarget === "function") {
    digitalTwin.hideExecutionWaypointTarget();
  } else if (typeof digitalTwin.hidePlannedModel === "function") {
    digitalTwin.hidePlannedModel();
  }
  state.executionGhostHidden = true;
}

function resetExecutionTargetCursor() {
  state.executionTargetCursor = null;
}

function executionTargetIdentity(execution) {
  return [
    execution.trajectory_id,
    execution.artifact_fingerprint,
    execution.plan_fingerprint,
    execution.artifact_generation,
  ].map((value) => String(value)).join("|");
}

function updateExecutionWaypointTarget(execution, feedback, artifact) {
  const digitalTwin = window.dualArmDigitalTwin;
  const feedbackState = feedback.combined_state || execution.state;
  const terminal = TERMINAL_GHOST_HIDE_STATES.has(execution.state)
    || TERMINAL_GHOST_HIDE_STATES.has(feedbackState);
  if (terminal && state.manualPreviewOverride) return;
  if (
    terminal
    || execution.state === "ABORT_REQUESTED"
  ) {
    resetExecutionTargetCursor();
    hideExecutionTargetGhost(digitalTwin);
    return;
  }
  const ownsCurrentArtifact = executionMatchesFrozenArtifact(execution, artifact);
  if (
    !execution.trajectory_id
    || feedback.authoritative !== true
    || !ownsCurrentArtifact
    || !currentFrozenPreviewLoaded()
  ) {
    resetExecutionTargetCursor();
    // A stale/previous execution must never leave its target visible. Manual
    // preview remains operator-owned when there is simply no active execution.
    if (execution.trajectory_id || !state.manualPreviewOverride) {
      hideExecutionTargetGhost(digitalTwin);
    }
    return;
  }
  if (!EXECUTION_TARGET_STATES.has(execution.state)) {
    resetExecutionTargetCursor();
    if (!state.manualPreviewOverride) hideExecutionTargetGhost(digitalTwin);
    return;
  }
  state.manualPreviewOverride = false;

  if (!digitalTwin) {
    resetExecutionTargetCursor();
    hideExecutionTargetGhost(digitalTwin);
    return;
  }
  try {
    if (
      typeof digitalTwin.getObjectGlobalPlanState !== "function"
      || typeof digitalTwin.getExecutionGateState !== "function"
      || typeof digitalTwin.getLatestActualPose !== "function"
      || typeof digitalTwin.getMirrorState !== "function"
      || typeof digitalTwin.setExecutionWaypointTarget !== "function"
    ) throw new Error("Integrated plan target renderer is unavailable");
    const planState = digitalTwin.getObjectGlobalPlanState();
    const gate = digitalTwin.getExecutionGateState();
    if (
      !planState
      || planState.status !== "READY"
      || !planState.plan
      || !gate
      || gate.currentPlanFingerprint !== execution.plan_fingerprint
      || gate.currentPlanFingerprint !== artifact.plan_fingerprint
    ) throw new Error("Current integrated plan identity does not match execution");
    const identity = executionTargetIdentity(execution);
    if (
      !state.executionTargetCursor
      || state.executionTargetCursor.executionIdentity !== identity
    ) {
      state.executionTargetCursor = createRigidWaypointTargetCursor(
        planState.plan, identity,
      );
    }
    const mirror = digitalTwin.getMirrorState();
    const actualPoseUsable = Boolean(
      mirror
      && mirror.enabled === true
      && mirror.hasValidSnapshot === true
      && mirror.mode !== "INVALID"
      && mirror.mode !== "STALE"
    );
    const actualPose = actualPoseUsable
      ? digitalTwin.getLatestActualPose() : null;
    const freshnessThresholdMs = Math.min(
      EXECUTION_GHOST_ACTUAL_FRESHNESS_MS,
      Number.isFinite(Number(mirror && mirror.staleTimeoutMs))
        ? Number(mirror.staleTimeoutMs) : EXECUTION_GHOST_ACTUAL_FRESHNESS_MS,
    );
    const target = updateRigidWaypointTargetCursor(
      planState.plan,
      state.executionTargetCursor,
      {
        executionIdentity: identity,
        actualPose,
        actualReceivedAtMs: mirror && mirror.lastAcceptedSnapshotMs,
        nowMs: Date.now(),
        freshnessThresholdMs,
      },
    );
    state.executionTargetCursor = target;
    // Fail closed visually when authoritative actual feedback is stale/missing.
    // Never fall back to elapsed execution time for waypoint advancement.
    if (target.actualFresh !== true || target.visible !== true) {
      hideExecutionTargetGhost(digitalTwin);
      return;
    }
    const result = digitalTwin.setExecutionWaypointTarget(target);
    if (!result || result.applied !== true) {
      throw new Error((result && result.reason) || "Waypoint target was rejected");
    }
    state.executionGhostHidden = false;
  } catch (_error) {
    // Any plan/sample/identity inconsistency fails closed visually. The frozen
    // trajectory itself stays loaded and unchanged for later manual review.
    resetExecutionTargetCursor();
    hideExecutionTargetGhost(digitalTwin);
  }
}

function applyStatus(payload) {
  const artifact = payload.artifact || {};
  const gate = payload.phase4_gate || {};
  const transport = payload.transport || {};
  const connection = transport.robot_connection || {};
  const safeState = payload.safe_state || {};
  const execution = payload.execution || {};
  const operatorAuthority = payload.operator_authority || {};
  const feedback = payload.driver_feedback || execution.driver_feedback || {};
  const sides = feedback.drivers || {};
  const commonTimeline = feedback.common_timeline || {};
  const actualJoints = payload.actual_joints || {};
  const startMatch = payload.start_match || {};
  const initialTarget = payload.initial_recovery_target || {};
  const stopGeneration = Number(payload.stop_generation);
  state.artifactFingerprint = artifact.available ? artifact.artifact_fingerprint : null;
  state.artifactGeneration = artifact.available ? Number(artifact.generation) : null;
  const frozenPreviewReady = currentFrozenPreviewReady();
  const executionMatchesArtifact = executionMatchesFrozenArtifact(execution, artifact);
  const previousArtifactExecution = Boolean(
    execution.trajectory_id && artifact.available === true && !executionMatchesArtifact,
  );
  const executionState = execution.state || "IDLE";
  const combinedResult = execution.state === "ABORT_REQUESTED"
    ? "ABORT_REQUESTED"
    : (feedback.authoritative === true
      ? (feedback.combined_state || executionState)
      : (feedback.feedback_status || executionState));
  const executionPrefix = previousArtifactExecution ? "PREVIOUS ARTIFACT — " : "";

  if (
    state.operatorAuthorityPrepared
    && (
      operatorAuthority.pending !== true
      || !frozenPreviewReady
      || (
        Number.isFinite(stopGeneration)
        && stopGeneration !== state.preparedStopGeneration
      )
    )
  ) {
    clearPendingOperatorAuthority(
      operatorAuthority.last_invalidation_reason || "OPERATOR AUTHORITY INVALIDATED",
    );
  }
  state.stopGeneration = Number.isFinite(stopGeneration) ? stopGeneration : null;

  setText(
    "digitalTwinPhase5ArtifactStatus",
    artifact.available ? `FROZEN (generation ${artifact.generation})` : "UNAVAILABLE",
  );
  setText(
    "digitalTwinPhase5ArtifactFingerprint",
    artifact.artifact_fingerprint || "NONE",
  );
  setText(
    "digitalTwinPhase5ArtifactGeneration",
    artifact.generation === undefined ? "NONE" : artifact.generation,
  );
  setText(
    "digitalTwinPhase5GateStatus",
    gate.execution_ready === true ? "READY" : (gate.status || "BLOCKED"),
  );
  setText(
    "digitalTwinPhase5TransportStatus",
    connection.both_ready === true ? "LEFT + RIGHT READY" : "NOT READY",
  );
  setText(
    "digitalTwinPhase5SafeState",
    safeState.ok === true ? "OK (BOTH)" : (safeState.message || "BLOCKED"),
  );
  setText(
    "digitalTwinPhase5StartMatch",
    startMatch.match === true ? "READY" : (startMatch.state || "BLOCKED"),
  );
  setText("digitalTwinPhase5ExecutionState", `${executionPrefix}${executionState}`);
  setText(
    "digitalTwinPhase5TrajectoryId",
    `${executionPrefix}${execution.trajectory_id || "NONE"}`,
  );
  setText(
    "digitalTwinPhase5CombinedResult",
    `${executionPrefix}${combinedResult}`,
  );
  setText(
    "digitalTwinPhase5CombinedReason",
    `${executionPrefix}${execution.reason || feedback.reason || "NONE"}`,
  );
  const elapsed = finiteNumber(commonTimeline.elapsed_s);
  const duration = finiteNumber(commonTimeline.duration_s) === null
    ? finiteNumber(execution.duration_s) : finiteNumber(commonTimeline.duration_s);
  setText(
    "digitalTwinPhase5CommonTimeline",
    `${executionPrefix}${(elapsed === null ? 0 : elapsed).toFixed(3)} / `
      + `${(duration === null ? 0 : duration).toFixed(3)} s`,
  );
  renderCommonProgress(commonTimeline.progress_0_to_1, previousArtifactExecution);
  renderDriverSide("left", sides.left, previousArtifactExecution);
  renderDriverSide("right", sides.right, previousArtifactExecution);
  setText("digitalTwinPhase5StartMaxDelta", formatNumber(startMatch.max_delta_rad));
  setText("digitalTwinPhase5StartThreshold", formatNumber(startMatch.threshold_rad));
  setText("digitalTwinPhase5StartReason", startMatch.reason || "UNAVAILABLE");
  setText(
    "digitalTwinPhase5StartJointDeltas",
    startMatch.joint_deltas_rad ? JSON.stringify(startMatch.joint_deltas_rad) : "UNAVAILABLE",
  );
  setText("digitalTwinPhase5LeftQuality", formatQuality(sides.left));
  setText("digitalTwinPhase5RightQuality", formatQuality(sides.right));
  setText("digitalTwinPhase5LeftTiming", formatTiming(sides.left));
  setText("digitalTwinPhase5RightTiming", formatTiming(sides.right));
  const configuredFilter = ((payload.motion_quality_configuration || {}).servo_filter);
  setText(
    "digitalTwinPhase5Filter",
    formatFilter((sides.left && sides.left.servo_filter) || configuredFilter),
  );
  setText("digitalTwinPhase5ActualLeftJoints", formatJointArray(actualJoints.left));
  setText(
    "digitalTwinPhase5ActualLeftJointStatus", formatJointCache(actualJoints.left),
  );
  setText("digitalTwinPhase5ActualRightJoints", formatJointArray(actualJoints.right));
  setText(
    "digitalTwinPhase5ActualRightJointStatus", formatJointCache(actualJoints.right),
  );
  renderPhase6JointTracking(payload.phase6_joint_tracking);
  renderReusedActualTcp();
  updateExecutionWaypointTarget(execution, feedback, artifact);
  setText(
    "digitalTwinPhase5StopGeneration",
    Number.isFinite(stopGeneration) ? stopGeneration : "UNAVAILABLE",
  );
  setText(
    "digitalTwinPhase5InitialTarget",
    initialTarget.available === true
      ? `${initialTarget.source || "AVAILABLE"}`
        + `${initialTarget.trajectory_name ? ` — ${initialTarget.trajectory_name}` : ""}`
      : "UNAVAILABLE",
  );
  const hasBlockingReasons = (
    Array.isArray(payload.blocking_reasons) && payload.blocking_reasons.length > 0
  );
  setText(
    "digitalTwinPhase5BlockingReasons",
    hasBlockingReasons ? payload.blocking_reasons.join(", ") : "NONE",
  );
  const blockerSummary = element("digitalTwinPhase5BlockingReasons")?.closest(
    ".digital-twin-phase5-blockers",
  );
  if (blockerSummary) blockerSummary.dataset.blocked = hasBlockingReasons ? "true" : "false";
  setText(
    "digitalTwinPhase5Error",
    `${executionPrefix}`
      + `${execution.reason || execution.trajectory_id || payload.semantic || "NONE"}`,
  );
  setText(
    "digitalTwinPhase5CommonStart",
    payload.semantic || "HOST-TIMED — NOT HARD REAL-TIME",
  );
  renderFrozenPreviewState(execution.state || "IDLE");

  const prepareButton = element("digitalTwinPhase5Prepare");
  if (prepareButton) {
    prepareButton.disabled = state.requestInFlight
      || !frozenPreviewReady
      || payload.ready_to_prepare !== true
      || ACTIVE_STATES.has(execution.state);
  }
  const replanButton = element("digitalTwinPhase5ReplanFromCurrent");
  if (replanButton) {
    replanButton.disabled = state.requestInFlight
      || payload.ready_to_replan_from_current !== true
      || ACTIVE_STATES.has(execution.state);
  }
  const moveButton = element("digitalTwinPhase5MoveToInitial");
  if (moveButton) {
    moveButton.disabled = state.requestInFlight
      || payload.ready_to_move_to_initial !== true
      || ACTIVE_STATES.has(execution.state);
  }
}

async function refreshPhase5ExecutionStatus() {
  if (state.statusRequestInFlight) return;
  state.statusRequestInFlight = true;
  try {
    const payload = await requestJson(STATUS_ENDPOINT);
    await ensureFrozenArtifactPreview(payload.artifact || {});
    applyStatus(payload);
  } catch (error) {
    setText(
      "digitalTwinPhase5Error",
      error && error.message ? error.message : "STATUS UNAVAILABLE",
    );
    const prepareButton = element("digitalTwinPhase5Prepare");
    if (prepareButton) prepareButton.disabled = true;
  } finally {
    state.statusRequestInFlight = false;
  }
}

async function executePhase5RealMotion() {
  if (state.requestInFlight) return;
  if (!currentFrozenPreviewReady()) {
    setText("digitalTwinPhase5Error", "EXECUTE BLOCKED — PREVIEW THE CURRENT FROZEN GHOST FIRST");
    return;
  }
  state.requestInFlight = true;
  clearPendingOperatorAuthority("PREPARING NO-MOTION AUTHORITY…");
  try {
    const prepared = await requestJson(PREPARE_ENDPOINT, { method: "POST" });
    if (prepared.ok !== true || prepared.motion_dispatched !== false) {
      throw new Error(errorText(prepared, "PREPARE FAILED CLOSED"));
    }
    state.operatorAuthorityPrepared = true;
    state.preparedStopGeneration = Number((prepared.authority || {}).stop_generation);
    const trajectory = prepared.trajectory || {};
    setOperatorAction(
      `PREPARED — ${trajectory.name || "trajectory"}: `
        + `${trajectory.sample_count || "?"} samples, ${trajectory.duration_s || "?"} s`,
    );
    setText("digitalTwinPhase5Error", "PREPARED — NO MOTION DISPATCHED");
    const confirmed = window.confirm(
      "REAL MOTION: Execute the verified frozen dual-arm trajectory on both real robots?\n\n"
        + "Clear the workspace and keep STOP / E-stop ready. OK submits the frozen "
        + "trajectory using the shared host-timed start.",
    );
    if (!confirmed) {
      clearPendingOperatorAuthority("OPERATOR CANCELLED — NO MOTION");
      setText("digitalTwinPhase5Error", "OPERATOR CANCELLED — NO MOTION DISPATCHED");
      return;
    }

    clearPendingOperatorAuthority("OPERATOR CONFIRMED — AUTHORITY CONSUMED");
    setText("digitalTwinPhase5Error", "SUBMITTING BOTH TRAJECTORIES…");
    const executed = await requestJson(EXECUTE_ENDPOINT, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ operator_confirmed: true }),
    });
    if (executed.ok !== true || executed.accepted !== true) {
      throw new Error(errorText(executed, "EXECUTION REJECTED"));
    }
    const execution = executed.execution || {};
    setText("digitalTwinPhase5ExecutionState", execution.state || "ARMED");
    setText(
      "digitalTwinPhase5Error",
      execution.trajectory_id || "BOTH TRAJECTORIES ACCEPTED",
    );
  } catch (error) {
    clearPendingOperatorAuthority("EXECUTION REQUEST FAILED CLOSED");
    setText(
      "digitalTwinPhase5Error",
      error && error.message ? error.message : "PREPARE / EXECUTION FAILED CLOSED",
    );
  } finally {
    state.requestInFlight = false;
    await refreshPhase5ExecutionStatus();
  }
}

async function replanFromCurrent() {
  if (state.requestInFlight) return;
  state.requestInFlight = true;
  setOperatorAction("REPLAN: CAPTURING CURRENT ACTUAL JOINTS — NO MOTION");
  try {
    const digitalTwin = window.dualArmDigitalTwin;
    if (!digitalTwin || typeof digitalTwin.setNextPlanningStartOverride !== "function") {
      throw new Error("Digital Twin planner API is unavailable");
    }
    if (typeof digitalTwin.getObjectWaypointPlanningState !== "function") {
      throw new Error("Object Planning Waypoint state API is unavailable");
    }
    const waypointState = digitalTwin.getObjectWaypointPlanningState();
    const waypointCount = waypointState && Array.isArray(waypointState.waypoints)
      ? waypointState.waypoints.length : 0;
    if (waypointCount < 2) {
      throw new Error(
        `OBJECT PLANNING WAYPOINTS REQUIRED — found ${waypointCount}; `
        + "add at least 2 points in the Object Planning Waypoints panel",
      );
    }
    setOperatorAction(`REPLAN: ${waypointCount} OBJECT WAYPOINTS FOUND — CAPTURING ACTUAL`);
    const payload = await requestJson(REPLAN_FROM_CURRENT_ENDPOINT, { method: "POST" });
    if (payload.ok !== true || payload.motion_dispatched !== false) {
      throw new Error(errorText(payload, "REPLAN FROM CURRENT BLOCKED"));
    }
    digitalTwin.setNextPlanningStartOverride(
      payload.initial_joint_state_rad,
      payload.planning_start_state_source,
    );
    setText("digitalTwinPhase5ReplanState", "PLANNING FROM CURRENT ACTUAL — NO MOTION");
    setText(
      "digitalTwinGlobalPlanPlanningStartSource",
      `${payload.planning_start_state_source} (CURRENT REPLAN)`,
    );
    if (typeof digitalTwin.planObjectGlobal !== "function") {
      throw new Error("Global planner public API is unavailable");
    }
    setOperatorAction(`REPLAN: PLANNING ${waypointCount} OBJECT WAYPOINTS — NO MOTION`);
    const planState = await digitalTwin.planObjectGlobal();
    if (!planState || planState.status !== "READY" || !planState.plan) {
      throw new Error((planState && planState.error) || "REPLAN FROM CURRENT GLOBAL PLAN FAILED");
    }
    setText("digitalTwinPhase5ReplanState", "VALIDATING NEW CURRENT-START PLAN — NO MOTION");
    setOperatorAction("REPLAN: GLOBAL PLAN READY — RUNNING PHASE-4 VALIDATION");
    if (typeof digitalTwin.validateCurrentGlobalPlanPhase4Unified !== "function") {
      throw new Error("Phase-4 Unified validation public API is unavailable");
    }
    const validationState = await digitalTwin.validateCurrentGlobalPlanPhase4Unified();
    if (
      !validationState
      || validationState.status !== "PASS"
      || !validationState.report
      || validationState.report.overall_status !== "PASS"
    ) {
      throw new Error(
        (validationState && validationState.error)
          || "REPLAN FROM CURRENT PHASE-4 VALIDATION FAILED",
      );
    }
    setText(
      "digitalTwinPhase5ReplanState",
      "READY — REPLANNED/VALIDATED/FROZEN FROM CURRENT ACTUAL",
    );
    setText(
      "digitalTwinPhase5Error",
      "REPLAN FROM CURRENT COMPLETE — NO MOTION; REVIEW THE NEW FROZEN GHOST",
    );
    setOperatorAction("REPLAN READY — VALIDATED/FROZEN FROM CURRENT ACTUAL — REVIEW GHOST");
  } catch (error) {
    const message = error && error.message ? error.message : "REPLAN FROM CURRENT BLOCKED";
    setText("digitalTwinPhase5ReplanState", "BLOCKED");
    setText("digitalTwinPhase5Error", message);
    setOperatorAction(`REPLAN FAILED — ${message}`);
  } finally {
    state.requestInFlight = false;
    await refreshPhase5ExecutionStatus();
    setText("digitalTwinPhase5OperatorAction", state.lastOperatorAction);
  }
}

async function moveToInitial() {
  if (state.requestInFlight) return;
  const confirmed = window.confirm(
    "Move both real robots to Frozen Artifact sample 0 using the conservative "
      + "Phase-5 recovery joint_move settings? This is REAL MOTION.",
  );
  if (!confirmed) return;
  state.requestInFlight = true;
  clearPendingOperatorAuthority("MOVE TO INITIAL REQUESTED");
  try {
    const payload = await requestJson(MOVE_TO_INITIAL_ENDPOINT, { method: "POST" });
    if (payload.ok !== true) {
      throw new Error(errorText(payload, "MOVE TO INITIAL REJECTED"));
    }
    setText(
      "digitalTwinPhase5Error",
      payload.accepted === false
        ? (payload.message || "ALREADY AT INITIAL")
        : "MOVE TO INITIAL ACCEPTED — MONITOR LIVE START MATCH",
    );
  } catch (error) {
    setText(
      "digitalTwinPhase5Error",
      error && error.message ? error.message : "MOVE TO INITIAL REJECTED",
    );
  } finally {
    state.requestInFlight = false;
    await refreshPhase5ExecutionStatus();
  }
}

export function bindPhase5ExecutionControls() {
  if (controlsBound) return;
  if (!element("digitalTwinPhase5ExecutionSection")) return;
  controlsBound = true;

  const refreshButton = element("digitalTwinPhase5Refresh");
  const prepareButton = element("digitalTwinPhase5Prepare");
  const previewReload = element("digitalTwinPhase5PreviewReload");
  const previewPlay = element("digitalTwinPhase5PreviewPlay");
  const previewReset = element("digitalTwinPhase5PreviewReset");
  const replanButton = element("digitalTwinPhase5ReplanFromCurrent");
  const moveButton = element("digitalTwinPhase5MoveToInitial");
  if (refreshButton) refreshButton.addEventListener("click", refreshPhase5ExecutionStatus);
  if (previewReload) previewReload.addEventListener("click", async () => {
    await ensureFrozenArtifactPreview({
      available: Boolean(state.artifactFingerprint),
      artifact_fingerprint: state.artifactFingerprint,
      generation: state.artifactGeneration,
    }, { force: true });
    await refreshPhase5ExecutionStatus();
  });
  if (previewPlay) previewPlay.addEventListener("click", playFrozenArtifactPreview);
  if (previewReset) previewReset.addEventListener("click", resetFrozenArtifactPreview);
  if (prepareButton) prepareButton.addEventListener("click", executePhase5RealMotion);
  if (replanButton) replanButton.addEventListener("click", replanFromCurrent);
  if (moveButton) moveButton.addEventListener("click", moveToInitial);
  window.addEventListener("dual-arm-phase5-authority-invalidated", (event) => {
    const reason = event && event.detail ? event.detail.reason : "AUTHORITY INVALIDATED";
    clearPendingOperatorAuthority(reason);
  });
  // Automatic work is read-only GET status + Frozen Ghost preview only;
  // all POST endpoints remain explicit click-handler-only actions.
  refreshPhase5ExecutionStatus();
  statusPollId = window.setInterval(refreshPhase5ExecutionStatus, STATUS_POLL_INTERVAL_MS);
}

window.dualArmPhase5Execution = {
  refreshStatus: refreshPhase5ExecutionStatus,
  reloadFrozenPreview: () => ensureFrozenArtifactPreview({
    available: Boolean(state.artifactFingerprint),
    artifact_fingerprint: state.artifactFingerprint,
    generation: state.artifactGeneration,
  }, { force: true }),
  previewReady: currentFrozenPreviewReady,
  invalidatePendingAuthority: clearPendingOperatorAuthority,
};

function bootstrapPhase5ExecutionControls() {
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", bindPhase5ExecutionControls, { once: true });
    return;
  }
  bindPhase5ExecutionControls();
}

// Keep Phase-5 operator controls alive even if another Digital Twin subsystem
// fails later during the main initialize() sequence. This bootstrap performs
// read-only status polling only; all POST actions remain explicit button clicks.
bootstrapPhase5ExecutionControls();
