// Phase 1 cache-only RobotMsg and JointState freshness source. It is isolated
// from the joint mirror so a status failure cannot stop model rendering.
import {
  deriveRobotStateAndAlert,
  feedbackConnectionStatus,
} from "./digital_twin_phase1_state.js";

const ROBOT_STATUS_ENDPOINT = "/api/digital-twin/robot-status";
const DEFAULT_STATUS_POLL_INTERVAL_MS = 500;
const FETCH_TIMEOUT_MS = 1000;
const INITIALIZATION_FLAG = "__dualArmDigitalTwinRobotStatusSourceInitialized";
const FEEDBACK_STATES = new Set(["LIVE", "STALE", "MISSING", "INVALID"]);
const STATE_FIELDS = [
  "power_state",
  "servo_state",
  "motion_state",
  "collision_state",
];

const sourceState = {
  running: false,
  status: "STOPPED",
  lastFetchMs: null,
  lastFetchError: null,
  latestSnapshot: null,
};
let pollTimerId = null;
let activeAbortController = null;
let activePollPromise = null;

function setText(id, value) {
  const element = document.getElementById(id);
  if (element) element.textContent = value;
}

function normalizeSide(side, label) {
  if (!side || !FEEDBACK_STATES.has(side.feedback_status)) {
    throw new Error(`${label} feedback status is invalid`);
  }
  const state = side.state;
  if (side.robot_state_valid === true) {
    if (!state || typeof state !== "object" || Array.isArray(state)) {
      throw new Error(`${label} robot state is invalid`);
    }
    for (const field of STATE_FIELDS) {
      if (!Number.isInteger(state[field])) {
        throw new Error(`${label} ${field} is invalid`);
      }
    }
  }
  return {
    feedbackStatus: side.feedback_status,
    robotStateValid: side.robot_state_valid === true,
    jointFeedbackValid: side.joint_feedback_valid === true,
    state: side.robot_state_valid === true
      ? Object.fromEntries(STATE_FIELDS.map((field) => [field, state[field]]))
      : null,
  };
}

function normalizeRobotStatus(payload) {
  if (!payload || payload.cache_only !== true) {
    throw new Error("Robot status response is not cache-only");
  }
  return {
    left: normalizeSide(payload.left, "left"),
    right: normalizeSide(payload.right, "right"),
  };
}

function renderSide(sideName, side) {
  const title = sideName[0].toUpperCase() + sideName.slice(1);
  setText(`digitalTwin${title}FeedbackStatus`, side.feedbackStatus);
  let connectionStatus = "MISSING";
  let summary = { robotState: "UNAVAILABLE", faultAlert: "FEEDBACK MISSING" };
  try {
    connectionStatus = feedbackConnectionStatus(side.feedbackStatus);
    summary = deriveRobotStateAndAlert(side);
  } catch (_error) {
    connectionStatus = "INVALID";
    summary = { robotState: "UNAVAILABLE", faultAlert: "INVALID FEEDBACK" };
  }
  setText(`digitalTwin${title}ConnectionStatus`, connectionStatus);
  setText(`digitalTwin${title}RobotStateSummary`, summary.robotState);
  setText(`digitalTwin${title}FaultAlert`, summary.faultAlert);
  for (const field of STATE_FIELDS) {
    const suffix = field.split("_").map(
      (part) => part[0].toUpperCase() + part.slice(1),
    ).join("");
    setText(
      `digitalTwin${title}${suffix}`,
      side.state ? String(side.state[field]) : "UNAVAILABLE",
    );
  }
}

function render() {
  setText("digitalTwinRobotStatusSourceState", sourceState.status);
  setText("digitalTwinRobotStatusError", sourceState.lastFetchError || "NONE");
  if (sourceState.latestSnapshot) {
    renderSide("left", sourceState.latestSnapshot.left);
    renderSide("right", sourceState.latestSnapshot.right);
  } else {
    for (const sideName of ["left", "right"]) {
      renderSide(sideName, {
        feedbackStatus: "MISSING",
        state: null,
      });
    }
  }
}

function getRobotStatusSourceState() {
  return {
    running: sourceState.running,
    status: sourceState.status,
    lastFetchMs: sourceState.lastFetchMs,
    lastFetchError: sourceState.lastFetchError,
    latestSnapshot: sourceState.latestSnapshot
      ? {
          left: {
            ...sourceState.latestSnapshot.left,
            state: sourceState.latestSnapshot.left.state
              ? { ...sourceState.latestSnapshot.left.state }
              : null,
          },
          right: {
            ...sourceState.latestSnapshot.right,
            state: sourceState.latestSnapshot.right.state
              ? { ...sourceState.latestSnapshot.right.state }
              : null,
          },
        }
      : null,
  };
}

async function executePoll() {
  const controller = new AbortController();
  activeAbortController = controller;
  const timeoutId = window.setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);
  try {
    const response = await fetch(ROBOT_STATUS_ENDPOINT, {
      method: "GET",
      cache: "no-store",
      signal: controller.signal,
    });
    sourceState.lastFetchMs = Date.now();
    if (!response.ok) {
      throw new Error(`Robot status request failed with HTTP ${response.status}`);
    }
    sourceState.latestSnapshot = normalizeRobotStatus(await response.json());
    const digitalTwin = window.dualArmDigitalTwin;
    if (digitalTwin && typeof digitalTwin.setModelTcpFeedbackStatus === "function") {
      const nonLiveStatuses = {};
      for (const side of ["left", "right"]) {
        const status = sourceState.latestSnapshot[side].feedbackStatus;
        if (status !== "LIVE") nonLiveStatuses[side] = status;
      }
      if (Object.keys(nonLiveStatuses).length > 0) {
        digitalTwin.setModelTcpFeedbackStatus(nonLiveStatuses);
      }
    }
    sourceState.lastFetchError = null;
    sourceState.status = sourceState.running ? "RUNNING — CACHE ONLY" : "STOPPED";
  } catch (error) {
    sourceState.latestSnapshot = null;
    if (error && error.name === "AbortError" && !sourceState.running) {
      sourceState.lastFetchError = null;
      sourceState.status = "STOPPED";
    } else {
      sourceState.lastFetchError = error && error.message
        ? error.message
        : "Unknown robot status error";
      sourceState.status = sourceState.running ? "ERROR — CACHE ONLY" : "STOPPED";
    }
  } finally {
    window.clearTimeout(timeoutId);
    if (activeAbortController === controller) activeAbortController = null;
    render();
  }
  return getRobotStatusSourceState();
}

function pollRobotStatusOnce() {
  if (activePollPromise) return activePollPromise;
  activePollPromise = executePoll().finally(() => { activePollPromise = null; });
  return activePollPromise;
}

function scheduleNextPoll() {
  if (!sourceState.running || pollTimerId !== null) return;
  pollTimerId = window.setTimeout(async () => {
    pollTimerId = null;
    await pollRobotStatusOnce();
    scheduleNextPoll();
  }, DEFAULT_STATUS_POLL_INTERVAL_MS);
}

function startRobotStatusSource() {
  if (sourceState.running) return getRobotStatusSourceState();
  sourceState.running = true;
  sourceState.status = "STARTING — CACHE ONLY";
  sourceState.lastFetchError = null;
  render();
  void pollRobotStatusOnce().finally(scheduleNextPoll);
  return getRobotStatusSourceState();
}

function stopRobotStatusSource() {
  sourceState.running = false;
  sourceState.status = "STOPPED";
  sourceState.latestSnapshot = null;
  if (pollTimerId !== null) {
    window.clearTimeout(pollTimerId);
    pollTimerId = null;
  }
  if (activeAbortController) activeAbortController.abort();
  render();
  return getRobotStatusSourceState();
}

export {
  getRobotStatusSourceState,
  normalizeRobotStatus,
  pollRobotStatusOnce,
  startRobotStatusSource,
  stopRobotStatusSource,
};

if (!window[INITIALIZATION_FLAG]) {
  window[INITIALIZATION_FLAG] = true;
  window.dualArmDigitalTwinRobotStatusSource = {
    getRobotStatusSourceState,
    pollRobotStatusOnce,
    startRobotStatusSource,
    stopRobotStatusSource,
  };
  const startButton = document.getElementById("digitalTwinStartLiveFeedback");
  const stopButton = document.getElementById("digitalTwinStopLiveFeedback");
  if (startButton) startButton.addEventListener("click", startRobotStatusSource);
  if (stopButton) stopButton.addEventListener("click", stopRobotStatusSource);
  render();
}
