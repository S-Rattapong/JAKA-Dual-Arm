// Phase 1 read-only FK source. The returned pose remains in each robot base
// frame and is not placed in Web world until Base-to-World is verified.
const TCP_ENDPOINT = "/api/digital-twin/tcp";
const DEFAULT_TCP_POLL_INTERVAL_MS = 500;
const FETCH_TIMEOUT_MS = 1200;
const INITIALIZATION_FLAG = "__dualArmDigitalTwinTcpSourceInitialized";
const TCP_PLACEMENT_BLOCKER =
  "TCP FRAME PLACEMENT BLOCKED PENDING VERIFIED BASE↔WORLD CONVERSION";

const sourceState = {
  running: false,
  status: "STOPPED",
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

function normalizeTcpSide(side, label) {
  if (!side || typeof side.valid !== "boolean") {
    throw new Error(`${label} TCP status is invalid`);
  }
  if (!side.valid) {
    return { valid: false, tcp: null, error: side.error || "FK unavailable" };
  }
  if (
    !Array.isArray(side.tcp) ||
    side.tcp.length !== 6 ||
    !side.tcp.every(
      (value) => typeof value === "number" && Number.isFinite(value),
    )
  ) {
    throw new Error(`${label} TCP pose is invalid`);
  }
  return { valid: true, tcp: [...side.tcp], error: null };
}

function normalizeTcpSnapshot(payload) {
  if (
    !payload ||
    payload.source !== "jaka_get_fk_from_current_joint_feedback" ||
    payload.cache_only !== false ||
    !payload.units ||
    payload.units.translation !== "millimeter" ||
    payload.units.orientation !== "radian" ||
    payload.frame !== "jaka_controller_current_user_coordinate" ||
    payload.orientation_convention !== "JAKA CartesianPose RPY rx_ry_rz"
  ) {
    throw new Error("TCP response source is invalid");
  }
  return {
    source: payload.source,
    units: { ...payload.units },
    frame: payload.frame,
    orientationConvention: payload.orientation_convention,
    left: normalizeTcpSide(payload.left, "left"),
    right: normalizeTcpSide(payload.right, "right"),
  };
}

function render() {
  setText("digitalTwinTcpSourceState", sourceState.status);
  setText("digitalTwinTcpSourceError", sourceState.lastFetchError || "NONE");
  setText("digitalTwinTcpPlacementState", TCP_PLACEMENT_BLOCKER);
  if (sourceState.latestSnapshot) {
    setText(
      "digitalTwinLeftTcpStatus",
      sourceState.latestSnapshot.left.valid ? "LIVE" : "UNAVAILABLE",
    );
    setText(
      "digitalTwinRightTcpStatus",
      sourceState.latestSnapshot.right.valid ? "LIVE" : "UNAVAILABLE",
    );
  }
}

function getTcpSourceState() {
  return {
    running: sourceState.running,
    status: sourceState.status,
    lastFetchError: sourceState.lastFetchError,
    latestSnapshot: sourceState.latestSnapshot
      ? {
          ...sourceState.latestSnapshot,
          units: { ...sourceState.latestSnapshot.units },
          left: {
            ...sourceState.latestSnapshot.left,
            tcp: sourceState.latestSnapshot.left.tcp
              ? [...sourceState.latestSnapshot.left.tcp]
              : null,
          },
          right: {
            ...sourceState.latestSnapshot.right,
            tcp: sourceState.latestSnapshot.right.tcp
              ? [...sourceState.latestSnapshot.right.tcp]
              : null,
          },
        }
      : null,
    framePlacement: "BLOCKED",
  };
}

async function executePoll() {
  const controller = new AbortController();
  activeAbortController = controller;
  const timeoutId = window.setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);
  try {
    const response = await fetch(TCP_ENDPOINT, {
      method: "GET",
      cache: "no-store",
      signal: controller.signal,
    });
    if (!response.ok) {
      throw new Error(`TCP request failed with HTTP ${response.status}`);
    }
    sourceState.latestSnapshot = normalizeTcpSnapshot(await response.json());
    sourceState.lastFetchError = null;
    sourceState.status = sourceState.running ? "RUNNING — READ ONLY FK" : "STOPPED";
  } catch (error) {
    // There are deliberately no TCP frame objects to leave frozen in the scene.
    sourceState.latestSnapshot = null;
    if (error && error.name === "AbortError" && !sourceState.running) {
      sourceState.lastFetchError = null;
      sourceState.status = "STOPPED";
    } else {
      sourceState.lastFetchError = error && error.message
        ? error.message
        : "Unknown TCP source error";
      sourceState.status = sourceState.running ? "ERROR — READ ONLY FK" : "STOPPED";
    }
  } finally {
    window.clearTimeout(timeoutId);
    if (activeAbortController === controller) activeAbortController = null;
    if (!sourceState.latestSnapshot) {
      setText("digitalTwinLeftTcpStatus", "UNAVAILABLE");
      setText("digitalTwinRightTcpStatus", "UNAVAILABLE");
    }
    render();
  }
  return getTcpSourceState();
}

function pollTcpOnce() {
  if (activePollPromise) return activePollPromise;
  activePollPromise = executePoll().finally(() => { activePollPromise = null; });
  return activePollPromise;
}

function scheduleNextPoll() {
  if (!sourceState.running || pollTimerId !== null) return;
  pollTimerId = window.setTimeout(async () => {
    pollTimerId = null;
    await pollTcpOnce();
    scheduleNextPoll();
  }, DEFAULT_TCP_POLL_INTERVAL_MS);
}

function startTcpSource() {
  if (sourceState.running) return getTcpSourceState();
  sourceState.running = true;
  sourceState.status = "STARTING — READ ONLY FK";
  sourceState.lastFetchError = null;
  render();
  void pollTcpOnce().finally(scheduleNextPoll);
  return getTcpSourceState();
}

function stopTcpSource() {
  sourceState.running = false;
  sourceState.status = "STOPPED";
  sourceState.latestSnapshot = null;
  if (pollTimerId !== null) {
    window.clearTimeout(pollTimerId);
    pollTimerId = null;
  }
  if (activeAbortController) activeAbortController.abort();
  setText("digitalTwinLeftTcpStatus", "UNAVAILABLE");
  setText("digitalTwinRightTcpStatus", "UNAVAILABLE");
  render();
  return getTcpSourceState();
}

export {
  TCP_PLACEMENT_BLOCKER,
  getTcpSourceState,
  normalizeTcpSnapshot,
  pollTcpOnce,
  startTcpSource,
  stopTcpSource,
};

if (!window[INITIALIZATION_FLAG]) {
  window[INITIALIZATION_FLAG] = true;
  window.dualArmDigitalTwinTcpSource = {
    getTcpSourceState,
    pollTcpOnce,
    startTcpSource,
    stopTcpSource,
  };
  const startButton = document.getElementById("digitalTwinStartLiveFeedback");
  const stopButton = document.getElementById("digitalTwinStopLiveFeedback");
  if (startButton) startButton.addEventListener("click", startTcpSource);
  if (stopButton) stopButton.addEventListener("click", stopTcpSource);
  render();
}
