// Phase 1C.2A read-only source. This module polls joint feedback only and
// deliberately contains no robot command integration.
const JOINT_FEEDBACK_ENDPOINT = "/api/digital-twin/joints";
const DEFAULT_POLL_INTERVAL_MS = 10;
const FETCH_TIMEOUT_MS = 1000;
const INITIALIZATION_FLAG = "__dualArmDigitalTwinLiveSourceInitialized";

const liveState = {
  running: false,
  status: "STOPPED",
  pollIntervalMs: DEFAULT_POLL_INTERVAL_MS,
  lastFetchMs: null,
  lastFeedbackTimestampMs: null,
  endpointSource: null,
  sourceDiagnostics: null,
  lastFetchError: null,
};

let pollTimerId = null;
let activeAbortController = null;
let activePollPromise = null;

function formatTimestamp(timestampMs) {
  if (typeof timestampMs !== "number" || !Number.isFinite(timestampMs)) {
    return "NEVER";
  }
  const date = new Date(timestampMs);
  return Number.isFinite(date.getTime()) ? date.toISOString() : "INVALID";
}

function setTextById(id, value) {
  const element = document.getElementById(id);
  if (element) element.textContent = value;
}

function getMirrorEnabledState() {
  const digitalTwin = window.dualArmDigitalTwin;
  if (!digitalTwin || typeof digitalTwin.getMirrorState !== "function") return null;
  try {
    return digitalTwin.getMirrorState().enabled === true;
  } catch (_error) {
    return null;
  }
}

function updateUnifiedLiveMirrorUi() {
  const switchElement = document.getElementById("digitalTwinLiveMirrorSwitch");
  const quickPanel = document.getElementById("digitalTwinLiveMirrorQuick");
  const mirrorEnabled = getMirrorEnabledState();
  const mirrorKnown = typeof mirrorEnabled === "boolean";
  const fullyEnabled = liveState.running && (mirrorKnown ? mirrorEnabled : true);
  const fullyDisabled = !liveState.running && (mirrorKnown ? !mirrorEnabled : true);
  const partial = !fullyEnabled && !fullyDisabled;

  if (switchElement) {
    switchElement.checked = fullyEnabled;
    switchElement.indeterminate = partial;
    if (typeof switchElement.setAttribute === "function") {
      switchElement.setAttribute(
        "aria-checked",
        partial ? "mixed" : (fullyEnabled ? "true" : "false"),
      );
    }
  }

  let status = "OFF — FEEDBACK + MIRROR STOPPED";
  let state = "off";
  if (liveState.running && liveState.lastFetchError) {
    status = "ON — LIVE FEEDBACK ERROR";
    state = "error";
  } else if (fullyEnabled) {
    status = "ON — LIVE FEEDBACK + MIRROR";
    state = "on";
  } else if (partial) {
    status = liveState.running
      ? "PARTIAL — FEEDBACK ON / MIRROR OFF"
      : "PARTIAL — FEEDBACK OFF / MIRROR ON";
    state = "partial";
  }
  setTextById("digitalTwinLiveMirrorQuickStatus", status);
  if (quickPanel && quickPanel.dataset) quickPanel.dataset.state = state;
}

function updateLiveSourceUi() {
  let operatingMode = "STATIC / LAST POSE";
  let feedbackState = "FEEDBACK STOPPED";
  if (liveState.running && liveState.lastFetchError) {
    operatingMode = "LIVE MIRROR ERROR";
    feedbackState = "JOINT FEEDBACK ERROR — READ ONLY";
  } else if (liveState.running) {
    operatingMode = "LIVE MIRROR";
    feedbackState = "JOINT FEEDBACK ACTIVE — READ ONLY";
  }

  const values = {
    digitalTwinLiveSourceState: liveState.status,
    digitalTwinPollInterval: `${liveState.pollIntervalMs} ms`,
    digitalTwinLastFetch: formatTimestamp(liveState.lastFetchMs),
    digitalTwinLastFetchError: liveState.lastFetchError || "NONE",
    digitalTwinOperatingMode: operatingMode,
    digitalTwinFeedbackState: feedbackState,
  };
  Object.entries(values).forEach(([id, value]) => {
    setTextById(id, value);
  });
  updateUnifiedLiveMirrorUi();
}

function getLiveFeedbackState() {
  return {
    running: liveState.running,
    status: liveState.status,
    pollIntervalMs: liveState.pollIntervalMs,
    lastFetchMs: liveState.lastFetchMs,
    lastFeedbackTimestampMs: liveState.lastFeedbackTimestampMs,
    endpointSource: liveState.endpointSource,
    sourceDiagnostics: liveState.sourceDiagnostics,
    lastFetchError: liveState.lastFetchError,
    hasInFlightRequest: activeAbortController !== null,
  };
}

function normalizedEndpointSnapshot(payload) {
  if (!payload || payload.ok !== true) {
    throw new Error("Joint feedback response is not ready for both arms");
  }
  for (const side of ["left", "right"]) {
    const feedback = payload[side];
    if (
      !feedback ||
      feedback.valid !== true ||
      !Array.isArray(feedback.joint) ||
      feedback.joint.length !== 6 ||
      !feedback.joint.every(
        (value) => typeof value === "number" && Number.isFinite(value),
      ) ||
      typeof feedback.received_at_ms !== "number" ||
      !Number.isFinite(feedback.received_at_ms)
    ) {
      throw new Error(`${side} joint feedback is invalid or missing`);
    }
  }

  return {
    snapshot: {
      left: { joint: [...payload.left.joint] },
      right: { joint: [...payload.right.joint] },
    },
    // A dual-arm snapshot is only as fresh as its older arm sample.
    receivedAtMs: Math.min(
      payload.left.received_at_ms,
      payload.right.received_at_ms,
    ),
    endpointSource: typeof payload.source === "string" ? payload.source : null,
    sourceDiagnostics: payload.source_diagnostics || null,
  };
}

async function executePoll() {
  const controller = new AbortController();
  activeAbortController = controller;
  const timeoutId = window.setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);

  try {
    const response = await fetch(JOINT_FEEDBACK_ENDPOINT, {
      method: "GET",
      cache: "no-store",
      signal: controller.signal,
    });
    liveState.lastFetchMs = Date.now();
    if (!response.ok) {
      throw new Error(`Joint feedback request failed with HTTP ${response.status}`);
    }
    const normalized = normalizedEndpointSnapshot(await response.json());
    const digitalTwin = window.dualArmDigitalTwin;
    if (!digitalTwin || typeof digitalTwin.ingestStatusSnapshot !== "function") {
      throw new Error("Digital Twin mirror API is unavailable");
    }
    digitalTwin.ingestStatusSnapshot(
      normalized.snapshot,
      normalized.receivedAtMs,
      "LIVE JOINT FEEDBACK",
    );
    liveState.lastFeedbackTimestampMs = normalized.receivedAtMs;
    liveState.endpointSource = normalized.endpointSource;
    liveState.sourceDiagnostics = normalized.sourceDiagnostics;
    liveState.lastFetchError = null;
    liveState.status = liveState.running ? "RUNNING — READ ONLY" : "STOPPED";
  } catch (error) {
    if (error && error.name === "AbortError" && !liveState.running) {
      liveState.lastFetchError = null;
      liveState.status = "STOPPED";
    } else {
      liveState.lastFetchError = error && error.message
        ? error.message
        : "Unknown joint feedback error";
      liveState.status = liveState.running ? "ERROR — READ ONLY" : "STOPPED";
    }
  } finally {
    window.clearTimeout(timeoutId);
    if (activeAbortController === controller) activeAbortController = null;
    updateLiveSourceUi();
  }
  return getLiveFeedbackState();
}

function pollLiveFeedbackOnce() {
  if (activePollPromise) return activePollPromise;
  activePollPromise = executePoll().finally(() => {
    activePollPromise = null;
  });
  return activePollPromise;
}

function scheduleNextPoll() {
  if (!liveState.running || pollTimerId !== null) return;
  pollTimerId = window.setTimeout(async () => {
    pollTimerId = null;
    await pollLiveFeedbackOnce();
    scheduleNextPoll();
  }, liveState.pollIntervalMs);
}

function startLiveFeedback() {
  if (liveState.running) return getLiveFeedbackState();
  const digitalTwin = window.dualArmDigitalTwin;
  if (!digitalTwin || typeof digitalTwin.setMirrorEnabled !== "function") {
    liveState.status = "ERROR — READ ONLY";
    liveState.lastFetchError = "Digital Twin mirror API is unavailable";
    updateLiveSourceUi();
    return getLiveFeedbackState();
  }

  digitalTwin.setMirrorEnabled(true);
  liveState.running = true;
  liveState.status = "STARTING — READ ONLY";
  liveState.lastFetchError = null;
  updateLiveSourceUi();
  void pollLiveFeedbackOnce().finally(scheduleNextPoll);
  return getLiveFeedbackState();
}

function stopLiveFeedback() {
  liveState.running = false;
  liveState.status = "STOPPED";
  if (pollTimerId !== null) {
    window.clearTimeout(pollTimerId);
    pollTimerId = null;
  }
  if (activeAbortController) {
    activeAbortController.abort();
    activeAbortController = null;
  }
  updateLiveSourceUi();
  return getLiveFeedbackState();
}

function setUnifiedLiveMirrorEnabled(enabled) {
  if (typeof enabled !== "boolean") {
    throw new TypeError("Live Mirror enabled state must be boolean");
  }
  const digitalTwin = window.dualArmDigitalTwin;
  if (!digitalTwin || typeof digitalTwin.setMirrorEnabled !== "function") {
    liveState.status = "ERROR — READ ONLY";
    liveState.lastFetchError = "Digital Twin mirror API is unavailable";
    updateLiveSourceUi();
    return getLiveFeedbackState();
  }

  if (enabled) {
    if (liveState.running) {
      digitalTwin.setMirrorEnabled(true);
      updateLiveSourceUi();
      return getLiveFeedbackState();
    }
    return startLiveFeedback();
  }

  stopLiveFeedback();
  digitalTwin.setMirrorEnabled(false);
  updateLiveSourceUi();
  return getLiveFeedbackState();
}

function bindLiveSourceControls() {
  const startButton = document.getElementById("digitalTwinStartLiveFeedback");
  const stopButton = document.getElementById("digitalTwinStopLiveFeedback");
  const unifiedSwitch = document.getElementById("digitalTwinLiveMirrorSwitch");
  if (startButton) startButton.addEventListener("click", startLiveFeedback);
  if (stopButton) stopButton.addEventListener("click", stopLiveFeedback);
  if (unifiedSwitch) {
    unifiedSwitch.addEventListener("change", () => {
      setUnifiedLiveMirrorEnabled(unifiedSwitch.checked === true);
    });
  }

  for (const id of [
    "digitalTwinEnableMirror",
    "digitalTwinDisableMirror",
    "digitalTwinResetStaticPose",
    "digitalTwinMockPoseA",
    "digitalTwinMockPoseB",
  ]) {
    const diagnosticControl = document.getElementById(id);
    if (diagnosticControl) {
      diagnosticControl.addEventListener("click", () => {
        Promise.resolve().then(updateUnifiedLiveMirrorUi);
      });
    }
  }
}

export {
  DEFAULT_POLL_INTERVAL_MS,
  getLiveFeedbackState,
  pollLiveFeedbackOnce,
  setUnifiedLiveMirrorEnabled,
  startLiveFeedback,
  stopLiveFeedback,
};

if (!window[INITIALIZATION_FLAG]) {
  window[INITIALIZATION_FLAG] = true;
  window.dualArmDigitalTwinLiveSource = {
    getLiveFeedbackState,
    pollLiveFeedbackOnce,
    setUnifiedLiveMirrorEnabled,
    startLiveFeedback,
    stopLiveFeedback,
  };
  bindLiveSourceControls();
  updateLiveSourceUi();
}
