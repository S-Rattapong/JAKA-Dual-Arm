const STATUS_URL = "/api/system-control/status";
const CONNECT_URL = "/api/system-control/connect-robots";
const RESET_MOVEIT_URL = "/api/system-control/reset-moveit";
const STATUS_POLL_MS = 1500;
const STATUS_TIMEOUT_MS = 3500;
const STATUS_SNAPSHOT_MAX_AGE_MS = 4500;
const MOTION_LABELS = new Map([
  [0, ["IDLE", "ok"]],
  [1, ["PAUSED", "warning"]],
  [2, ["E-STOP", "error"]],
  [3, ["MOVING", "warning"]],
  [4, ["ERROR", "error"]],
]);

const byId = (id) => document.getElementById(id);
const ui = {
  workspace: byId("digitalTwinSplitWorkspace"),
  divider: byId("digitalTwinSplitDivider"),
  resetSplit: byId("digitalTwinSplitReset"),
  connect: byId("systemControlConnectRobots"),
  resetMoveIt: byId("systemControlResetMoveIt"),
  detail: byId("systemControlActionDetail"),
  dialog: byId("systemControlConfirmDialog"),
  dialogTitle: byId("systemControlConfirmTitle"),
  dialogDescription: byId("systemControlConfirmDescription"),
  dialogError: byId("systemControlConfirmError"),
  dialogCancel: byId("systemControlCancelAction"),
  dialogConfirm: byId("systemControlConfirmAction"),
};

const armUi = Object.fromEntries(["left", "right"].map((side) => {
  const cap = side[0].toUpperCase() + side.slice(1);
  return [side, {
    connection: byId(`systemControl${cap}Connection`),
    power: byId(`systemControl${cap}Power`),
    enabled: byId(`systemControl${cap}EnableState`),
    motion: byId(`systemControl${cap}Motion`),
    feedback: byId(`systemControl${cap}Feedback`),
    age: byId(`systemControl${cap}FeedbackAge`),
    powerToggle: byId(`systemControl${cap}PowerToggle`),
    enableToggle: byId(`systemControl${cap}EnableToggle`),
  }];
}));

let latestStatus = null;
let latestStatusReceivedAt = 0;
let statusController = null;
let statusInFlight = null;
let statusEpoch = 0;
let mutationBusy = false;
let pendingConfirmation = null;
let confirmationTrigger = null;

function setValue(element, text, tone = "unknown") {
  if (!element) return;
  element.textContent = text;
  element.dataset.tone = tone;
}

function setDetail(message, tone = "neutral") {
  if (!ui.detail) return;
  ui.detail.textContent = String(message || "No additional detail.");
  ui.detail.title = ui.detail.textContent;
  ui.detail.dataset.tone = tone;
}

function statusTone(state) {
  if (state === "RUNNING" || state === "CONNECTED") return "ok";
  if (["STARTING", "STOPPING", "CONNECTING", "STOPPED", "OFFLINE"].includes(state)) return "warning";
  if (state === "ERROR") return "error";
  return "unknown";
}

function sourceTone(source) {
  if (source === "operator") return "ok";
  if (["manual", "external"].includes(source)) return "warning";
  return "unknown";
}

function booleanState(value, onText, offText) {
  if (value === true) return [onText, "ok"];
  if (value === false) return [offText, "warning"];
  return ["UNKNOWN", "unknown"];
}

function feedbackState(actual) {
  if (actual?.fresh === true) return ["FRESH", "ok"];
  if (actual?.received_at_ms != null) return ["STALE", "warning"];
  return ["MISSING", "unknown"];
}

function renderArm(side, driver, safeGate) {
  const target = armUi[side];
  const connection = driver?.connection || "UNKNOWN";
  const actual = driver?.actual || {};
  const authoritative = connection === "CONNECTED" && actual.fresh === true;
  setValue(target.connection, connection, statusTone(connection));

  const [powerText, powerTone] = booleanState(actual.power, "ON", "OFF");
  const [enableText, enableTone] = booleanState(actual.enabled, "ENABLED", "DISABLED");
  const motion = MOTION_LABELS.get(actual.motion) || ["UNKNOWN", "unknown"];
  const suffix = actual?.received_at_ms != null ? " · STALE" : " · UNKNOWN";

  if (authoritative) {
    setValue(target.power, powerText, powerTone);
    setValue(target.enabled, enableText, enableTone);
    setValue(target.motion, motion[0], motion[1]);
  } else {
    setValue(target.power, `${powerText}${suffix}`, actual.power == null ? "unknown" : "warning");
    setValue(target.enabled, `${enableText}${suffix}`, actual.enabled == null ? "unknown" : "warning");
    const staleMotionTone = motion[1] === "error" ? "error" : actual.motion == null ? "unknown" : "warning";
    setValue(target.motion, `${motion[0]}${suffix}`, staleMotionTone);
  }

  const feedback = feedbackState(actual);
  setValue(target.feedback, feedback[0], feedback[1]);
  const age = Number.isFinite(actual.age_ms) ? `${Math.max(0, Math.round(actual.age_ms))} ms` : "UNKNOWN";
  setValue(target.age, age, actual.fresh === true ? "ok" : Number.isFinite(actual.age_ms) ? "warning" : "unknown");

  const known = authoritative
    && typeof actual.power === "boolean" && typeof actual.enabled === "boolean"
    && Number.isInteger(actual.motion);
  const safe = safeGate === true && known && actual.motion === 0 && !mutationBusy;

  const toggleText = (kind, value, onText, offText) => {
    if (!authoritative || typeof value !== "boolean") {
      return `${kind}: ${actual?.received_at_ms != null ? "STALE" : "UNKNOWN"}`;
    }
    return `${kind}: ${value ? onText : offText}`;
  };
  const toggleTone = (value) => authoritative && typeof value === "boolean"
    ? (value ? "ok" : "warning")
    : (actual?.received_at_ms != null ? "warning" : "unknown");

  target.powerToggle.textContent = toggleText("POWER", actual.power, "ON", "OFF");
  target.powerToggle.dataset.tone = toggleTone(actual.power);
  target.powerToggle.dataset.risk = authoritative && actual.power === true ? "true" : "false";
  target.powerToggle.setAttribute("aria-pressed", authoritative && typeof actual.power === "boolean" ? String(actual.power) : "mixed");
  target.powerToggle.disabled = !(safe && (actual.power === false || (actual.power === true && actual.enabled === false)));

  target.enableToggle.textContent = toggleText("ENABLE", actual.enabled, "ENABLED", "DISABLED");
  target.enableToggle.dataset.tone = toggleTone(actual.enabled);
  target.enableToggle.dataset.risk = authoritative && actual.enabled === true ? "true" : "false";
  target.enableToggle.setAttribute("aria-pressed", authoritative && typeof actual.enabled === "boolean" ? String(actual.enabled) : "mixed");
  target.enableToggle.disabled = !(safe && actual.power === true && typeof actual.enabled === "boolean");
}
function renderSoftware(status) {
  const moveit = status?.software?.moveit || {};
  const web = status?.software?.web || {};
  const shutdown = status?.shutdown || {};
  setValue(byId("systemControlMoveItState"), moveit.state || "UNKNOWN", statusTone(moveit.state));
  setValue(byId("systemControlMoveItSource"), moveit.source || "UNKNOWN", sourceTone(moveit.source));
  setValue(byId("systemControlWebState"), web.state || "UNKNOWN", statusTone(web.state));
  setValue(byId("systemControlWebSource"), web.source || "UNKNOWN", sourceTone(web.source));
  const shutdownState = shutdown.state || "UNKNOWN";
  setValue(byId("systemControlShutdownState"), shutdownState,
    shutdown.safe === true ? "ok" : shutdownState === "ACTIVE" ? "warning" : shutdownState === "UNKNOWN" ? "unknown" : "error");
  setValue(byId("systemControlSafeToClose"),
    shutdown.safe === true ? "YES" : shutdown.safe === false ? "NO" : "UNKNOWN",
    shutdown.safe === true ? "ok" : shutdown.safe === false ? "warning" : "unknown");
  setValue(byId("systemControlShutdownReason"), shutdown.reason || "UNKNOWN",
    shutdown.safe === true ? "ok" : shutdown.safe === false ? "warning" : "unknown");

  const processOk = !status?.process_scan_error;
  const missingStartable = ["left", "right"].some((side) =>
    status?.drivers?.[side]?.duplicate_blocked === false);
  ui.connect.disabled = mutationBusy || !processOk || !missingStartable;
  ui.resetMoveIt.disabled = mutationBusy || shutdown.safe !== true
    || moveit.state !== "RUNNING" || moveit.source !== "operator";
}

function renderStatus(status, receivedAt = Date.now()) {
  latestStatus = status;
  latestStatusReceivedAt = receivedAt;
  const safeGate = status?.shutdown?.safe === true;
  renderArm("left", status?.drivers?.left, safeGate);
  renderArm("right", status?.drivers?.right, safeGate);
  renderSoftware(status);
  if (status?.process_scan_error) {
    setDetail("Process inspection failed. System controls remain blocked until ownership can be verified.", "error");
  }
}

function renderUnavailable(message) {
  latestStatus = null;
  latestStatusReceivedAt = 0;
  for (const side of ["left", "right"]) {
    for (const key of ["connection", "power", "enabled", "motion", "feedback", "age"]) {
      setValue(armUi[side][key], "UNKNOWN", "unknown");
    }
    for (const key of ["powerToggle", "enableToggle"]) {
      armUi[side][key].disabled = true;
      armUi[side][key].dataset.tone = "unknown";
      armUi[side][key].dataset.risk = "false";
      armUi[side][key].setAttribute("aria-pressed", "mixed");
      armUi[side][key].textContent = key === "powerToggle" ? "POWER: UNKNOWN" : "ENABLE: UNKNOWN";
    }
  }
  for (const id of ["systemControlMoveItState", "systemControlMoveItSource", "systemControlWebState",
    "systemControlWebSource", "systemControlShutdownState", "systemControlSafeToClose", "systemControlShutdownReason"]) {
    setValue(byId(id), "UNKNOWN", "unknown");
  }
  ui.connect.disabled = true;
  ui.resetMoveIt.disabled = true;
  setDetail(message || "System status unavailable. Controls remain blocked.", "error");
}

function invalidateStatusRead() {
  statusEpoch += 1;
  const controller = statusController;
  statusController = null;
  statusInFlight = null;
  controller?.abort();
}

async function readStatus({ announceFailure = false, force = false } = {}) {
  if (statusInFlight && !force) return null;
  if (force && statusInFlight) invalidateStatusRead();

  const epoch = ++statusEpoch;
  const controller = new AbortController();
  statusController = controller;
  statusInFlight = epoch;
  const timeout = window.setTimeout(() => controller.abort(), STATUS_TIMEOUT_MS);
  try {
    const response = await fetch(STATUS_URL, { method: "GET", cache: "no-store", signal: controller.signal });
    const payload = await response.json().catch(() => ({}));
    if (epoch !== statusEpoch) return null;
    if (!response.ok) throw new Error(payload?.detail || `Status request failed (${response.status})`);
    renderStatus(payload, Date.now());
    return payload;
  } catch (error) {
    if (epoch !== statusEpoch) return null;
    const message = error?.name === "AbortError"
      ? "System status timed out. Controls remain blocked until a fresh snapshot arrives."
      : "System status unavailable. Controls remain blocked. Check the operator Web backend.";
    renderUnavailable(message);
    if (announceFailure) setDetail(String(error?.message || message), "error");
    return null;
  } finally {
    window.clearTimeout(timeout);
    if (statusInFlight === epoch) {
      statusInFlight = null;
      statusController = null;
    }
  }
}

async function postJson(url, body) {
  const options = { method: "POST", headers: {} };
  if (body !== undefined) {
    options.headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(body);
  }
  const response = await fetch(url, options);
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload?.detail || payload?.error || `Request failed (${response.status})`);
  return payload;
}

async function runMutation(label, action) {
  if (mutationBusy) return { ok: false, uncertain: true };
  mutationBusy = true;
  invalidateStatusRead();
  if (latestStatus) renderStatus(latestStatus, latestStatusReceivedAt);
  setDetail(`${label} — request pending. Waiting for backend confirmation.`, "busy");
  try {
    const payload = await action();
    const message = payload?.message || payload?.detail || `${label} request accepted.`;
    setDetail(`${label}: ${message} Refreshing actual state…`, "busy");
    const refreshed = await readStatus({ announceFailure: true, force: true });
    if (!refreshed) {
      setDetail(`${label}: backend responded, but actual state could not be re-verified. Controls remain blocked; do not repeat the action until fresh status returns.`, "error");
      return { ok: false, uncertain: true };
    }
    setDetail(`${label}: backend responded; displayed state is a fresh status snapshot.`, "ok");
    return { ok: true, uncertain: false };
  } catch (error) {
    setDetail(`${label} blocked, failed, or response was lost: ${String(error?.message || error)} Re-reading actual state…`, "error");
    const refreshed = await readStatus({ force: true });
    if (!refreshed) {
      setDetail(`${label}: request outcome is unverified because status could not be refreshed. Do not repeat until fresh status returns.`, "error");
    }
    return { ok: false, uncertain: !refreshed };
  } finally {
    mutationBusy = false;
    if (latestStatus) renderStatus(latestStatus, latestStatusReceivedAt);
  }
}

function stateUrl(side, control) {
  if (!["left", "right"].includes(side) || !["power", "enable"].includes(control)) {
    throw new Error("Invalid fixed system-control endpoint");
  }
  return `/api/system-control/robot/${side}/${control}`;
}

function requestState(side, control, enabled, label) {
  if (typeof enabled !== "boolean") throw new Error("System-control state must be boolean");
  return runMutation(label, () => postJson(stateUrl(side, control), { enabled }));
}

function verifiedArmActual(side) {
  const driver = latestStatus?.drivers?.[side];
  const actual = driver?.actual;
  const snapshotFresh = latestStatusReceivedAt > 0
    && Date.now() - latestStatusReceivedAt <= STATUS_SNAPSHOT_MAX_AGE_MS;
  const verified = snapshotFresh
    && latestStatus?.shutdown?.safe === true
    && driver?.connection === "CONNECTED"
    && actual?.fresh === true
    && typeof actual.power === "boolean"
    && typeof actual.enabled === "boolean"
    && actual.motion === 0
    && !mutationBusy;
  return verified ? actual : null;
}

function openConfirmation(trigger, title, description, confirmLabel, action) {
  if (!ui.dialog || mutationBusy) return;
  confirmationTrigger = trigger;
  pendingConfirmation = action;
  ui.dialogTitle.textContent = title;
  ui.dialogDescription.textContent = description;
  ui.dialogConfirm.textContent = confirmLabel;
  ui.dialogConfirm.disabled = false;
  ui.dialogCancel.disabled = false;
  ui.dialogError.hidden = true;
  ui.dialogError.textContent = "";
  ui.dialog.showModal();
  queueMicrotask(() => ui.dialogCancel.focus());
}

function closeConfirmation() {
  if (mutationBusy) return;
  if (ui.dialog?.open) ui.dialog.close();
  pendingConfirmation = null;
  const target = confirmationTrigger;
  confirmationTrigger = null;
  if (target?.isConnected) queueMicrotask(() => target.focus());
}

ui.dialogCancel?.addEventListener("click", closeConfirmation);
ui.dialog?.addEventListener("cancel", (event) => {
  event.preventDefault();
  if (!mutationBusy) closeConfirmation();
});
ui.dialog?.addEventListener("close", () => {
  pendingConfirmation = null;
  const target = confirmationTrigger;
  confirmationTrigger = null;
  if (target?.isConnected) queueMicrotask(() => target.focus());
});

ui.dialogConfirm?.addEventListener("click", async () => {
  if (!pendingConfirmation || mutationBusy) return;
  const action = pendingConfirmation;
  ui.dialogConfirm.disabled = true;
  ui.dialogCancel.disabled = true;
  ui.dialogError.hidden = true;
  let keepConfirmBlocked = false;
  try {
    const result = await action();
    if (!result?.ok) {
      keepConfirmBlocked = result?.uncertain === true;
      ui.dialogError.textContent = ui.detail?.textContent || "The requested action was not confirmed.";
      ui.dialogError.hidden = false;
      return;
    }
    closeConfirmation();
  } catch (error) {
    keepConfirmBlocked = true;
    ui.dialogError.textContent = String(error?.message || error);
    ui.dialogError.hidden = false;
  } finally {
    if (ui.dialog?.open) {
      ui.dialogConfirm.disabled = keepConfirmBlocked;
      ui.dialogCancel.disabled = false;
    }
  }
});

ui.connect?.addEventListener("click", () => {
  runMutation("Connect robots", () => postJson(CONNECT_URL));
});

for (const side of ["left", "right"]) {
  const name = side[0].toUpperCase() + side.slice(1);

  armUi[side].powerToggle?.addEventListener("click", (event) => {
    const actual = verifiedArmActual(side);
    if (!actual) {
      renderUnavailable("Power state is not fresh and verified. Controls remain blocked.");
      return;
    }
    if (actual.power === false) {
      requestState(side, "power", true, `${name} Power ON`);
      return;
    }
    if (actual.enabled !== false) {
      setDetail(`${name} Power OFF is blocked until actual feedback confirms the robot is disabled and idle.`, "error");
      renderStatus(latestStatus, latestStatusReceivedAt);
      return;
    }
    openConfirmation(
      event.currentTarget,
      `Power OFF ${name} robot`,
      `Power OFF is allowed only after ${name} is disabled and confirmed idle. This action does not send STOP.`,
      "Power OFF",
      () => requestState(side, "power", false, `${name} Power OFF`),
    );
  });

  armUi[side].enableToggle?.addEventListener("click", (event) => {
    const actual = verifiedArmActual(side);
    if (!actual) {
      renderUnavailable("Enable state is not fresh and verified. Controls remain blocked.");
      return;
    }
    if (actual.enabled === false) {
      if (actual.power !== true) {
        setDetail(`${name} Enable is blocked until actual feedback confirms power is ON.`, "error");
        renderStatus(latestStatus, latestStatusReceivedAt);
        return;
      }
      requestState(side, "enable", true, `${name} Enable`);
      return;
    }
    openConfirmation(
      event.currentTarget,
      `Disable ${name} robot`,
      `DISABLE removes servo enable from the ${name} robot. The backend requires fresh idle feedback. This action does not send STOP.`,
      "Disable robot",
      () => requestState(side, "enable", false, `${name} Disable`),
    );
  });
}

ui.resetMoveIt?.addEventListener("click", (event) => openConfirmation(
  event.currentTarget,
  "Reset operator MoveIt",
  "Restart only the operator-managed MoveIt unit. The backend requires the robot execution state to be confirmed safe and refuses external MoveIt ownership.",
  "Reset MoveIt",
  () => runMutation("Reset MoveIt", () => postJson(RESET_MOVEIT_URL)),
));

const SPLIT_HARD_MIN = 35;
const SPLIT_HARD_MAX = 65;
const SPLIT_DEFAULT_TWIN = 55;
const SPLIT_VIEWER_MIN_PX = 600;
const SPLIT_OPERATOR_MIN_PX = 520;
const SPLIT_DIVIDER_PX = 12;
const splitMedia = window.matchMedia("(min-width: 1321px)");
let splitDragging = false;

function splitBounds() {
  const width = ui.workspace?.getBoundingClientRect().width || 0;
  const usable = Math.max(1, width - SPLIT_DIVIDER_PX);
  const dynamicMin = (SPLIT_VIEWER_MIN_PX / usable) * 100;
  const dynamicMax = 100 - (SPLIT_OPERATOR_MIN_PX / usable) * 100;
  const min = Math.max(SPLIT_HARD_MIN, dynamicMin);
  const max = Math.min(SPLIT_HARD_MAX, dynamicMax);
  if (min <= max) return { min, max, usable };
  return { min: SPLIT_DEFAULT_TWIN, max: SPLIT_DEFAULT_TWIN, usable };
}

function setSplit(twinPercent) {
  if (!ui.workspace || !ui.divider) return;
  if (!splitMedia.matches) {
    ui.workspace.style.removeProperty("--twin-size");
    return;
  }
  const { min, max, usable } = splitBounds();
  const requested = Number.isFinite(Number(twinPercent)) ? Number(twinPercent) : SPLIT_DEFAULT_TWIN;
  const value = Math.min(max, Math.max(min, requested));
  const twinPx = (usable * value) / 100;
  ui.workspace.style.setProperty("--twin-size", `${twinPx.toFixed(2)}px`);
  ui.workspace.style.setProperty("--twin-share", `${value.toFixed(2)}%`);
  ui.workspace.style.setProperty("--operator-share", `${(100 - value).toFixed(2)}%`);
  ui.divider.setAttribute("aria-valuemin", String(Math.ceil(min)));
  ui.divider.setAttribute("aria-valuemax", String(Math.floor(max)));
  ui.divider.setAttribute("aria-valuenow", String(Math.round(value)));
  ui.divider.setAttribute("aria-valuetext",
    `Digital Twin ${Math.round(value)} percent; Operator workspace ${Math.round(100 - value)} percent`);
}

function splitFromPointer(clientX) {
  const rect = ui.workspace?.getBoundingClientRect();
  if (!rect || rect.width <= SPLIT_DIVIDER_PX) return SPLIT_DEFAULT_TWIN;
  const usable = rect.width - SPLIT_DIVIDER_PX;
  return ((clientX - rect.left) / usable) * 100;
}

ui.divider?.addEventListener("pointerdown", (event) => {
  if (!splitMedia.matches) return;
  splitDragging = true;
  ui.divider.dataset.dragging = "true";
  document.body.classList.add("digital-twin-split-resizing");
  ui.divider.setPointerCapture?.(event.pointerId);
  setSplit(splitFromPointer(event.clientX));
});
ui.divider?.addEventListener("pointermove", (event) => {
  if (!splitDragging || !splitMedia.matches) return;
  setSplit(splitFromPointer(event.clientX));
});

function endSplitDrag(event) {
  if (!splitDragging) return;
  splitDragging = false;
  delete ui.divider.dataset.dragging;
  document.body.classList.remove("digital-twin-split-resizing");
  if (event?.pointerId != null && ui.divider.hasPointerCapture?.(event.pointerId)) {
    ui.divider.releasePointerCapture(event.pointerId);
  }
}
ui.divider?.addEventListener("pointerup", endSplitDrag);
ui.divider?.addEventListener("pointercancel", endSplitDrag);

ui.divider?.addEventListener("keydown", (event) => {
  if (!splitMedia.matches || !["ArrowLeft", "ArrowRight"].includes(event.key)) return;
  event.preventDefault();
  const current = Number(ui.divider.getAttribute("aria-valuenow")) || SPLIT_DEFAULT_TWIN;
  setSplit(current + (event.key === "ArrowRight" ? 2 : -2));
});

ui.resetSplit?.addEventListener("click", () => setSplit(SPLIT_DEFAULT_TWIN));
window.addEventListener("resize", () => {
  const current = Number(ui.divider?.getAttribute("aria-valuenow")) || SPLIT_DEFAULT_TWIN;
  setSplit(current);
});
splitMedia.addEventListener?.("change", () => setSplit(SPLIT_DEFAULT_TWIN));
setSplit(SPLIT_DEFAULT_TWIN);

function pollStatus() {
  if (latestStatus && latestStatusReceivedAt > 0
      && Date.now() - latestStatusReceivedAt > STATUS_SNAPSHOT_MAX_AGE_MS) {
    renderUnavailable("System status snapshot expired. Controls remain blocked until a fresh snapshot arrives.");
  }
  if (!mutationBusy && !statusInFlight) readStatus();
}

readStatus();
window.setInterval(pollStatus, STATUS_POLL_MS);
