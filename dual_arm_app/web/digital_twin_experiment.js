// EXP-0 observational run library and offline path inspection.
// This module has no robot-command endpoint and creates no execution authority.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

const EXPERIMENT_API = "/api/experiments";
const EXPERIMENT_MODEL_URL = "/digital-twin/assets/dual_jaka_a12_web.urdf";
const JOINT_NAMES = Object.freeze([
  ...Array.from({ length: 6 }, (_unused, i) => `left_joint_${i + 1}`),
  ...Array.from({ length: 6 }, (_unused, i) => `right_joint_${i + 1}`),
]);
const TERMINAL_LINKS = Object.freeze({ left: "left_J6", right: "right_J6" });
const FK_SOURCE = "OFFLINE URDF FK FROM RECORDED MODEL/ENCODER JOINTS — NOT EXTERNAL METROLOGY";
const RELATIVE_ERROR_RESERVED = "EXP-2 RELATIVE POSE IS MODEL-BASED — NOT EXTERNAL METROLOGY";

const SERIES = Object.freeze({
  plannedLeft: { color: 0x22d3ee, planned: true, side: "left" },
  plannedRight: { color: 0x3b82f6, planned: true, side: "right" },
  plannedCenter: { color: 0xffffff, planned: true, center: true },
  actualLeft: { color: 0x22c55e, actual: true, side: "left" },
  actualRight: { color: 0xf59e0b, actual: true, side: "right" },
  actualCenterFromLeft: { color: 0xe879f9, actual: true, center: true },
  actualCenterFromRight: { color: 0xef4444, actual: true, center: true },
});

const state = {
  recorder: null,
  runs: [],
  loaded: null,
  derived: null,
  selectedIndex: 0,
  hiddenFkModel: null,
  fkStatus: "LOADING",
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  pathRoot: null,
  markerRoot: null,
  paths: new Map(),
  markers: new Map(),
  lastRecorderState: "IDLE",
  viewerBounds: null,
};

function element(id) {
  return document.getElementById(id);
}

function setText(id, value) {
  const target = element(id);
  if (target) target.textContent = value;
}

function setMessage(message, isError = false) {
  const target = element("experimentMessage");
  if (!target) return;
  target.textContent = message;
  target.dataset.error = isError ? "true" : "false";
}

function finiteVector(value, length) {
  return Array.isArray(value)
    && value.length === length
    && value.every((item) => typeof item === "number" && Number.isFinite(item));
}

function formatNumber(value, digits = 4) {
  return typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(digits) : "N/A";
}

function formatPoint(point) {
  return finiteVector(point, 3)
    ? `[${point.map((value) => formatNumber(value)).join(", ")}] m`
    : "UNAVAILABLE";
}

function cloneJson(value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
}

async function requestExperiment(path = "", options = {}) {
  const response = await fetch(`${EXPERIMENT_API}${path}`, {
    cache: "no-store",
    ...options,
    headers: options.body
      ? { "Content-Type": "application/json", ...(options.headers || {}) }
      : options.headers,
  });
  let payload;
  try {
    payload = await response.json();
  } catch (_error) {
    payload = null;
  }
  if (!response.ok) {
    const detail = payload && (payload.detail || payload.error);
    throw new Error(detail || `Experiment request failed (${response.status})`);
  }
  return payload;
}

function currentPlanSnapshot() {
  const controller = window.dualArmDigitalTwin;
  if (!controller || typeof controller.getObjectGlobalPlanState !== "function") {
    return null;
  }
  const snapshot = controller.getObjectGlobalPlanState();
  return snapshot && snapshot.status === "READY" && snapshot.plan
    ? cloneJson(snapshot.plan) : null;
}

function renderRecorder(recorder) {
  state.recorder = recorder || {};
  const counts = state.recorder.sample_counts || {};
  const torque = state.recorder.torque_availability || {};
  const trajectory = state.recorder.trajectory || {};
  setText("experimentRecorderState", state.recorder.state || "IDLE");
  setText(
    "experimentArmedArtifact",
    `${state.recorder.run_id || "NONE"} / ${state.recorder.armed_artifact_fingerprint || "NONE"}`,
  );
  setText(
    "experimentActiveTrajectory",
    state.recorder.active_trajectory_id
      || trajectory.name
      || "NONE",
  );
  setText("experimentSampleCounts", Object.keys(counts).length ? JSON.stringify(counts) : "NONE");
  setText("experimentLeftTorqueAvailability", torque.left ? "AVAILABLE" : "UNAVAILABLE");
  setText("experimentRightTorqueAvailability", torque.right ? "AVAILABLE" : "UNAVAILABLE");
  setText("experimentRecorderError", state.recorder.error || "NONE");
  const arm = element("experimentArmRecorder");
  const disarm = element("experimentDisarmRecorder");
  if (arm) arm.disabled = ["ARMED", "RECORDING"].includes(state.recorder.state);
  if (disarm) disarm.disabled = state.recorder.state !== "ARMED";
}

async function refreshRecorder({ quiet = false } = {}) {
  try {
    const recorder = await requestExperiment("/recorder");
    const wasActive = ["ARMED", "RECORDING"].includes(state.lastRecorderState);
    renderRecorder(recorder);
    state.lastRecorderState = recorder.state || "IDLE";
    if (wasActive && recorder.state === "CLOSED") {
      await refreshRuns({ selectRunId: recorder.run_id, quiet: true });
      await loadRun(recorder.run_id);
    } else if (!quiet) {
      setMessage("Recorder state refreshed. No robot command was sent.");
    }
    return recorder;
  } catch (error) {
    if (!quiet) setMessage(error.message, true);
    return null;
  }
}

async function armRecorder() {
  const planSnapshot = currentPlanSnapshot();
  const payload = {
    label: element("experimentLabel")?.value || "",
    path_type: element("experimentPathType")?.value || "CUSTOM",
    fixture_condition: element("experimentFixtureCondition")?.value || "NONE",
    notes: element("experimentNotes")?.value || "",
    plan_snapshot: planSnapshot,
  };
  try {
    const recorder = await requestExperiment("/arm", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    renderRecorder(recorder);
    state.lastRecorderState = recorder.state;
    await refreshRuns({ selectRunId: recorder.run_id, quiet: true });
    setMessage(
      `Recorder armed for ${recorder.run_id}. This is metadata/observation only; no robot command was sent.`,
    );
  } catch (error) {
    setMessage(error.message, true);
  }
}

async function disarmRecorder() {
  try {
    const recorder = await requestExperiment("/disarm", { method: "POST" });
    renderRecorder(recorder);
    state.lastRecorderState = recorder.state;
    await refreshRuns({ selectRunId: recorder.run_id, quiet: true });
    setMessage("Recorder disarmed before execution. No robot command was sent.");
  } catch (error) {
    setMessage(error.message, true);
  }
}

function selectedRunId() {
  return element("experimentRunSelect")?.value || "";
}

function renderRunList(selectRunId = null) {
  const selector = element("experimentRunSelect");
  if (!selector) return;
  const previous = selectRunId || selector.value;
  selector.replaceChildren();
  if (!state.runs.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No experiment runs";
    selector.append(option);
    return;
  }
  state.runs.forEach((run) => {
    const option = document.createElement("option");
    option.value = run.run_id;
    const warning = run.warning ? ` ⚠ ${run.warning}` : "";
    option.textContent = `${run.label || run.run_id} — ${run.state || "UNKNOWN"}${warning}`;
    option.dataset.corrupt = run.warning ? "true" : "false";
    selector.append(option);
  });
  if (state.runs.some((run) => run.run_id === previous)) selector.value = previous;
}

async function refreshRuns({ selectRunId = null, quiet = false } = {}) {
  try {
    const payload = await requestExperiment();
    state.runs = Array.isArray(payload.runs) ? payload.runs : [];
    renderRunList(selectRunId);
    if (payload.recorder) renderRecorder(payload.recorder);
    const corruptCount = state.runs.filter((run) => run.warning).length;
    if (!quiet) {
      setMessage(
        `${state.runs.length} run(s) listed${corruptCount ? `; ${corruptCount} corrupt run warning(s)` : ""}.`,
        corruptCount > 0,
      );
    }
    return payload;
  } catch (error) {
    setMessage(error.message, true);
    return null;
  }
}

async function loadRun(runId = selectedRunId()) {
  if (!runId) {
    setMessage("Select an experiment run to load.", true);
    return null;
  }
  try {
    const payload = await requestExperiment(`/${encodeURIComponent(runId)}`);
    state.loaded = payload.run;
    const manifest = state.loaded.manifest || {};
    if (element("experimentLabel")) element("experimentLabel").value = manifest.label || "";
    if (element("experimentPathType")) element("experimentPathType").value = manifest.path_type || "CUSTOM";
    if (element("experimentFixtureCondition")) {
      element("experimentFixtureCondition").value = manifest.fixture_condition || "NONE";
    }
    if (element("experimentNotes")) element("experimentNotes").value = manifest.notes || "";
    renderExp1Analysis(state.loaded.exp1_analysis || null);
    renderExp2Analysis(state.loaded.exp2_analysis || null);
    rebuildDerivedRun();
    const exp1Status = state.loaded.exp1_analysis?.status || "NOT ANALYZED";
    const exp2Status = state.loaded.exp2_analysis?.status || "NOT ANALYZED";
    setMessage(`Loaded ${runId}. EXP-1: ${exp1Status}; EXP-2: ${exp2Status}. Raw evidence is authoritative; derived alignment/analysis is offline only.`);
    return state.loaded;
  } catch (error) {
    setMessage(error.message, true);
    return null;
  }
}

async function relabelRun() {
  const runId = selectedRunId();
  if (!runId) return setMessage("Select an experiment run to relabel.", true);
  const current = state.runs.find((run) => run.run_id === runId)?.label || "";
  const label = window.prompt("New display label", current);
  if (label === null) return null;
  try {
    await requestExperiment(`/${encodeURIComponent(runId)}`, {
      method: "PATCH",
      body: JSON.stringify({ label }),
    });
    await refreshRuns({ selectRunId: runId, quiet: true });
    if (state.loaded?.manifest?.run_id === runId) state.loaded.manifest.label = label.trim();
    setMessage(`Relabeled ${runId}. The immutable run ID and raw evidence were unchanged.`);
  } catch (error) {
    setMessage(error.message, true);
  }
  return null;
}

async function deleteRun() {
  const runId = selectedRunId();
  if (!runId) return setMessage("Select an experiment run to delete.", true);
  if (!window.confirm(`Delete experiment run ${runId}? This explicit action cannot be undone.`)) {
    return null;
  }
  try {
    await requestExperiment(`/${encodeURIComponent(runId)}`, { method: "DELETE" });
    if (state.loaded?.manifest?.run_id === runId) {
      state.loaded = null;
      state.derived = null;
      clearViewerPaths();
      updateSelectedSample();
      drawJointSeries();
      renderExp1Analysis(null);
      renderExp2Analysis(null);
    }
    await refreshRuns({ quiet: true });
    setMessage(`Deleted experiment run ${runId} after explicit confirmation.`);
  } catch (error) {
    setMessage(error.message, true);
  }
  return null;
}

async function exportRun() {
  const runId = selectedRunId();
  if (!runId) return setMessage("Select an experiment run to export.", true);
  try {
    const payload = await requestExperiment(`/${encodeURIComponent(runId)}/export`);
    const blob = new Blob([`${JSON.stringify(payload.run, null, 2)}\n`], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `${runId}.json`;
    anchor.click();
    URL.revokeObjectURL(url);
    setMessage(`Exported ${runId} as machine-readable JSON.`);
  } catch (error) {
    setMessage(error.message, true);
  }
  return null;
}

function coverageText(coverage) {
  const usable = Number.isInteger(coverage?.usable_rows) ? coverage.usable_rows : 0;
  const total = Number.isInteger(coverage?.total_aligned_rows) ? coverage.total_aligned_rows : 0;
  const percentage = total ? ` (${formatNumber(100 * usable / total, 1)}%)` : "";
  return `${usable} / ${total} rows${percentage}`;
}

function worstJointText(summary) {
  const maximum = summary?.worst_joint_by_max_error || "N/A";
  const rmse = summary?.worst_joint_by_rmse || "N/A";
  return maximum === rmse ? maximum : `${maximum} max · ${rmse} RMSE`;
}

function renderExp1Summary(name, summary, coverage) {
  setText(`experimentExp1${name}Rmse`, formatNumber(summary?.rmse_rad, 6));
  setText(`experimentExp1${name}Mae`, formatNumber(summary?.mae_rad, 6));
  setText(`experimentExp1${name}Max`, formatNumber(summary?.max_abs_error_rad, 6));
  setText(`experimentExp1${name}Worst`, worstJointText(summary));
  setText(`experimentExp1${name}Coverage`, coverageText(coverage));
}

function renderExp1JointRows(analysis) {
  const body = element("experimentExp1JointRows");
  if (!body) return;
  body.replaceChildren();
  for (const side of ["left", "right"]) {
    const metrics = Array.isArray(analysis?.arms?.[side]?.joints)
      ? analysis.arms[side].joints : [];
    for (let index = 0; index < 6; index += 1) {
      const metric = metrics.find((item) => item?.joint_index === index) || metrics[index] || {};
      const row = document.createElement("tr");
      const cells = [
        `${side === "left" ? "Left" : "Right"} J${index + 1}`,
        formatNumber(metric.mean_signed_error_rad, 6),
        formatNumber(metric.mae_rad, 6),
        formatNumber(metric.rmse_rad, 6),
        formatNumber(metric.max_abs_error_rad, 6),
        Number.isInteger(metric.sample_count) ? String(metric.sample_count) : "0",
      ];
      cells.forEach((value) => {
        const cell = document.createElement("td");
        cell.textContent = value;
        row.append(cell);
      });
      body.append(row);
    }
  }
}

function renderExp1Analysis(analysis = state.loaded?.exp1_analysis || null) {
  setText("experimentExp1Status", analysis?.status || "NOT ANALYZED");
  const source = analysis?.actual_source_semantics;
  const observedSources = Array.isArray(source?.observed_sources) && source.observed_sources.length
    ? ` Sources: ${source.observed_sources.join(", ")}.` : "";
  setText(
    "experimentExp1ActualSource",
    source ? `${source.description || "Recorded joint evidence"}.${observedSources} NOT EXTERNAL METROLOGY.`
      : "UNAVAILABLE",
  );
  renderExp1Summary("Combined", analysis?.combined, analysis?.coverage);
  renderExp1Summary("Left", analysis?.arms?.left?.summary, analysis?.coverage?.sides?.left);
  renderExp1Summary("Right", analysis?.arms?.right?.summary, analysis?.coverage?.sides?.right);
  renderExp1JointRows(analysis);
  drawExp1ErrorSeries(analysis);
}

async function analyzeExp1() {
  const runId = selectedRunId();
  if (!runId) return setMessage("Select an experiment run to analyze.", true);
  const button = element("experimentExp1Analyze");
  if (button) button.disabled = true;
  try {
    const payload = await requestExperiment(`/${encodeURIComponent(runId)}/exp1-analysis`, {
      method: "POST",
    });
    if (state.loaded?.manifest?.run_id === runId) {
      state.loaded.exp1_analysis = payload.exp1_analysis;
      renderExp1Analysis(payload.exp1_analysis);
    } else {
      await loadRun(runId);
    }
    setMessage(`EXP-1 ${payload.exp1_analysis?.status || "UNAVAILABLE"} for ${runId}. Offline analysis only; no robot command was sent.`);
    return payload.exp1_analysis;
  } catch (error) {
    setMessage(error.message, true);
    return null;
  } finally {
    if (button) button.disabled = false;
  }
}

function exp2Mm(value) {
  return Number.isFinite(value) ? formatNumber(value * 1000, 3) : "N/A";
}

function exp2Time(location) {
  return Number.isFinite(location?.time_from_start_s)
    ? formatNumber(location.time_from_start_s, 3) : "N/A";
}

function exp2CoverageText(coverage) {
  const usable = Number.isInteger(coverage?.usable_synchronized_samples)
    ? coverage.usable_synchronized_samples : 0;
  const total = Number.isInteger(coverage?.synchronized_candidate_samples)
    ? coverage.synchronized_candidate_samples : 0;
  const percentage = total ? ` (${formatNumber(100 * usable / total, 1)}%)` : "";
  return `${usable} / ${total} synchronized samples${percentage}`;
}

function renderExp2AxisRows(analysis) {
  const body = element("experimentExp2AxisRows");
  if (!body) return;
  body.replaceChildren();
  const axes = analysis?.relative_translation?.axes_m || {};
  for (const axis of ["x", "y", "z"]) {
    const metric = axes[axis] || {};
    const row = document.createElement("tr");
    [axis.toUpperCase(), exp2Mm(metric.mean), exp2Mm(metric.mae), exp2Mm(metric.rmse), exp2Mm(metric.max_abs)]
      .forEach((value) => {
        const cell = document.createElement("td");
        cell.textContent = value;
        row.append(cell);
      });
    body.append(row);
  }
}

function renderExp2Analysis(analysis = state.loaded?.exp2_analysis || null) {
  setText("experimentExp2Status", analysis?.status || "NOT ANALYZED");
  const source = analysis?.actual_source_semantics;
  const observed = Array.isArray(source?.observed_sources) && source.observed_sources.length
    ? ` Sources: ${source.observed_sources.join(", ")}.` : "";
  setText(
    "experimentExp2ActualSource",
    source ? `${source.fk_source || "OFFLINE URDF FK"}; ${source.description || "recorded joints"}.${observed} NOT EXTERNAL METROLOGY.`
      : "UNAVAILABLE",
  );
  setText("experimentExp2Coverage", exp2CoverageText(analysis?.coverage));
  const translation = analysis?.relative_translation?.magnitude_m;
  const orientation = analysis?.relative_orientation?.angle;
  const centerTranslation = analysis?.center_consistency?.translation_magnitude_m;
  const centerOrientation = analysis?.center_consistency?.orientation_angle;
  setText("experimentExp2TranslationRms", exp2Mm(translation?.rms));
  setText("experimentExp2TranslationMax", exp2Mm(translation?.max));
  setText("experimentExp2TranslationWorstTime", exp2Time(translation?.max_location));
  setText("experimentExp2OrientationRms", formatNumber(orientation?.rms_deg, 4));
  setText("experimentExp2OrientationMax", formatNumber(orientation?.max_deg, 4));
  setText("experimentExp2OrientationWorstTime", exp2Time(orientation?.max_location));
  setText("experimentExp2CenterTranslationRms", exp2Mm(centerTranslation?.rms));
  setText("experimentExp2CenterTranslationMax", exp2Mm(centerTranslation?.max));
  setText("experimentExp2CenterOrientationRms", formatNumber(centerOrientation?.rms_deg, 4));
  setText("experimentExp2CenterOrientationMax", formatNumber(centerOrientation?.max_deg, 4));
  renderExp2AxisRows(analysis);
  drawExp2Series(analysis);
}

async function analyzeExp2() {
  const runId = selectedRunId();
  if (!runId) return setMessage("Select an experiment run to analyze.", true);
  const button = element("experimentExp2Analyze");
  if (button) button.disabled = true;
  try {
    const payload = await requestExperiment(`/${encodeURIComponent(runId)}/exp2-analysis`, {
      method: "POST",
    });
    if (state.loaded?.manifest?.run_id === runId) {
      state.loaded.exp2_analysis = payload.exp2_analysis;
      renderExp2Analysis(payload.exp2_analysis);
      updateSelectedSample();
    } else {
      await loadRun(runId);
    }
    setMessage(`EXP-2 ${payload.exp2_analysis?.status || "UNAVAILABLE"} for ${runId}. Encoder+URDF model-based offline analysis only; no robot command was sent.`);
    return payload.exp2_analysis;
  } catch (error) {
    setMessage(error.message, true);
    return null;
  } finally {
    if (button) button.disabled = false;
  }
}

function exp2SeriesDefinition(sample, selected) {
  if (selected === "translation_x") return { value: sample?.relative_translation_error_m?.[0], unit: "mm", scale: 1000, label: "Relative translation X" };
  if (selected === "translation_y") return { value: sample?.relative_translation_error_m?.[1], unit: "mm", scale: 1000, label: "Relative translation Y" };
  if (selected === "translation_z") return { value: sample?.relative_translation_error_m?.[2], unit: "mm", scale: 1000, label: "Relative translation Z" };
  if (selected === "orientation") return { value: sample?.relative_orientation_error_deg, unit: "deg", scale: 1, label: "Relative orientation angle" };
  if (selected === "center_translation") return { value: sample?.center_translation_disagreement_magnitude_m, unit: "mm", scale: 1000, label: "Center translation disagreement" };
  if (selected === "center_orientation") return { value: sample?.center_orientation_disagreement_deg, unit: "deg", scale: 1, label: "Center orientation disagreement" };
  return { value: sample?.relative_translation_error_magnitude_m, unit: "mm", scale: 1000, label: "Relative translation magnitude" };
}

function drawExp2Series(analysis = state.loaded?.exp2_analysis || null) {
  const canvas = element("experimentExp2SeriesCanvas");
  if (!canvas) return;
  const context = canvas.getContext("2d");
  if (!context) return;
  const displayWidth = Math.max(320, Math.floor(canvas.clientWidth || 760));
  const displayHeight = Math.max(220, Math.floor(canvas.clientHeight || 300));
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const pixelWidth = Math.floor(displayWidth * ratio);
  const pixelHeight = Math.floor(displayHeight * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth; canvas.height = pixelHeight;
  }
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, displayWidth, displayHeight);
  context.fillStyle = "#020617";
  context.fillRect(0, 0, displayWidth, displayHeight);
  const selected = element("experimentExp2Series")?.value || "translation_magnitude";
  const samples = Array.isArray(analysis?.samples) ? analysis.samples : [];
  let seriesLabel = "EXP-2 series";
  let unit = "";
  const series = samples.map((sample) => {
    const definition = exp2SeriesDefinition(sample, selected);
    seriesLabel = definition.label; unit = definition.unit;
    return { x: sample?.time_from_start_s, y: Number.isFinite(definition.value) ? definition.value * definition.scale : null };
  }).filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
  const left = 58, right = displayWidth - 16, top = 28, bottom = displayHeight - 38;
  context.font = "12px sans-serif";
  if (!series.length) {
    context.fillStyle = "#94a3b8";
    context.fillText("No persisted EXP-2 synchronized samples for this run.", left, top + 24);
    return;
  }
  const minimumTime = Math.min(...series.map((point) => point.x));
  const maximumTime = Math.max(...series.map((point) => point.x));
  const timeSpan = Math.max(maximumTime - minimumTime, 1e-9);
  let minimum = Math.min(...series.map((point) => point.y));
  let maximum = Math.max(...series.map((point) => point.y));
  if (minimum === maximum) { minimum -= 1; maximum += 1; }
  const padding = Math.max((maximum - minimum) * 0.1, 1e-9);
  minimum -= padding; maximum += padding;
  const xScale = (value) => left + (right - left) * (value - minimumTime) / timeSpan;
  const yScale = (value) => top + (bottom - top) * (maximum - value) / (maximum - minimum);
  context.strokeStyle = "#334155"; context.lineWidth = 1;
  for (const value of [minimum, (minimum + maximum) / 2, maximum]) {
    context.beginPath(); context.moveTo(left, yScale(value)); context.lineTo(right, yScale(value)); context.stroke();
  }
  context.fillStyle = "#94a3b8";
  context.fillText(`${seriesLabel} [${unit}]`, 8, 16);
  context.fillText(formatNumber(maximum, 4), 8, top + 4);
  context.fillText(formatNumber(minimum, 4), 8, bottom + 4);
  context.fillText(`time ${formatNumber(minimumTime, 3)} … ${formatNumber(maximumTime, 3)} s`, left, displayHeight - 12);
  canvasLine(context, series, xScale, yScale, "#e879f9", 2);
}

function drawExp1ErrorSeries(analysis = state.loaded?.exp1_analysis || null) {
  const canvas = element("experimentExp1ErrorSeries");
  if (!canvas) return;
  const context = canvas.getContext("2d");
  if (!context) return;
  const displayWidth = Math.max(320, Math.floor(canvas.clientWidth || 760));
  const displayHeight = Math.max(220, Math.floor(canvas.clientHeight || 300));
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const pixelWidth = Math.floor(displayWidth * ratio);
  const pixelHeight = Math.floor(displayHeight * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, displayWidth, displayHeight);
  context.fillStyle = "#020617";
  context.fillRect(0, 0, displayWidth, displayHeight);
  const side = element("experimentSeriesSide")?.value || "left";
  const joint = Number(element("experimentSeriesJoint")?.value || 0);
  const series = (Array.isArray(analysis?.error_samples) ? analysis.error_samples : [])
    .filter((sample) => sample?.side === side
      && Number.isFinite(sample.time_from_start_s)
      && finiteVector(sample.error_joints_rad, 6))
    .map((sample) => ({ x: sample.time_from_start_s, y: sample.error_joints_rad[joint] }))
    .sort((first, second) => first.x - second.x);
  const left = 58;
  const right = displayWidth - 16;
  const top = 28;
  const bottom = displayHeight - 38;
  context.font = "12px sans-serif";
  if (!series.length) {
    context.fillStyle = "#94a3b8";
    context.fillText("No persisted usable aligned error samples for this joint.", left, top + 24);
    return;
  }
  const minimumTime = Math.min(...series.map((point) => point.x));
  const maximumTime = Math.max(...series.map((point) => point.x));
  const timeSpan = Math.max(maximumTime - minimumTime, 1e-9);
  const extent = Math.max(...series.map((point) => Math.abs(point.y)), 1e-9) * 1.12;
  const xScale = (value) => left + (right - left) * (value - minimumTime) / timeSpan;
  const yScale = (value) => top + (bottom - top) * (extent - value) / (2 * extent);
  context.strokeStyle = "#334155";
  context.lineWidth = 1;
  for (const value of [-extent, 0, extent]) {
    context.beginPath();
    context.moveTo(left, yScale(value));
    context.lineTo(right, yScale(value));
    context.stroke();
  }
  context.fillStyle = "#94a3b8";
  context.fillText(`${side.toUpperCase()} J${joint + 1} error [rad]`, 8, 16);
  context.fillText(formatNumber(extent, 5), 8, top + 4);
  context.fillText("0", 34, yScale(0) + 4);
  context.fillText(formatNumber(-extent, 5), 8, bottom + 4);
  context.fillText(`time ${formatNumber(minimumTime, 3)} … ${formatNumber(maximumTime, 3)} s`, left, displayHeight - 12);
  canvasLine(context, series, xScale, yScale, side === "left" ? "#2dd4bf" : "#f59e0b", 2);
}

function setAllFkJoints(values) {
  if (!state.hiddenFkModel || !finiteVector(values, 12)) return false;
  JOINT_NAMES.forEach((jointName, index) => {
    state.hiddenFkModel.setJointValue(jointName, values[index]);
  });
  state.hiddenFkModel.updateMatrixWorld(true);
  return true;
}

function fkMatrix(side, joints) {
  if (!state.hiddenFkModel || !finiteVector(joints, 6)) return null;
  const combined = new Array(12).fill(0);
  const offset = side === "left" ? 0 : 6;
  joints.forEach((value, index) => { combined[offset + index] = value; });
  if (!setAllFkJoints(combined)) return null;
  const terminal = state.hiddenFkModel.links?.[TERMINAL_LINKS[side]];
  return terminal ? terminal.matrixWorld.clone() : null;
}

function matrixTranslation(matrix) {
  if (!matrix) return null;
  const value = new THREE.Vector3().setFromMatrixPosition(matrix);
  return [value.x, value.y, value.z];
}

function poseMatrix(pose) {
  if (!pose || !finiteVector(pose.translation_m, 3) || !finiteVector(pose.rpy_rad, 3)) {
    return null;
  }
  const [x, y, z] = pose.translation_m;
  const [roll, pitch, yaw] = pose.rpy_rad;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  // Match the project's canonical ROS/URDF fixed-axis RPY exactly:
  // R = Rz(yaw) * Ry(pitch) * Rx(roll).  Keep this explicit rather than
  // relying on Three.js Euler order semantics.
  return new THREE.Matrix4().set(
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
    -sp, cp * sr, cp * cr, z,
    0, 0, 0, 1,
  );
}

function centerFromTip(worldTip, lockedPose) {
  const objectToTip = poseMatrix(lockedPose);
  if (!worldTip || !objectToTip) return null;
  const worldObject = worldTip.clone().multiply(objectToTip.clone().invert());
  return matrixTranslation(worldObject);
}

// CENTER-FROM-LEFT: W_T_L * inverse(O_T_L)
// CENTER-FROM-RIGHT: W_T_R * inverse(O_T_R)

function cartesianTranslation(cartesian, key) {
  const value = cartesian && cartesian[key] && cartesian[key].translation_m;
  return finiteVector(value, 3) ? [...value] : null;
}

function derivePlannedSample(sample) {
  const cartesian = sample?.cartesian;
  const result = {
    timeS: sample?.time_from_start_s,
    left: cartesianTranslation(cartesian, "left_target"),
    right: cartesianTranslation(cartesian, "right_target"),
    center: cartesianTranslation(cartesian, "object_pose"),
    leftSource: "PLAN_SNAPSHOT combined_object_samples.left_target.translation_m",
    rightSource: "PLAN_SNAPSHOT combined_object_samples.right_target.translation_m",
    centerSource: "PLAN_SNAPSHOT combined_object_samples.object_pose.translation_m",
  };
  if (!result.left) {
    result.left = matrixTranslation(fkMatrix("left", sample?.left?.joints_rad));
    result.leftSource = result.left
      ? "OFFLINE URDF FK FALLBACK FROM FROZEN PLANNED JOINTS" : "UNAVAILABLE";
  }
  if (!result.right) {
    result.right = matrixTranslation(fkMatrix("right", sample?.right?.joints_rad));
    result.rightSource = result.right
      ? "OFFLINE URDF FK FALLBACK FROM FROZEN PLANNED JOINTS" : "UNAVAILABLE";
  }
  if (!result.center) result.centerSource = "UNAVAILABLE — NO PLANNED object_pose";
  return result;
}

function deriveObservedSample(observation, lockedGrasp) {
  const matrices = {};
  const result = {
    timeS: observation?.timeline_s,
    left: null,
    right: null,
    centerFromLeft: null,
    centerFromRight: null,
    leftSource: "UNAVAILABLE",
    rightSource: "UNAVAILABLE",
  };
  for (const side of ["left", "right"]) {
    const actual = observation?.actual?.[side];
    if (actual?.valid === true && finiteVector(actual.joints_rad, 6)) {
      matrices[side] = fkMatrix(side, actual.joints_rad);
      result[side] = matrixTranslation(matrices[side]);
      result[`${side}Source`] = `${actual.source || "RECORDED ACTUAL JOINT SOURCE"} → ${FK_SOURCE}`;
    }
  }
  result.centerFromLeft = centerFromTip(matrices.left, lockedGrasp?.left);
  result.centerFromRight = centerFromTip(matrices.right, lockedGrasp?.right);
  return result;
}

function rebuildDerivedRun() {
  if (!state.loaded || !state.hiddenFkModel) {
    state.derived = null;
    if (state.loaded) {
      setMessage("Run loaded; waiting for hidden offline URDF FK model.");
    }
    return;
  }
  const raw = state.loaded.raw || {};
  const planned = Array.isArray(raw.planned?.samples) ? raw.planned.samples : [];
  const observed = Array.isArray(raw.observations) ? raw.observations : [];
  const lockedGrasp = state.loaded.manifest?.locked_grasp;
  state.derived = {
    planned: planned.map(derivePlannedSample),
    observed: observed.map((sample) => deriveObservedSample(sample, lockedGrasp)),
  };
  buildViewerPaths();
  const slider = element("experimentSampleSlider");
  const count = Math.max(planned.length, observed.length);
  state.selectedIndex = Math.min(state.selectedIndex, Math.max(0, count - 1));
  if (slider) {
    slider.max = String(Math.max(0, count - 1));
    slider.value = String(state.selectedIndex);
    slider.disabled = count === 0;
  }
  updateSelectedSample();
  drawJointSeries();
}

function initializeViewer() {
  const container = element("experimentPathViewer");
  if (!container) return;
  state.scene = new THREE.Scene();
  state.scene.background = new THREE.Color(0x07111f);
  state.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);
  state.camera.up.set(0, 0, 1);
  state.camera.position.set(2.4, -2.4, 1.8);
  state.renderer = new THREE.WebGLRenderer({ antialias: true });
  state.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  container.replaceChildren(state.renderer.domElement);
  state.controls = new OrbitControls(state.camera, state.renderer.domElement);
  state.controls.enableDamping = true;
  state.controls.dampingFactor = 0.08;
  // OrbitControls wheel zoom is intentionally disabled here. Browser/mouse
  // wheel delta sizes vary wildly, which made a single notch jump metres for
  // sub-metre experiment paths. Rotation/pan remain OrbitControls-owned; zoom
  // is handled by a bounded custom dolly below.
  state.controls.enableZoom = false;
  state.controls.zoomToCursor = false;
  state.controls.minDistance = 0.005;
  state.controls.maxDistance = 10;
  state.controls.addEventListener("change", syncViewerClipping);
  state.renderer.domElement.addEventListener("wheel", handleViewerWheel, { passive: false });
  state.pathRoot = new THREE.Group();
  state.markerRoot = new THREE.Group();
  state.scene.add(state.pathRoot, state.markerRoot);
  const grid = new THREE.GridHelper(6, 30, 0x155e75, 0x164e63);
  grid.rotation.x = Math.PI / 2;
  state.scene.add(grid, new THREE.AxesHelper(0.5));
  const resize = () => {
    const width = Math.max(1, container.clientWidth);
    const height = Math.max(1, container.clientHeight);
    state.camera.aspect = width / height;
    syncViewerClipping();
    state.camera.updateProjectionMatrix();
    state.renderer.setSize(width, height, false);
  };
  resize();
  window.addEventListener("resize", resize);
  if (typeof ResizeObserver !== "undefined") new ResizeObserver(resize).observe(container);
  const animate = () => {
    requestAnimationFrame(animate);
    state.controls.update();
    state.renderer.render(state.scene, state.camera);
  };
  animate();
}

function loadHiddenOfflineFkModel() {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.parseCollision = false;
  loader.load(
    EXPERIMENT_MODEL_URL,
    (robot) => {
      const missingJoints = JOINT_NAMES.filter((name) => !robot.joints?.[name]);
      const missingLinks = Object.values(TERMINAL_LINKS).filter((name) => !robot.links?.[name]);
      if (missingJoints.length || missingLinks.length) {
        state.fkStatus = "ERROR";
        setMessage(`Offline URDF FK contract missing: ${[...missingJoints, ...missingLinks].join(", ")}`, true);
        return;
      }
      robot.visible = false;
      robot.name = "EXP-0 hidden offline kinematic model — never actual robot";
      state.hiddenFkModel = robot;
      state.fkStatus = "READY";
      setAllFkJoints(new Array(12).fill(0));
      if (state.loaded) rebuildDerivedRun();
      else setMessage("Offline FK model ready. Select a recorded run.");
    },
    undefined,
    (error) => {
      state.fkStatus = "ERROR";
      setMessage(`Hidden offline URDF FK model failed to load: ${error.message || error}`, true);
    },
  );
}

function clearViewerPaths() {
  for (const object of [...state.paths.values(), ...state.markers.values()]) {
    object.removeFromParent();
    object.geometry?.dispose();
    object.material?.dispose();
  }
  state.paths.clear();
  state.markers.clear();
  state.viewerBounds = null;
}

function addPath(name, points) {
  if (!state.pathRoot || !state.markerRoot) return;
  const definition = SERIES[name];
  const valid = points.filter((point) => finiteVector(point, 3));
  if (!valid.length) return;
  const geometry = new THREE.BufferGeometry().setFromPoints(
    valid.map((point) => new THREE.Vector3(...point)),
  );
  const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: definition.color }));
  line.name = name;
  line.userData.series = definition;
  state.pathRoot.add(line);
  state.paths.set(name, line);
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 16, 12),
    new THREE.MeshBasicMaterial({ color: definition.color }),
  );
  marker.name = `${name} selected sample marker`;
  marker.userData.series = definition;
  state.markerRoot.add(marker);
  state.markers.set(name, marker);
}

function buildViewerPaths() {
  clearViewerPaths();
  if (!state.derived) return;
  addPath("plannedLeft", state.derived.planned.map((sample) => sample.left));
  addPath("plannedRight", state.derived.planned.map((sample) => sample.right));
  addPath("plannedCenter", state.derived.planned.map((sample) => sample.center));
  addPath("actualLeft", state.derived.observed.map((sample) => sample.left));
  addPath("actualRight", state.derived.observed.map((sample) => sample.right));
  addPath("actualCenterFromLeft", state.derived.observed.map((sample) => sample.centerFromLeft));
  addPath("actualCenterFromRight", state.derived.observed.map((sample) => sample.centerFromRight));
  updateVisibility();
  updateMarkers();
  fitAll();
}

function visibilitySettings() {
  return {
    planned: element("experimentTogglePlanned")?.checked !== false,
    actual: element("experimentToggleActual")?.checked !== false,
    left: element("experimentToggleLeft")?.checked !== false,
    right: element("experimentToggleRight")?.checked !== false,
    center: element("experimentToggleCenter")?.checked !== false,
  };
}

function seriesVisible(definition, settings) {
  if (definition.planned && !settings.planned) return false;
  if (definition.actual && !settings.actual) return false;
  if (definition.side && !settings[definition.side]) return false;
  if (definition.center && !settings.center) return false;
  return true;
}

function updateVisibility() {
  const settings = visibilitySettings();
  for (const object of [...state.paths.values(), ...state.markers.values()]) {
    object.visible = seriesVisible(object.userData.series, settings);
  }
  const bounds = boundsOfPaths();
  if (bounds && state.controls) configureViewerNavigation(bounds);
}

function markerPoint(name) {
  const index = state.selectedIndex;
  if (!state.derived) return null;
  const planned = state.derived.planned[index];
  const observed = state.derived.observed[index];
  return {
    plannedLeft: planned?.left,
    plannedRight: planned?.right,
    plannedCenter: planned?.center,
    actualLeft: observed?.left,
    actualRight: observed?.right,
    actualCenterFromLeft: observed?.centerFromLeft,
    actualCenterFromRight: observed?.centerFromRight,
  }[name];
}

function updateMarkers() {
  for (const [name, marker] of state.markers.entries()) {
    const point = markerPoint(name);
    if (finiteVector(point, 3)) {
      marker.position.set(...point);
      marker.userData.hasPoint = true;
    } else {
      marker.userData.hasPoint = false;
    }
  }
  updateVisibility();
  for (const marker of state.markers.values()) {
    marker.visible = marker.visible && marker.userData.hasPoint === true;
  }
}

function boundsOfPaths() {
  const box = new THREE.Box3();
  for (const path of state.paths.values()) {
    if (path.visible) box.expandByObject(path);
  }
  if (box.isEmpty()) return null;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  return { box, center, radius: Math.max(0.25, size.length() * 0.5) };
}

function wheelDeltaPixels(event) {
  if (!event || !Number.isFinite(event.deltaY)) return 0;
  if (event.deltaMode === 1) return event.deltaY * 16;
  if (event.deltaMode === 2) return event.deltaY * Math.max(240, window.innerHeight || 800);
  return event.deltaY;
}

function handleViewerWheel(event) {
  if (!state.camera || !state.controls) return;
  const deltaPixels = wheelDeltaPixels(event);
  if (!deltaPixels) return;
  event.preventDefault();

  const offset = state.camera.position.clone().sub(state.controls.target);
  const distance = offset.length();
  if (!Number.isFinite(distance) || distance <= 0) return;

  // tanh caps even very large wheel deltas; one normal mouse notch changes
  // distance by only a few percent. Trackpads retain fine-grained control.
  const normalized = Math.tanh(deltaPixels / 120);
  const scale = Math.exp(normalized * 0.035);
  const nextDistance = THREE.MathUtils.clamp(
    distance * scale,
    state.controls.minDistance,
    state.controls.maxDistance,
  );
  offset.setLength(nextDistance);
  state.camera.position.copy(state.controls.target).add(offset);
  syncViewerClipping();
  state.controls.update();
}

function viewerFitDistance(bounds) {
  if (!state.camera) return bounds.radius * 3.2;
  const verticalHalfFov = THREE.MathUtils.degToRad(state.camera.fov * 0.5);
  const aspect = Math.max(0.05, Number(state.camera.aspect) || 1);
  const horizontalHalfFov = Math.atan(Math.tan(verticalHalfFov) * aspect);
  const limitingHalfFov = Math.max(0.01, Math.min(verticalHalfFov, horizontalHalfFov));
  // Sphere-fit is stable for every view preset and leaves a little breathing room.
  return Math.max(bounds.radius * 2.4, (bounds.radius / Math.sin(limitingHalfFov)) * 1.18);
}

function configureViewerNavigation(bounds, fitDistance = null) {
  if (!state.camera || !state.controls) return;
  const radius = Math.max(0.01, bounds.radius);
  const distance = Number.isFinite(fitDistance)
    ? fitDistance : state.camera.position.distanceTo(state.controls.target);
  state.viewerBounds = { center: bounds.center.clone(), radius };
  state.controls.minDistance = Math.max(0.003, radius * 0.02, distance * 0.02);
  // Keep zoom-out useful but bounded to the experiment itself. The previous
  // 25 m floor dwarfed a ~0.2 m path and made a single wheel action appear to
  // throw the scene away.
  state.controls.maxDistance = Math.max(2, radius * 30, distance * 8);
  syncViewerClipping();
}

function syncViewerClipping() {
  if (!state.camera || !state.controls) return;
  const radius = Math.max(0.01, state.viewerBounds?.radius || 1);
  const distance = Math.max(1e-6, state.camera.position.distanceTo(state.controls.target));
  // Near follows the current dolly distance, while far always encloses the
  // complete OrbitControls range. This prevents the whole scene vanishing
  // during aggressive wheel/trackpad zoom in either direction.
  const near = Math.max(0.00001, Math.min(radius / 1000, distance / 500));
  const far = Math.max(100, state.controls.maxDistance * 1.5, distance + radius * 100);
  if (Math.abs(state.camera.near - near) > 1e-9 || Math.abs(state.camera.far - far) > 1e-6) {
    state.camera.near = near;
    state.camera.far = far;
    state.camera.updateProjectionMatrix();
  }
}

function applyView(direction, up) {
  if (!state.camera || !state.controls) return;
  const bounds = boundsOfPaths() || {
    center: new THREE.Vector3(),
    radius: 1,
  };
  const distance = viewerFitDistance(bounds);
  state.camera.up.copy(up);
  state.camera.position.copy(bounds.center).add(
    direction.clone().normalize().multiplyScalar(distance),
  );
  state.controls.target.copy(bounds.center);
  configureViewerNavigation(bounds, distance);
  state.camera.updateProjectionMatrix();
  state.controls.update();
}

function fitAll() {
  applyView(new THREE.Vector3(1.4, -1.4, 1.0), new THREE.Vector3(0, 0, 1));
}

function setViewPreset(preset) {
  const presets = {
    "3D": [new THREE.Vector3(1.4, -1.4, 1.0), new THREE.Vector3(0, 0, 1)],
    "TOP_XY": [new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 1, 0)],
    "FRONT_XZ": [new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0, 1)],
    "SIDE_YZ": [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 1)],
  };
  const [direction, up] = presets[preset] || presets["3D"];
  applyView(direction, up);
}

function sourcePointText(point, source) {
  return `${formatPoint(point)} — ${source || "UNAVAILABLE"}`;
}

function torqueText(observation, side) {
  const torque = observation?.torque?.[side];
  const status = torque?.status || "MISSING";
  const values = finiteVector(torque?.values, 6)
    ? `[${torque.values.map((value) => formatNumber(value, 3)).join(", ")}] N·m`
    : "NO VALUES";
  const age = Number.isFinite(torque?.age_ms) ? `; age=${torque.age_ms} ms` : "";
  return `${side.toUpperCase()}: ${status}; ${values}${age}`;
}

function updateSelectedSample() {
  const raw = state.loaded?.raw || {};
  const plannedRaw = raw.planned?.samples?.[state.selectedIndex];
  const observedRaw = raw.observations?.[state.selectedIndex];
  const planned = state.derived?.planned?.[state.selectedIndex];
  const observed = state.derived?.observed?.[state.selectedIndex];
  const count = Math.max(
    raw.planned?.samples?.length || 0,
    raw.observations?.length || 0,
  );
  setText("experimentSelectedSample", count ? `${state.selectedIndex + 1} / ${count}` : "0 / 0");
  setText(
    "experimentSelectedTime",
    plannedRaw ? `${formatNumber(plannedRaw.time_from_start_s, 3)} s planned`
      : observedRaw ? `${formatNumber(observedRaw.timeline_s, 3)} s observed` : "N/A",
  );
  setText("experimentSelectedPlannedLeft", sourcePointText(planned?.left, planned?.leftSource));
  setText("experimentSelectedPlannedRight", sourcePointText(planned?.right, planned?.rightSource));
  setText("experimentSelectedPlannedCenter", sourcePointText(planned?.center, planned?.centerSource));
  setText("experimentSelectedActualLeft", sourcePointText(observed?.left, observed?.leftSource));
  setText("experimentSelectedActualRight", sourcePointText(observed?.right, observed?.rightSource));
  setText(
    "experimentSelectedCenterLeft",
    sourcePointText(observed?.centerFromLeft, "LOCKED GRASP INVERSE FROM LEFT — DISTINCT; NOT AVERAGED"),
  );
  setText(
    "experimentSelectedCenterRight",
    sourcePointText(observed?.centerFromRight, "LOCKED GRASP INVERSE FROM RIGHT — DISTINCT; NOT AVERAGED"),
  );
  setText(
    "experimentSelectedTorque",
    `${torqueText(observedRaw, "left")} / ${torqueText(observedRaw, "right")}`,
  );
  const exp2Samples = Array.isArray(state.loaded?.exp2_analysis?.samples)
    ? state.loaded.exp2_analysis.samples : [];
  const sampleTime = observedRaw ? observedTimeS(observedRaw, raw)
    : Number.isFinite(plannedRaw?.time_from_start_s) ? plannedRaw.time_from_start_s : null;
  let exp2Nearest = null;
  if (Number.isFinite(sampleTime) && exp2Samples.length) {
    exp2Nearest = exp2Samples.reduce((best, item) => (
      !best || Math.abs(item.time_from_start_s - sampleTime) < Math.abs(best.time_from_start_s - sampleTime)
        ? item : best
    ), null);
  }
  setText(
    "experimentRelativeErrorSeries",
    exp2Nearest
      ? `Δp=${exp2Mm(exp2Nearest.relative_translation_error_magnitude_m)} mm; Δθ=${formatNumber(exp2Nearest.relative_orientation_error_deg, 4)}°; Center Δ=${exp2Mm(exp2Nearest.center_translation_disagreement_magnitude_m)} mm — MODEL-BASED`
      : RELATIVE_ERROR_RESERVED,
  );
  updateMarkers();
}

function canvasLine(context, points, xScale, yScale, color, width = 1.5) {
  const valid = points.filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
  if (!valid.length) return;
  context.beginPath();
  context.strokeStyle = color;
  context.lineWidth = width;
  valid.forEach((point, index) => {
    const x = xScale(point.x);
    const y = yScale(point.y);
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.stroke();
}

function seriesRange(series, fallback = [-1, 1]) {
  const values = series.flatMap((items) => items.map((item) => item.y)).filter(Number.isFinite);
  if (!values.length) return fallback;
  let minimum = Math.min(...values);
  let maximum = Math.max(...values);
  if (minimum === maximum) {
    minimum -= 1;
    maximum += 1;
  }
  const pad = (maximum - minimum) * 0.08;
  return [minimum - pad, maximum + pad];
}

function observedTimeS(sample, run) {
  if (Number.isFinite(sample?.timeline_s)) return sample.timeline_s;
  const startNs = run?.execution?.start_time_unix_ns;
  const receiveTimes = ["left", "right"]
    .map((side) => sample?.actual?.[side]?.received_at_ms)
    .filter(Number.isFinite);
  if (Number.isInteger(startNs) && startNs > 0 && receiveTimes.length) {
    return (Math.min(...receiveTimes) - startNs / 1e6) / 1000;
  }
  return null;
}

function drawJointSeries() {
  const canvas = element("experimentJointSeries");
  if (!canvas) return;
  const context = canvas.getContext("2d");
  if (!context) return;
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#020617";
  context.fillRect(0, 0, width, height);
  const raw = state.loaded?.raw || {};
  const planned = raw.planned?.samples || [];
  const observed = raw.observations || [];
  const side = element("experimentSeriesSide")?.value || "left";
  const joint = Number(element("experimentSeriesJoint")?.value || 0);
  const plannedSeries = planned.map((sample) => ({
    x: sample?.time_from_start_s,
    y: sample?.[side]?.joints_rad?.[joint],
  }));
  const commandedSeries = observed.map((sample) => ({
    x: sample?.commanded?.[side]?.time_from_start_s,
    y: sample?.commanded?.[side]?.joints_rad?.[joint],
  }));
  const actualSeries = observed.map((sample) => ({
    x: observedTimeS(sample, raw),
    y: sample?.actual?.[side]?.joints_rad?.[joint],
  }));
  const torqueSeries = observed.map((sample) => ({
    x: observedTimeS(sample, raw),
    y: sample?.torque?.[side]?.values?.[joint],
  }));
  const timeValues = [
    ...plannedSeries, ...commandedSeries, ...actualSeries, ...torqueSeries,
  ].map((point) => point.x).filter(Number.isFinite);
  const minimumTime = timeValues.length ? Math.min(...timeValues) : 0;
  const maximumTime = timeValues.length ? Math.max(...timeValues) : 1;
  const timeSpan = Math.max(maximumTime - minimumTime, 1e-9);
  const left = 54;
  const right = width - 18;
  const xScale = (value) => left + (right - left) * (value - minimumTime) / timeSpan;
  const [jointMinimum, jointMaximum] = seriesRange([
    plannedSeries, commandedSeries, actualSeries,
  ]);
  const jointY = (value) => 18 + (168 - 18) * (jointMaximum - value)
    / (jointMaximum - jointMinimum);
  const [torqueMinimum, torqueMaximum] = seriesRange([torqueSeries]);
  const torqueY = (value) => 187 + (248 - 187) * (torqueMaximum - value)
    / (torqueMaximum - torqueMinimum);
  context.strokeStyle = "#334155";
  context.lineWidth = 1;
  for (const y of [18, 93, 168, 187, 248, 260, 291]) {
    context.beginPath(); context.moveTo(left, y); context.lineTo(right, y); context.stroke();
  }
  context.fillStyle = "#94a3b8";
  context.font = "12px sans-serif";
  context.fillText(`${side.toUpperCase()} J${joint + 1} [rad]`, 8, 16);
  context.fillText(formatNumber(jointMaximum, 3), 8, 30);
  context.fillText(formatNumber(jointMinimum, 3), 8, 168);
  context.fillText("Torque [N·m]", 8, 184);
  context.fillText(RELATIVE_ERROR_RESERVED, left + 8, 281);
  canvasLine(context, plannedSeries, xScale, jointY, "#22d3ee", 2);
  canvasLine(context, commandedSeries, xScale, jointY, "#a78bfa", 1.6);
  canvasLine(context, actualSeries, xScale, jointY, "#22c55e", 1.6);
  if (element("experimentTorqueToggle")?.checked !== false) {
    canvasLine(context, torqueSeries, xScale, torqueY, "#f59e0b", 1.6);
  }
  const selectedPlanned = planned[state.selectedIndex]?.time_from_start_s;
  const selectedObserved = observedTimeS(observed[state.selectedIndex], raw);
  const selectedTime = Number.isFinite(selectedPlanned) ? selectedPlanned : selectedObserved;
  if (Number.isFinite(selectedTime)) {
    const selectedX = xScale(selectedTime);
    context.strokeStyle = "#f8fafc";
    context.setLineDash([4, 4]);
    context.beginPath(); context.moveTo(selectedX, 18); context.lineTo(selectedX, 291); context.stroke();
    context.setLineDash([]);
  }
  context.fillStyle = "#cbd5e1";
  context.fillText("planned", right - 265, 16);
  context.fillStyle = "#a78bfa";
  context.fillText("commanded", right - 195, 16);
  context.fillStyle = "#22c55e";
  context.fillText("actual", right - 105, 16);
  context.fillStyle = "#94a3b8";
  context.fillText(
    `time ${formatNumber(minimumTime, 3)} … ${formatNumber(maximumTime, 3)} s`,
    left,
    298,
  );
}

function bindControls() {
  const clicks = {
    experimentArmRecorder: armRecorder,
    experimentDisarmRecorder: disarmRecorder,
    experimentRefreshRecorder: () => refreshRecorder(),
    experimentRefreshRuns: () => refreshRuns(),
    experimentLoadRun: () => loadRun(),
    experimentRelabelRun: relabelRun,
    experimentDeleteRun: deleteRun,
    experimentExportRun: exportRun,
    experimentExp1Analyze: analyzeExp1,
    experimentExp2Analyze: analyzeExp2,
    experimentView3D: () => setViewPreset("3D"),
    experimentViewTopXY: () => setViewPreset("TOP_XY"),
    experimentViewFrontXZ: () => setViewPreset("FRONT_XZ"),
    experimentViewSideYZ: () => setViewPreset("SIDE_YZ"),
    experimentFitAll: fitAll,
  };
  Object.entries(clicks).forEach(([id, handler]) => element(id)?.addEventListener("click", handler));
  for (const id of [
    "experimentTogglePlanned", "experimentToggleActual", "experimentToggleLeft",
    "experimentToggleRight", "experimentToggleCenter",
  ]) {
    element(id)?.addEventListener("change", updateVisibility);
  }
  element("experimentSampleSlider")?.addEventListener("input", (event) => {
    state.selectedIndex = Number(event.target.value);
    updateSelectedSample();
    drawJointSeries();
  });
  element("experimentSeriesSide")?.addEventListener("change", () => {
    drawJointSeries();
    drawExp1ErrorSeries();
  });
  element("experimentSeriesJoint")?.addEventListener("change", () => {
    drawJointSeries();
    drawExp1ErrorSeries();
  });
  element("experimentTorqueToggle")?.addEventListener("change", drawJointSeries);
  element("experimentExp2Series")?.addEventListener("change", () => drawExp2Series());
  window.addEventListener("resize", () => { drawExp1ErrorSeries(); drawExp2Series(); });
}

function initialize() {
  if (!element("digitalTwinExperimentPanel")) return;
  bindControls();
  try {
    initializeViewer();
  } catch (error) {
    setMessage(
      `Experimental WebGL path viewer unavailable: ${error.message || error}. Run Library controls remain available.`,
      true,
    );
  }
  loadHiddenOfflineFkModel();
  renderRecorder({ state: "IDLE", sample_counts: {}, torque_availability: {} });
  renderExp1Analysis(null);
  renderExp2Analysis(null);
  drawJointSeries();
  refreshRuns({ quiet: true });
  refreshRecorder({ quiet: true });
  window.setInterval(() => refreshRecorder({ quiet: true }), 1000);
  window.dualArmExperimentViewer = Object.freeze({
    refreshRuns,
    loadRun,
    fitAll,
    setViewPreset,
    getState: () => ({
      recorder: cloneJson(state.recorder),
      loadedRunId: state.loaded?.manifest?.run_id || null,
      fkStatus: state.fkStatus,
      selectedIndex: state.selectedIndex,
      exp1Status: state.loaded?.exp1_analysis?.status || "NOT ANALYZED",
      exp2Status: state.loaded?.exp2_analysis?.status || "NOT ANALYZED",
      relativeErrorSeries: RELATIVE_ERROR_RESERVED,
    }),
  });
}

initialize();
