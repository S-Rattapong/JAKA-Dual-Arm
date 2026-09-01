// Pure Web contract for Phase-4A PLAN/VALIDATION ONLY. No execution endpoint.

export const PHASE4_VALIDATION_ENDPOINT = (
  "/api/digital-twin/validate-phase4-trajectory"
);
export const PHASE4_DEFAULT_POSITION_TOLERANCE_M = 0.001;
export const PHASE4_DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.001;

function finiteJointArray(value, label) {
  if (!Array.isArray(value) || value.length !== 6) {
    throw new TypeError(`${label} must contain exactly six joint values`);
  }
  if (!value.every((item) => typeof item === "number" && Number.isFinite(item))) {
    throw new TypeError(`${label} must contain finite numbers`);
  }
  return [...value];
}

function nonnegativeTolerance(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value) || value < 0) {
    throw new TypeError(`${label} must be a finite non-negative number`);
  }
  return value;
}

export function normalizePhase4ValidationStartState(value) {
  if (!value || typeof value !== "object") {
    throw new TypeError("Phase-4 validation start state must be an object");
  }
  if (typeof value.source !== "string" || value.source.trim().length === 0) {
    throw new TypeError("Phase-4 validation start source must be non-empty");
  }
  return {
    left: finiteJointArray(value.left, "start_state.left"),
    right: finiteJointArray(value.right, "start_state.right"),
    source: value.source.trim(),
  };
}

export function buildPhase4ValidationRequest({
  plan,
  startState = null,
  positionToleranceM = PHASE4_DEFAULT_POSITION_TOLERANCE_M,
  orientationToleranceRad = PHASE4_DEFAULT_ORIENTATION_TOLERANCE_RAD,
}) {
  if (!plan || typeof plan !== "object" || plan.ok !== true) {
    throw new TypeError("A successful Phase-3 Global Plan is required");
  }
  return {
    plan,
    start_state: startState === null
      ? null
      : normalizePhase4ValidationStartState(startState),
    position_tolerance_m: nonnegativeTolerance(
      positionToleranceM,
      "positionToleranceM",
    ),
    orientation_tolerance_rad: nonnegativeTolerance(
      orientationToleranceRad,
      "orientationToleranceRad",
    ),
  };
}

export async function requestPhase4TrajectoryValidation(
  request,
  fetchImpl = fetch,
) {
  if (typeof fetchImpl !== "function") {
    throw new TypeError("Phase-4 validation transport is unavailable");
  }
  const response = await fetchImpl(PHASE4_VALIDATION_ENDPOINT, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(request),
  });
  let payload;
  try {
    payload = await response.json();
  } catch (_error) {
    throw new Error("Phase-4 validation returned invalid JSON");
  }
  if (!response.ok) {
    throw new Error(payload.detail || `Phase-4 validation HTTP ${response.status}`);
  }
  if (!payload || typeof payload !== "object" || !payload.validation) {
    throw new Error("Phase-4 validation response is malformed");
  }
  return payload.validation;
}
