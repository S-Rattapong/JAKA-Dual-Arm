// Read-only calibration/model revision transport and fail-closed comparison.
export const WORLD_CALIBRATION_ENDPOINT = "/api/digital-twin/world-calibration";
export const WEB_MODEL_METADATA_URL = (
  "/digital-twin/assets/dual_jaka_a12_web.metadata.json"
);
export const MODEL_CALIBRATION_MISMATCH = (
  "MODEL/CALIBRATION REVISION MISMATCH"
);

function requireNonEmptyString(value, field) {
  if (typeof value !== "string" || value.trim().length === 0) {
    throw new TypeError(`${field} must be a non-empty string`);
  }
  return value.trim();
}

export function normalizeCalibrationRevisionSources(backend, modelMetadata) {
  if (!backend || backend.ok !== true || !backend.calibration) {
    throw new TypeError("World calibration response is malformed");
  }
  if (!modelMetadata || typeof modelMetadata !== "object") {
    throw new TypeError("Web model calibration metadata is malformed");
  }
  const backendRevision = requireNonEmptyString(
    backend.calibration.revision,
    "backend calibration revision",
  );
  const modelRevision = requireNonEmptyString(
    modelMetadata.calibration_revision,
    "Web model calibration revision",
  );
  const matches = backendRevision === modelRevision;
  return {
    status: matches ? "MATCH" : MODEL_CALIBRATION_MISMATCH,
    matches,
    planningReady: matches,
    backendRevision,
    modelRevision,
    calibrationState: requireNonEmptyString(
      backend.calibration.calibration_state,
      "calibration state",
    ),
    physicallyCalibrated: backend.calibration.physically_calibrated === true,
    error: matches ? null : MODEL_CALIBRATION_MISMATCH,
  };
}

async function fetchJson(url, label, fetchImpl) {
  const response = await fetchImpl(url, { method: "GET", cache: "no-store" });
  let payload;
  try {
    payload = await response.json();
  } catch (_error) {
    throw new Error(`${label} returned invalid JSON`);
  }
  if (!response.ok) {
    throw new Error(`${label} HTTP ${response.status}`);
  }
  return payload;
}

export async function requestCalibrationRevisionState(fetchImpl = fetch) {
  if (typeof fetchImpl !== "function") {
    throw new TypeError("World calibration transport is unavailable");
  }
  const [backend, modelMetadata] = await Promise.all([
    fetchJson(WORLD_CALIBRATION_ENDPOINT, "World calibration", fetchImpl),
    fetchJson(WEB_MODEL_METADATA_URL, "Web model metadata", fetchImpl),
  ]);
  return normalizeCalibrationRevisionSources(backend, modelMetadata);
}
