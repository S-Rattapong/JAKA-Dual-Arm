// Read-only/state-only transport for the customizable rigid grasp lifecycle.
export const GRASP_CONFIGURATION_ENDPOINT = (
  "/api/digital-twin/grasp-configuration"
);
export const GRASP_LOCK_ENDPOINT = (
  "/api/digital-twin/grasp-configuration/lock"
);
export const GRASP_UNLOCK_ENDPOINT = (
  "/api/digital-twin/grasp-configuration/unlock"
);

function finiteVector3(value, label) {
  if (!Array.isArray(value) || value.length !== 3) {
    throw new TypeError(`${label} must contain exactly three finite numbers`);
  }
  return value.map((component, index) => {
    if (typeof component !== "number" || !Number.isFinite(component)) {
      throw new TypeError(`${label}[${index}] must be a finite number`);
    }
    return component;
  });
}

function normalizePose(value, label) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError(`${label} must be an object`);
  }
  return {
    translationM: finiteVector3(value.translation_m, `${label}.translation_m`),
    rpyRad: finiteVector3(value.rpy_rad, `${label}.rpy_rad`),
  };
}

function normalizeContent(value, label) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError(`${label} must contain left and right poses`);
  }
  return {
    left: normalizePose(value.left, `${label}.left`),
    right: normalizePose(value.right, `${label}.right`),
  };
}

function nonEmptyString(value, label) {
  if (typeof value !== "string" || !value.trim()) {
    throw new TypeError(`${label} must be a non-empty string`);
  }
  return value.trim();
}

export function normalizeGraspConfigurationResponse(payload) {
  if (!payload || payload.ok !== true || !payload.grasp) {
    throw new TypeError("Rigid grasp state response is malformed");
  }
  const grasp = payload.grasp;
  if (!["GRASP_UNLOCKED", "GRASP_LOCKED"].includes(grasp.state)) {
    throw new TypeError("Rigid grasp state is invalid");
  }
  const draft = normalizeContent(grasp.draft, "draft");
  let lockedSnapshot = null;
  if (grasp.locked_snapshot !== null) {
    const locked = normalizeContent(grasp.locked_snapshot, "locked_snapshot");
    lockedSnapshot = {
      ...locked,
      contentRevision: nonEmptyString(
        grasp.locked_snapshot.content_revision,
        "locked content revision",
      ),
      lockRevision: nonEmptyString(
        grasp.locked_snapshot.lock_revision,
        "lock revision",
      ),
      lockGeneration: grasp.locked_snapshot.lock_generation,
    };
  }
  if (grasp.state === "GRASP_LOCKED" && !lockedSnapshot) {
    throw new TypeError("Locked rigid grasp snapshot is missing");
  }
  if (grasp.state === "GRASP_UNLOCKED" && lockedSnapshot) {
    throw new TypeError("Unlocked rigid grasp must not expose a locked snapshot");
  }
  if (!Number.isInteger(grasp.lock_generation) || grasp.lock_generation < 0) {
    throw new TypeError("Rigid grasp lock generation is invalid");
  }
  return {
    state: grasp.state,
    draft,
    draftSource: nonEmptyString(grasp.draft_source, "draft source"),
    draftContentRevision: nonEmptyString(
      grasp.draft_content_revision,
      "draft content revision",
    ),
    lockedSnapshot,
    authoritativeRevision: grasp.authoritative_revision,
    lockGeneration: grasp.lock_generation,
    rotationConvention: nonEmptyString(
      grasp.rotation_convention,
      "rotation convention",
    ),
    units: { ...grasp.units },
    semantic: nonEmptyString(grasp.semantic, "grasp semantic"),
    physicallyCalibrated: grasp.physically_calibrated === true,
    plannerIntegration: nonEmptyString(
      grasp.planner_integration,
      "planner integration",
    ),
    worldCalibrationRevision: nonEmptyString(
      grasp.world_calibration_revision,
      "world calibration revision",
    ),
    error: null,
  };
}

function lockPayload(draft) {
  const normalized = normalizeContent({
    left: {
      translation_m: draft.left.translationM,
      rpy_rad: draft.left.rpyRad,
    },
    right: {
      translation_m: draft.right.translationM,
      rpy_rad: draft.right.rpyRad,
    },
  }, "draft");
  return {
    left: {
      translation_m: normalized.left.translationM,
      rpy_rad: normalized.left.rpyRad,
    },
    right: {
      translation_m: normalized.right.translationM,
      rpy_rad: normalized.right.rpyRad,
    },
  };
}

async function requestJson(url, options, fetchImpl) {
  const response = await fetchImpl(url, options);
  let payload;
  try {
    payload = await response.json();
  } catch (_error) {
    throw new Error("Rigid grasp state endpoint returned invalid JSON");
  }
  if (!response.ok) {
    throw new Error(payload.detail || `Rigid grasp state HTTP ${response.status}`);
  }
  return normalizeGraspConfigurationResponse(payload);
}

export function requestGraspConfiguration(fetchImpl = fetch) {
  return requestJson(
    GRASP_CONFIGURATION_ENDPOINT,
    { method: "GET", cache: "no-store" },
    fetchImpl,
  );
}

export function requestLockGraspConfiguration(draft, fetchImpl = fetch) {
  return requestJson(
    GRASP_LOCK_ENDPOINT,
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(lockPayload(draft)),
    },
    fetchImpl,
  );
}

export function requestUnlockGraspConfiguration(fetchImpl = fetch) {
  return requestJson(
    GRASP_UNLOCK_ENDPOINT,
    { method: "POST" },
    fetchImpl,
  );
}
