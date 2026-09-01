// Pure Phase-4B Web transport. PLAN/VALIDATION ONLY; no execution endpoint.

export const PHASE4B_COLLISION_ENDPOINT = (
  "/api/digital-twin/validate-phase4-collision"
);

export function buildPhase4CollisionRequest(plan, maxJointStepRad = 0.05) {
  if (!plan || typeof plan !== "object" || plan.ok !== true) {
    throw new TypeError("A successful Phase-3 Global Plan is required");
  }
  if (
    typeof maxJointStepRad !== "number"
    || !Number.isFinite(maxJointStepRad)
    || maxJointStepRad <= 0
  ) {
    throw new TypeError("maxJointStepRad must be a finite number greater than zero");
  }
  return { plan, max_joint_step_rad: maxJointStepRad };
}

export async function requestPhase4CollisionValidation(request, fetchImpl = fetch) {
  if (typeof fetchImpl !== "function") {
    throw new TypeError("Phase-4B validation transport is unavailable");
  }
  const response = await fetchImpl(PHASE4B_COLLISION_ENDPOINT, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(request),
  });
  let payload;
  try {
    payload = await response.json();
  } catch (_error) {
    throw new Error("Phase-4B validation returned invalid JSON");
  }
  if (!response.ok) {
    throw new Error(payload.detail || `Phase-4B validation HTTP ${response.status}`);
  }
  if (!payload || typeof payload !== "object" || !payload.validation) {
    throw new Error("Phase-4B validation response is malformed");
  }
  return payload.validation;
}
