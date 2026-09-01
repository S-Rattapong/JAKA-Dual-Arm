// Phase-4C report transport and pure fail-closed Web gate. No robot command.
import {
  buildPhase4ValidationRequest,
} from "./digital_twin_phase4_validation.js";
import {
  buildPhase4CollisionRequest,
} from "./digital_twin_phase4_collision.js";

export const PHASE4_UNIFIED_VALIDATION_ENDPOINT = (
  "/api/digital-twin/validate-phase4"
);
export const PHASE4_UNIFIED_REPORT_VERSION = (
  "PHASE4C_UNIFIED_VALIDATION_V3_GRASP_CALIBRATION_IDENTITY"
);

export const PHASE4_REQUIRED_CHECKS = Object.freeze([
  "plan_identity",
  "authoritative_inputs",
  "ik_complete",
  "joint_position_limits",
  "start_transition",
  "joint_continuity",
  "joint_jump",
  "fk_target",
  "relative_pose",
  "fixed_grasp",
  "common_timeline",
  "velocity",
  "self_collision",
  "inter_arm_collision",
  "sampled_path_collision",
]);

export const PHASE4_DEFERRED_CHECKS = Object.freeze([
  Object.freeze({
    check: "object_collision",
    roadmapItem: "P4.5",
    reason: "DEFERRED UNTIL MAIN PROJECT COMPLETION",
  }),
  Object.freeze({
    check: "environment_collision",
    roadmapItem: "P4.6",
    reason: "DEFERRED UNTIL MAIN PROJECT COMPLETION",
  }),
  Object.freeze({
    check: "acceleration",
    roadmapItem: "P4.20",
    reason: "ACCELERATION LIMIT VALIDATION DEFERRED; DIAGNOSTIC RETAINED",
  }),
]);

export const PHASE4_CURRENT_SCOPE_READY_SEMANTIC = (
  "READY UNDER CURRENT PROJECT VALIDATION SCOPE — "
  + "DEFERRED CHECKS REMAIN UNVALIDATED"
);

export function buildPhase4UnifiedValidationRequest({
  plan,
  startState = null,
  positionToleranceM = 0.001,
  orientationToleranceRad = 0.001,
  maxJointStepRad = 0.05,
}) {
  const trajectory = buildPhase4ValidationRequest({
    plan,
    startState,
    positionToleranceM,
    orientationToleranceRad,
  });
  const collision = buildPhase4CollisionRequest(plan, maxJointStepRad);
  return {
    ...trajectory,
    max_joint_step_rad: collision.max_joint_step_rad,
  };
}

export function normalizePhase4UnifiedReport(value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError("Phase-4C report must be an object");
  }
  if (typeof value.plan_fingerprint !== "string" || !value.plan_fingerprint) {
    throw new TypeError("Phase-4C report plan fingerprint is missing");
  }
  if (!["PASS", "FAIL", "INCOMPLETE", "ERROR"].includes(value.overall_status)) {
    throw new TypeError("Phase-4C report overall status is invalid");
  }
  if (!value.checks || typeof value.checks !== "object") {
    throw new TypeError("Phase-4C report checks are missing");
  }
  if (!value.execution_gate || typeof value.execution_gate !== "object") {
    throw new TypeError("Phase-4C report execution gate is missing");
  }
  if (
    value.report_version !== PHASE4_UNIFIED_REPORT_VERSION
    || !value.scope_policy
    || JSON.stringify(value.scope_policy.required_checks) !== JSON.stringify(
      PHASE4_REQUIRED_CHECKS,
    )
    || !Array.isArray(value.scope_policy.deferred_checks)
  ) {
    throw new TypeError("Phase-4C report scope policy is invalid");
  }
  return JSON.parse(JSON.stringify(value));
}

export async function requestPhase4UnifiedValidation(
  request,
  fetchImpl = fetch,
) {
  if (typeof fetchImpl !== "function") {
    throw new TypeError("Phase-4C validation transport is unavailable");
  }
  const response = await fetchImpl(PHASE4_UNIFIED_VALIDATION_ENDPOINT, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(request),
  });
  let payload;
  try {
    payload = await response.json();
  } catch (_error) {
    throw new Error("Phase-4C validation returned invalid JSON");
  }
  if (!response.ok) {
    throw new Error(payload.detail || `Phase-4C validation HTTP ${response.status}`);
  }
  if (!payload || payload.ok !== true || !payload.report) {
    throw new Error("Phase-4C validation response is malformed");
  }
  return normalizePhase4UnifiedReport(payload.report);
}

export function phase4ExecutionGateState({
  report = null,
  currentPlanFingerprint = null,
  validationRunning = false,
  stale = false,
} = {}) {
  const reasons = [];
  if (validationRunning) reasons.push("VALIDATION_RUNNING");
  if (!report) {
    reasons.push("UNIFIED_VALIDATION_REPORT_MISSING");
  } else {
    if (report.overall_status !== "PASS") {
      reasons.push(`OVERALL_STATUS_${report.overall_status || "MISSING"}`);
    }
    if (PHASE4_REQUIRED_CHECKS.some((name) => (
      !report.checks[name] || report.checks[name].status !== "PASS"
    ))) {
      reasons.push("REQUIRED_VALIDATION_NOT_ALL_PASS");
    }
    if (
      report.report_version !== PHASE4_UNIFIED_REPORT_VERSION
      || !report.scope_policy
      || JSON.stringify(report.scope_policy.required_checks) !== JSON.stringify(
        PHASE4_REQUIRED_CHECKS,
      )
    ) {
      reasons.push("REPORT_SCOPE_POLICY_MISMATCH");
    }
    if (
      !report.execution_gate
      || report.execution_gate.execution_ready !== true
      || report.execution_gate.execution_ready_label !== "YES"
    ) {
      reasons.push("REPORT_EXECUTION_READY_NOT_YES");
    }
  }
  const validatedPlanFingerprint = report ? report.plan_fingerprint : null;
  if (!currentPlanFingerprint) reasons.push("CURRENT_PLAN_FINGERPRINT_MISSING");
  if (
    currentPlanFingerprint
    && validatedPlanFingerprint
    && currentPlanFingerprint !== validatedPlanFingerprint
  ) reasons.push("PLAN_FINGERPRINT_MISMATCH");
  if (stale) reasons.push("STALE_VALIDATION");
  const blockingReasons = [...new Set(reasons)];
  const executionReady = blockingReasons.length === 0;
  return {
    executionReady,
    executionReadyLabel: executionReady ? "YES" : "NO",
    status: executionReady ? "READY" : "BLOCKED",
    blockingReasons,
    planFingerprint: validatedPlanFingerprint,
    currentPlanFingerprint,
    stale: stale || blockingReasons.includes("PLAN_FINGERPRINT_MISMATCH"),
    semantic: PHASE4_CURRENT_SCOPE_READY_SEMANTIC,
  };
}
