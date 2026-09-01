// Pure helpers for smooth read-only Live Mirror visualization.
// No fetch, ROS, backend, or motion-command dependencies.

export const DEFAULT_VISUAL_SMOOTHING_TAU_MS = 18;
export const MAX_SMOOTHING_FRAME_DELTA_MS = 100;

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

export function smoothingAlpha(
  deltaTimeMs,
  tauMs = DEFAULT_VISUAL_SMOOTHING_TAU_MS,
) {
  const dt = Math.max(0, Math.min(
    finiteNumber(deltaTimeMs, "deltaTimeMs"),
    MAX_SMOOTHING_FRAME_DELTA_MS,
  ));
  const tau = finiteNumber(tauMs, "tauMs");
  if (tau <= 0) return 1;
  return 1 - Math.exp(-dt / tau);
}

export function shortestAngularDeltaRad(current, target) {
  const from = finiteNumber(current, "current");
  const to = finiteNumber(target, "target");
  const delta = to - from;
  return Math.atan2(Math.sin(delta), Math.cos(delta));
}

function smoothJointArray(current, target, alpha, label) {
  if (!Array.isArray(current) || current.length !== 6) {
    throw new RangeError(`${label}.current must contain six joints`);
  }
  if (!Array.isArray(target) || target.length !== 6) {
    throw new RangeError(`${label}.target must contain six joints`);
  }
  return current.map((value, index) => {
    const currentValue = finiteNumber(value, `${label}.current[${index}]`);
    const targetValue = finiteNumber(target[index], `${label}.target[${index}]`);
    return currentValue + alpha * shortestAngularDeltaRad(
      currentValue,
      targetValue,
    );
  });
}

export function smoothDualArmPose(
  currentPose,
  targetPose,
  deltaTimeMs,
  tauMs = DEFAULT_VISUAL_SMOOTHING_TAU_MS,
) {
  if (!currentPose || !targetPose) {
    throw new TypeError("currentPose and targetPose are required");
  }
  const alpha = smoothingAlpha(deltaTimeMs, tauMs);
  return {
    left: smoothJointArray(
      currentPose.left,
      targetPose.left,
      alpha,
      "left",
    ),
    right: smoothJointArray(
      currentPose.right,
      targetPose.right,
      alpha,
      "right",
    ),
    alpha,
  };
}

export function copyDualArmPose(pose) {
  return {
    left: [...pose.left],
    right: [...pose.right],
  };
}
