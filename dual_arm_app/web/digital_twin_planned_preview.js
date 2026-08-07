// Pure Phase 1D.1 helpers. Every pose and delta in this module uses radians.

function normalizeJointArray(side, values) {
  if (!Array.isArray(values) || values.length !== 6) {
    throw new TypeError(`${side} must be an array of exactly six radians values`);
  }
  if (!values.every(
    (value) => typeof value === "number" && Number.isFinite(value),
  )) {
    throw new TypeError(`${side} must contain exactly six finite numbers`);
  }
  return [...values];
}

export function normalizePlannedDualArmPose(values) {
  if (!values || typeof values !== "object" || Array.isArray(values)) {
    throw new TypeError("Planned pose must contain left and right joint arrays");
  }
  return {
    left: normalizeJointArray("left", values.left),
    right: normalizeJointArray("right", values.right),
  };
}

export function flattenDualArmPose(values) {
  const normalized = normalizePlannedDualArmPose(values);
  return [...normalized.left, ...normalized.right];
}

export function plannedMinusActualJointDelta(planned, actual) {
  const normalizedPlanned = normalizePlannedDualArmPose(planned);
  const normalizedActual = normalizePlannedDualArmPose(actual);
  return {
    left: normalizedPlanned.left.map(
      (plannedValue, index) => plannedValue - normalizedActual.left[index],
    ),
    right: normalizedPlanned.right.map(
      (plannedValue, index) => plannedValue - normalizedActual.right[index],
    ),
  };
}

export function maxAbsoluteJointDelta(planned, actual) {
  const delta = flattenDualArmPose(
    plannedMinusActualJointDelta(planned, actual),
  );
  return delta.reduce(
    (maximum, value) => Math.max(maximum, Math.abs(value)),
    0,
  );
}
