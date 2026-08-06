// Phase 1C.1 pure status-snapshot adapter with no external runtime dependencies.

export const DEFAULT_STALE_TIMEOUT_MS = 1500;

function requireFiniteTimestamp(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number of milliseconds`);
  }
  return value;
}

function normalizeJointArray(value, label) {
  if (!Array.isArray(value)) {
    throw new TypeError(`${label} must be an array of exactly six radians values`);
  }
  if (value.length !== 6) {
    throw new RangeError(`${label} must contain exactly six radians values`);
  }

  return value.map((jointValue, index) => {
    if (typeof jointValue !== "number" || !Number.isFinite(jointValue)) {
      throw new TypeError(`${label}[${index}] must be a finite number in radians`);
    }
    return jointValue;
  });
}

export function normalizeDualArmStatusSnapshot(
  snapshot,
  receivedAtMs = Date.now(),
) {
  if (snapshot === null || typeof snapshot !== "object" || Array.isArray(snapshot)) {
    throw new TypeError("Status snapshot must be a non-null object");
  }
  if (!snapshot.left || typeof snapshot.left !== "object") {
    throw new TypeError("Status snapshot is missing left robot data");
  }
  if (!snapshot.right || typeof snapshot.right !== "object") {
    throw new TypeError("Status snapshot is missing right robot data");
  }

  return {
    left: normalizeJointArray(snapshot.left.joint, "snapshot.left.joint"),
    right: normalizeJointArray(snapshot.right.joint, "snapshot.right.joint"),
    receivedAtMs: requireFiniteTimestamp(receivedAtMs, "receivedAtMs"),
  };
}

export function isNormalizedSnapshotStale(
  snapshot,
  nowMs = Date.now(),
  staleTimeoutMs = DEFAULT_STALE_TIMEOUT_MS,
) {
  if (snapshot === null || typeof snapshot !== "object") {
    throw new TypeError("Normalized snapshot must be a non-null object");
  }
  const receivedAtMs = requireFiniteTimestamp(
    snapshot.receivedAtMs,
    "snapshot.receivedAtMs",
  );
  const checkedAtMs = requireFiniteTimestamp(nowMs, "nowMs");
  const timeoutMs = requireFiniteTimestamp(staleTimeoutMs, "staleTimeoutMs");
  if (timeoutMs < 0) {
    throw new RangeError("staleTimeoutMs must be greater than or equal to zero");
  }
  return checkedAtMs - receivedAtMs > timeoutMs;
}
