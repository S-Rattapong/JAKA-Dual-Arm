// Pure Phase-1 helpers. Matrix elements follow THREE.Matrix4 column-major
// storage. RPY uses R = Rz(yaw) * Ry(pitch) * Rx(roll), returned as rx/ry/rz.

export const MODEL_TCP_LINKS = Object.freeze({
  left: "left_J6",
  right: "right_J6",
});

export const MODEL_TCP_SOURCE = "URDF MODEL FK FROM MIRRORED JOINT STATE";
export const MODEL_TCP_SEMANTIC = "Digital Twin model TCP/flange";
export const MODEL_TCP_RPY_CONVENTION =
  "RPY radians; R = Rz(yaw) * Ry(pitch) * Rx(roll)";

const FEEDBACK_STATUSES = new Set(["LIVE", "STALE", "MISSING", "INVALID"]);

function requireFiniteArray(values, length, label) {
  if (
    !Array.isArray(values) ||
    values.length !== length ||
    !values.every(
      (value) => typeof value === "number" && Number.isFinite(value),
    )
  ) {
    throw new TypeError(`${label} must contain ${length} finite numbers`);
  }
  return [...values];
}

export function rotationMatrixToRpy(rotationRowMajor) {
  const r = requireFiniteArray(rotationRowMajor, 9, "rotation matrix");
  const r11 = r[0];
  const r21 = r[3];
  const r22 = r[4];
  const r23 = r[5];
  const r31 = r[6];
  const r32 = r[7];
  const r33 = r[8];
  const pitch = Math.asin(Math.max(-1, Math.min(1, -r31)));
  const cosPitch = Math.cos(pitch);
  let roll;
  let yaw;
  if (Math.abs(cosPitch) > 1e-10) {
    roll = Math.atan2(r32, r33);
    yaw = Math.atan2(r21, r11);
  } else {
    // Deterministic gimbal-lock branch: choose yaw=0 and solve roll.
    roll = Math.atan2(-r23, r22);
    yaw = 0;
  }
  return [roll, pitch, yaw];
}

export function matrix4ElementsToWorldPose(matrixElements) {
  const e = requireFiniteArray(matrixElements, 16, "THREE.Matrix4 elements");
  const rpyRad = rotationMatrixToRpy([
    e[0], e[4], e[8],
    e[1], e[5], e[9],
    e[2], e[6], e[10],
  ]);
  return {
    translationM: [e[12], e[13], e[14]],
    rpyRad,
  };
}

export function feedbackConnectionStatus(feedbackStatus) {
  if (!FEEDBACK_STATUSES.has(feedbackStatus)) {
    throw new TypeError("feedback status must be LIVE, STALE, MISSING, or INVALID");
  }
  return feedbackStatus;
}

export function deriveRobotStateAndAlert(side) {
  if (!side || typeof side !== "object") {
    throw new TypeError("robot status side must be an object");
  }
  const feedbackStatus = feedbackConnectionStatus(side.feedbackStatus);
  if (feedbackStatus === "INVALID") {
    return { robotState: "UNAVAILABLE", faultAlert: "INVALID FEEDBACK" };
  }
  if (feedbackStatus === "MISSING") {
    return { robotState: "UNAVAILABLE", faultAlert: "FEEDBACK MISSING" };
  }
  if (feedbackStatus === "STALE") {
    return { robotState: "UNAVAILABLE", faultAlert: "FEEDBACK STALE" };
  }

  const state = side.state;
  const fields = ["power_state", "servo_state", "motion_state", "collision_state"];
  if (
    !state ||
    fields.some((field) => !Number.isInteger(state[field]))
  ) {
    return { robotState: "UNAVAILABLE", faultAlert: "INVALID FEEDBACK" };
  }

  const robotState = state.motion_state === 0 ? "IDLE" : "MOVING";
  if (state.collision_state !== 0) {
    return { robotState, faultAlert: "COLLISION" };
  }
  if (state.power_state !== 1) {
    return { robotState, faultAlert: "POWER OFF" };
  }
  if (state.servo_state !== 1) {
    return { robotState, faultAlert: "SERVO OFF" };
  }
  return { robotState, faultAlert: "NONE" };
}
